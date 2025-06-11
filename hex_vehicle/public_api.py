#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################

from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .generated.public_api_types_pb2 import ParkingStopCategory as ParkingStopCategory
from .params import WsError, ProtocolError
from .utils import is_valid_ws_url, InvalidWSURLException, delay
from .vehicle import Vehicle
from .utils import log_warn, log_info, log_err, log_common

import string
from copy import deepcopy, copy
from numpy import pi as PI
import asyncio
import threading
import time

import websockets
from typing import Optional, Tuple
from websockets.exceptions import ConnectionClosed

MAX_CYCLES = 1000
RAW_DATA_LEN = 50

class PublicAPI:

    def __init__(self, ws_url: string, control_hz: int):
        self.__websocket = None
        try:
            self.__ws_url: string = is_valid_ws_url(ws_url)
        except InvalidWSURLException as e:
            log_err("Invalid WebSocket URL: " + str(e))

        if control_hz > MAX_CYCLES:
            log_warn(f"control_cycle is limit to {MAX_CYCLES}")
            control_hz = MAX_CYCLES
        self.__control_hz = control_hz

        self.__shutdown_event = asyncio.Event()
        self.__last_data_frame_time = None
        self.vehicle = None
        self.__last_warning_time = time.perf_counter()
        self.__loop_thread = threading.Thread(target=self.__loop_start, daemon=True)
        self.__api_data = []

        # init api
        self.__loop_thread.start()
        self.wait_init()

    def construct_control_message(self, control_mode: str,
                                  data: list) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a control message.
        @patams:
            control_mode: "speed" or "torque"
            data: a list of target values
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        motor_targets = public_api_types_pb2.MotorTargets()
        single_motor_target = public_api_types_pb2.SingleMotorTarget()

        if control_mode == "speed":
            for target in data:
                single_motor_target.speed = target
                motor_targets.targets.append(deepcopy(single_motor_target))
        elif control_mode == "torque":
            for target in data:
                single_motor_target.torque = target
                motor_targets.targets.append(deepcopy(single_motor_target))
        else:
            raise ValueError("construct_down_message: control_mode error")

        base_command.motor_targets.CopyFrom(motor_targets)
        msg.base_command.CopyFrom(base_command)
        return msg

    def construct_init_message(self) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a init message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        base_command.api_control_initialize = True

        msg.base_command.CopyFrom(base_command)
        return msg

    def construct_clear_parking_stop_message(self):
        """
        @brief: For constructing a clear_parking_stop message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        base_command.clear_parking_stop = True
        msg.base_command.CopyFrom(base_command)
        return msg

    def construct_set_parking_stop_message(
            self, reason: str, category: ParkingStopCategory,
            is_remotely_clearable: bool) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a set_parking_stop message.
        @params:
            reason: what caused the parking stop
            category: parking stop category, values can be :
                ParkingStopCategory.EmergencyStopButton,
                ParkingStopCategory.MotorHasError,
                ParkingStopCategory.BatteryFail
                ParkingStopCategory.GamepadTriggered,
                ParkingStopCategory.UnknownParkingStopCategory,
                ParkingStopCategory.APICommunicationTimeout
            is_remotely_clearable: whether the parking stop can be cleared remotely
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        parking_stop_category = category

        parking_stop_detail.reason = reason
        parking_stop_detail.category = parking_stop_category
        parking_stop_detail.is_remotely_clearable = is_remotely_clearable

        base_command.trigger_parking_stop.CopyFrom(parking_stop_detail)
        msg.base_command.CopyFrom(base_command)
        return msg

    async def send_down_message(self, data: public_api_down_pb2.APIDown):
        msg = data.SerializeToString()
        # if self.__websocket is None:
        #     raise AttributeError("send_down_message: websocket tx is None")
        # else:
        #     await self.__websocket.send(msg)

    async def __connect_ws(self):
        """
        @brief: Connect to the WebSocket server, used by "initialize" function.
        """
        try:
            self.__websocket = await websockets.connect(self.__ws_url,
                                                          ping_interval=20,
                                                          ping_timeout=60,
                                                          close_timeout=5)
        except Exception as e:
            log_err(f"Failed to open WebSocket connection: {e}")
            log_common(
                "Public API haved exited, please check your network connection and restart the server again."
            )
            exit(1)

    async def __reconnect(self):
        retry_count = 0
        max_retries = 5
        base_delay = 1

        while retry_count < max_retries:
            try:
                if self.__websocket:
                    await self.__websocket.close()
                self.__websocket = await websockets.connect(self.__ws_url,
                                                          ping_interval=20,
                                                          ping_timeout=60,
                                                          close_timeout=5)
                return
            except Exception as e:
                delay = base_delay * (2**retry_count)
                log_warn(
                    f"Reconnect failed (attempt {retry_count+1}): {e}, retrying in {delay}s"
                )
                await asyncio.sleep(delay)
                retry_count += 1
        raise ConnectionError("Maximum reconnect retries exceeded")

    async def __capture_data_frame(self) -> Optional[public_api_up_pb2.APIUp]:
        """
        @brief: Continuously monitor WebSocket connections until:
        1. Received a valid binary Protobuf message
        2. Protocol error occurred
        3. Connection closed
        4. No data due to timeout
        
        @params:
            websocket: 已建立的 WebSocket 连接对象
            
        返回:
            base_backend.APIUp 对象 或 None
        """
        while True:
            try:
                # 设置 3 秒超时接收
                message = await asyncio.wait_for(self.__websocket.recv(),
                                                 timeout=3.0)
                # 仅处理二进制消息
                if isinstance(message, bytes):
                    try:
                        # Protobuf 反序列化
                        api_up = public_api_up_pb2.APIUp()
                        api_up.ParseFromString(message)

                        if not api_up.IsInitialized():
                            raise ProtocolError("Incomplete message")
                        # 过滤其他类型的数据
                        elif api_up.base_status.IsInitialized():
                            return api_up
                        
                    except Exception as e:
                        log_err(f"Protobuf encode fail: {e}")
                        raise ProtocolError("Invalid message format") from e

                elif isinstance(message, str):
                    log_common(f"ignore string message: {message[:50]}...")
                    continue

            except asyncio.TimeoutError:
                log_err("No data received for 3 seconds")
                continue

            except ConnectionClosed as e:
                log_err(f"Connection closed (code: {e.code}, reason: {e.reason})")
                try:
                    await self.__reconnect()
                    continue
                except ConnectionError as e:
                    log_err(f"Reconnect failed: {e}")
                    self.close()

            except Exception as e:
                log_err(f"Unknown error: {str(e)}")
                raise WsError("Unexpected error") from e

    async def __capture_first_frame(self):
        api_up = await self.__capture_data_frame()
        try:
            self.vehicle = Vehicle(api_up.robot_type)
            log_info("\033[32m**Vehicle is ready to use**\033[0m")
        except Exception as e:
            log_err(f"\033[31mFailed to initialize vehicle: {e}\033[0m")
            self.close()

    def __loop_start(self):
        self.__loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.__loop)
        self.__loop.run_until_complete(self.__main_loop())

    def close(self):
        if self.__loop and self.__loop.is_running():
            
            log_warn("\033[33mHexVehicle API is closing...\033[0m")
            asyncio.run_coroutine_threadsafe(self.__async_close(), self.__loop)
            # try:
            #     self.__loop_thread.join(timeout=1)
            # finally:
            #     pass

    async def __async_close(self):
        if self.__websocket:
            await self.__websocket.close()
        self.__shutdown_event.set()

    async def __main_loop(self):
        log_common("HexVehicle Api started.")
        await self.__connect_ws()
        await self.__capture_first_frame()
        task1 = asyncio.create_task(self.__periodic_state_checker())
        task2 = asyncio.create_task(self.__periodic_data_parser())
        self.__tasks = [task1, task2]
        await self.__shutdown_event.wait()
        for task in self.__tasks:
            task.cancel()
        await asyncio.gather(*self.__tasks, return_exceptions=True)
        log_err("HexVehicle api main_loop exited.")

    def _parse_raw_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[list, list, list]:
        vv = []
        tt = []
        pp = []
        try:
            pass
            for motor_status in api_up.base_status.motor_status:
                # parser motor data
                # TODO: also have other data can be parser
                torque = motor_status.torque  #Nm
                speed = motor_status.speed
                position = (motor_status.position % motor_status.pulse_per_rotation) / motor_status.pulse_per_rotation * (2.0 * PI) - PI  # radian

                tt.append(torque)
                vv.append(speed)
                pp.append(position)
        except Exception as e:
            log_err("parse_raw_data error: no motor_status data.")
        return (tt, vv, pp)

    async def __periodic_data_parser(self):
        """
        Capture and parse data from WebSocket connection.
        """
        while True:
            api_up = await self.__capture_data_frame()

            if len(self.__api_data) >= RAW_DATA_LEN:
                self.__api_data.pop(0)
            self.__api_data.append(api_up)

            tt,v,p = self._parse_raw_data(api_up)
            self.__last_data_frame_time = time.perf_counter()
            self.vehicle.update_wheel_data(api_up.base_status, tt, v, p)
                
    async def __periodic_state_checker(self):
        cycle_time = 1000.0 / self.__control_hz
        start_time = time.perf_counter()
        self.__last_warning_time = start_time
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            # if have not data received, we should not do anything
            if self.__last_data_frame_time is None:
                continue

            base_status = self.vehicle.get_base_status()

            if base_status.parking_stop_detail != public_api_types_pb2.ParkingStopDetail():
                if start_time - self.__last_warning_time > 1.0:
                    log_err(
                        f"emergency stop: {base_status.parking_stop_detail}"
                    )
                    self.__last_warning_time = start_time

            if base_status.api_control_initialized == False:
                msg = self.construct_init_message()
                await self.send_down_message(msg)

            # Sending control message
            torque = self.vehicle.get_target_torque()
            msg = self.construct_control_message(
                "torque", torque)
            await self.send_down_message(msg)

    def is_api_exit(self) -> bool:
        return self.__loop.is_closed()
    
    def wait_init(self):
        try:
            while True:
                if self.vehicle is not None:
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.close()
            exit(1)

    def _get_raw_data(self) -> Tuple[public_api_up_pb2.APIUp, int]:
        """
        Retrieve the oldest raw data in the buffer. 
        The maximum length of this buffer is RAW-DATA_LEN.
        You can use '_parse_raw_data' to parse the raw data.
        """
        if len(self.__api_data) == 0:
            return (None, 0)
        return (self.__api_data.pop(0), len(self.__api_data))