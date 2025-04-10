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

import string
from copy import deepcopy
from numpy import pi as PI
import asyncio
import threading
import time

import websockets
from typing import Optional, Tuple
import logging
from websockets.exceptions import ConnectionClosed

MAX_CYCLES = 1000

class PublicAPI:

    def __init__(self, ws_url: string, control_hz: int):
        self.__websocket = None
        try:
            self.__ws_url: string = is_valid_ws_url(ws_url)
        except InvalidWSURLException as e:
            print(e)

        if control_hz > MAX_CYCLES:
            print(f"control_cycle is limit to {MAX_CYCLES}")
            control_hz = MAX_CYCLES
        self.control_hz = control_hz

        self.__shutdown_event = asyncio.Event()
        self.__last_data_frame_time = None
        self.__vehicle = None
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
        if self.__websocket is None:
            raise AttributeError("send_down_message: websocket tx is None")
        else:
            await self.__websocket.send(msg)

    async def __connect_ws(self):
        """
        @brief: Connect to the WebSocket server, used by "initialize" function.
        """
        try:
            async with websockets.connect(self.__ws_url,
                                          ping_interval=20,
                                          ping_timeout=60,
                                          close_timeout=5) as websocket:
                print("Connected to WebSocket server.")
                self.__websocket = websocket
        except Exception as e:
            print(f"Failed to open WebSocket connection: {e}")
            print(
                "Public API haved exited, please check your network connection and restart the server again."
            )
            exit(1)

    def __loop_start(self):
        self.__loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.__loop)
        self.__loop.run_until_complete(self.__main_loop())

    def close(self):
        print("Received Ctrl+C, closing...")
        if self.__loop and self.__loop.is_running():
            asyncio.run_coroutine_threadsafe(self.__async_close(), self.__loop)
        self.__loop_thread.join(timeout=1)

    async def __async_close(self):
        if self.__websocket:
            await self.__websocket.close()
        self.__shutdown_event.set()

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
                        return api_up
                    except Exception as e:
                        logging.error(f"Protobuf encode fail: {e}")
                        raise ProtocolError("Invalid message format") from e

                elif isinstance(message, str):
                    logging.debug(f"ignore string message: {message[:50]}...")
                    continue

            except asyncio.TimeoutError:
                logging.warning("No data received for 3 seconds")
                continue

            except ConnectionClosed as e:
                self.__websocket = None
                logging.info(f"Connection closed (code: {e.code}, reason: {e.reason})")
                await self.__reconnect()
                continue

            except Exception as e:
                logging.error(f"Unknown error: {str(e)}")
                raise WsError("Unexpected error") from e

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
                logging.warning(
                    f"Reconnect failed (attempt {retry_count+1}): {e}, retrying in {delay}s"
                )
                await asyncio.sleep(delay)
                retry_count += 1
        raise ConnectionError("Maximum reconnect retries exceeded")

    async def __capture_first_frame(self):
        api_up = await self.__capture_data_frame()
        self.__vehicle = Vehicle(api_up.robot_type)
        print("**Vehicle is ready to use**")

    async def __stop_websocket(self):
        self.__websocket.close()

    async def __main_loop(self):
        print("Public API started.")
        await self.__connect_ws()
        await self.__capture_first_frame()
        task1 = asyncio.create_task(self.__periodic_state_checker())
        task2 = asyncio.create_task(self.__periodic_data_parser())
        self.__tasks = [task1, task2]
        await self.__shutdown_event.wait()
        for task in self.__tasks:
            task.cancel()
        await asyncio.gather(*self.__tasks, return_exceptions=True)
        print("api exited gracefully.")


    async def __periodic_data_parser(self):
        while True:
            api_up = await self.__capture_data_frame()
            if len(self.__api_data) >= 50:
                self.__api_data.pop(0)
            self.__api_data.append(api_up)

            vv = []
            tt = []
            pp = []

            for motor_status in api_up.motor_status:
                # parser motor data
                # TODO: also have other data can be parser
                torque = motor_status.torque  #Nm
                speed = motor_status.speed * motor_status.wheel_radius  #m/s
                position = motor_status.position / motor_status.pulse_per_rotation * 2.0 * PI * motor_status.wheel_radius  #m

                tt.append(torque)
                vv.append(speed)
                pp.append(position)

            # 调整轮子编号
            v = deepcopy(vv)
            p = deepcopy(pp)

            # v[0] = vv[1]
            # v[1] = vv[0]
            # v[2] = vv[2]

            # p[0] = pp[1]
            # p[1] = pp[0]
            # p[2] = pp[2]

            with self.__vehicle.lock:
                self.__vehicle.has_new = True
                self.__vehicle.vehicle_state = api_up.base_status
                result = self.__vehicle.forward_kinematic(v)
                if result is not None:
                    self.__vehicle.read_spd_x, self.__vehicle.read_spd_y, self.__vehicle.read_spd_z = result
                self.__vehicle.wheel_speeds = vv
                self.__vehicle.wheel_torques = tt
                self.__vehicle.wheel_positions = pp

    async def __periodic_state_checker(self):
        cycle_time = 1000.0 / self.control_hz
        start_time = time.perf_counter()
        while True:
            await delay(start_time, cycle_time)
            start_time = time.perf_counter()

            # if have not data received, we should not do anything
            if self.__last_data_frame_time is None:
                continue

            vehicle = None
            with self.__vehicle.lock:
                vehicle = deepcopy(self.__vehicle)

            if vehicle.vehicle_state is None:
                continue
            else:
                if vehicle.vehicle_state.parking_stop_detail is not None:
                    if start_time - self.__last_warning_time > 500:
                        print(
                            f"tate-emergency-stop: {vehicle.vehicle_state.parking_stop_detail}"
                        )
                        self.__last_data_frame_time = start_time

            if vehicle.vehicle_state.api_control_initialized == False:
                msg = self.construct_init_message()
                self.send_down_message(msg)
            else:
                if vehicle.last_target_spd_write_time is None:
                    msg = self.construct_control_message(
                        "speed", [0.0, 0.0, 0.0])
                else:
                    v = self.__vehicle.inverse_kinematic(
                        self.__vehicle.target_spd_x, self.__vehicle.target_spd_y,
                        self.__vehicle.target_spd_r)
                    msg = self.construct_control_message("speed", v)
                    self.send_down_message(msg)

    def is_api_exit(self) -> bool:
        return self.__loop.is_closed()
    
    def wait_init(self):
        try:
            while True:
                if self.__vehicle is not None:
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.close()
            exit(1)

    def set_speed(self, target_spd_x, target_spd_y, target_spd_r):
        with self.__vehicle.lock:
            self.__vehicle.last_target_spd_write_time = time.time()
            self.__vehicle.target_spd_x = target_spd_x
            self.__vehicle.target_spd_y = target_spd_y
            self.__vehicle.target_spd_r = target_spd_r

    def set_torque(self, torques: list):
        with self.__vehicle.lock:
            self.__vehicle.last_target_spd_write_time = time.time()
            self.__vehicle.target_torques = torques

    def get_speed(self) -> Tuple[float, float, float]:
        with self.__vehicle.lock:
            self.__vehicle.has_new = False
            if self.__vehicle.read_spd_x is None:
                return (0.0, 0.0, 0.0)
            else:
                return (deepcopy(self.__vehicle.target_spd_x), 
                        deepcopy(self.__vehicle.target_spd_y), 
                        deepcopy(self.__vehicle.target_spd_r))
        
    def get_motor_torque(self) -> list:
        with self.__vehicle.lock:
            self.__vehicle.has_new = False
            return deepcopy(self.__vehicle.wheel_torques)
        
    def get_motor_position(self) -> list:
        with self.__vehicle.lock:
            self.__vehicle.has_new = False
            return deepcopy(self.__vehicle.wheel_positions)
    
    def get_motor_velocity(self) -> list:
        with self.__vehicle.lock:
            self.__vehicle.has_new = False
            return deepcopy(self.__vehicle.wheel_speeds)
        
    def _get_raw_data(self) -> Tuple[public_api_up_pb2.APIUp, int]:
        if len(self.__api_data) == 0:
            return (None, 0)
        return (self.__api_data.pop(0), len(self.__api_data))
    
    def has_new_data(self) -> bool:
        with self.__vehicle.lock:
            return self.__vehicle.has_new