#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################

from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .error_type import WsError, ProtocolError
from .utils import is_valid_ws_url, InvalidWSURLException, delay
from .vehicle import Vehicle
from .utils import log_warn, log_info, log_err, log_common

import string
from copy import deepcopy, copy
from math import pi as PI
import asyncio
import threading
import time

import websockets
from typing import Optional, Tuple
from websockets.exceptions import ConnectionClosed

MAX_CYCLES = 1000
RAW_DATA_LEN = 50

class PublicAPI:

    def __init__(self, ws_url: str, control_hz: int, control_mode: str = "speed"):
        self.__websocket = None
        try:
            self.__ws_url: str = is_valid_ws_url(ws_url)
        except InvalidWSURLException as e:
            log_err("Invalid WebSocket URL: " + str(e))

        if control_mode not in ["speed"]:
            raise ValueError("control_mode must be 'speed'")
        self.__control_mode = control_mode

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

    def construct_wheel_control_message(self, control_mode: str,
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

    def construct_simple_control_message(self,
                                         data: Tuple[float, float, float]) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a simple control message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        simple_base_move_command = public_api_types_pb2.SimpleBaseMoveCommand()
        xyz_speed = public_api_types_pb2.XyzSpeed()
        xyz_speed.speed_x = data[0]
        xyz_speed.speed_y = data[1]
        xyz_speed.speed_z = data[2]
        simple_base_move_command.xyz_speed.CopyFrom(xyz_speed)
        base_command.simple_move_command.CopyFrom(simple_base_move_command)
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
            self, reason: str, category: int,
            is_remotely_clearable: bool) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a set_parking_stop message.
        @params:
            reason: what caused the parking stop
            category: parking stop category, values can be :
                0: EmergencyStopButton,
                1: MotorHasError,
                2: BatteryFail
                3: GamepadTriggered,
                4: UnknownParkingStopCategory,
                5: APICommunicationTimeout
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
            websocket: Established WebSocket connection object
            
        @return:
            base_backend.APIUp object or None
        """
        while True:
            try:
                # Check if websocket is connected
                if self.__websocket is None:
                    log_err("WebSocket is not connected")
                    await asyncio.sleep(1)
                    continue
                
                # Timeout
                message = await asyncio.wait_for(self.__websocket.recv(),
                                                 timeout=3.0)
                # Only process binary messages
                if isinstance(message, bytes):
                    try:
                        # Protobuf parse
                        api_up = public_api_up_pb2.APIUp()
                        api_up.ParseFromString(message)

                        if not api_up.IsInitialized():
                            raise ProtocolError("Incomplete message")
                        # Filter other type message
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
        # init parameter
        api_up = public_api_up_pb2.APIUp()
        api_up.robot_type = public_api_types_pb2.RobotType.RtUnknown
        # wait for robot type
        while api_up.robot_type == public_api_types_pb2.RobotType.RtUnknown:
            api_up = await self.__capture_data_frame()
        # try to init vehicle
        try:
            motor_cnt = len(api_up.base_status.motor_status)
            log_common(f"motor_cnt: {motor_cnt}")
            self.vehicle = Vehicle(api_up.robot_type, self.__control_mode, motor_cnt)
            log_info("**Vehicle is ready to use**")
        except Exception as e:
            log_err(f"Failed to initialize vehicle: {e}")
            self.close()

    def __loop_start(self):
        self.__loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.__loop)
        self.__loop.run_until_complete(self.__main_loop())

    def close(self):
        if self.__loop and self.__loop.is_running():
            
            log_warn("HexVehicle API is closing...")
            asyncio.run_coroutine_threadsafe(self.__async_close(), self.__loop)

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

    def _parse_wheel_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[list, list, list]:
        vv = []
        tt = []
        pp = []
        ee = []
        try:
            for motor_status in api_up.base_status.motor_status:
                # parser motor data
                # TODO: also have other data can be parser
                torque = motor_status.torque  #Nm
                tt.append(torque)
                speed = motor_status.speed  #m/s
                vv.append(speed)
                position = (motor_status.position % motor_status.pulse_per_rotation) / motor_status.pulse_per_rotation * (2.0 * PI) - PI  # radian
                pp.append(position)
                error = motor_status.error
                ee.append(error)

        except Exception as e:
            log_err("parse_raw_data error: no motor_status data.")

        self.vehicle.update_wheel_data(api_up.base_status, tt, vv, pp, ee)
        self.__last_data_frame_time = time.perf_counter()
        
        return (tt, vv, pp)

    def _parse_vehicle_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        '''
        parse the vehilce data from odometry.
        '''
        (spd_x, spd_y, spd_r) = (0.0, 0.0, 0.0)
        (pos_x, pos_y, pos_r) = (0.0, 0.0, 0.0)
        try:
            if api_up.base_status.estimated_odometry.IsInitialized():
                spd_x = api_up.base_status.estimated_odometry.speed_x
                spd_y = api_up.base_status.estimated_odometry.speed_y
                spd_r = api_up.base_status.estimated_odometry.speed_z
                pos_x = api_up.base_status.estimated_odometry.pos_x
                pos_y = api_up.base_status.estimated_odometry.pos_y
                pos_r = api_up.base_status.estimated_odometry.pos_z
        except Exception as e:
            pass
            # log_err("parse_vehicle_data error: no estimated_odometry data.")
        return (spd_x, spd_y, spd_r), (pos_x, pos_y, pos_r)

    async def __periodic_data_parser(self):
        """
        Capture and parse data from WebSocket connection.
        """
        while True:
            api_up = await self.__capture_data_frame()

            if len(self.__api_data) >= RAW_DATA_LEN:
                self.__api_data.pop(0)
            self.__api_data.append(api_up)

            # parse wheel data
            self._parse_wheel_data(api_up)
            
            # parse vehicle data
            (spd_x, spd_y, spd_r), (pos_x, pos_y, pos_r) = self._parse_vehicle_data(api_up)
            self.vehicle.update_vehicle_data((spd_x, spd_y, spd_r), (pos_x, pos_y, pos_r))


    
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

            # Check if vehicle is initialized
            if self.vehicle is None:
                continue
            base_status = self.vehicle.get_base_status()

            # check if have parking stop
            if base_status.parking_stop_detail != public_api_types_pb2.ParkingStopDetail():
                if start_time - self.__last_warning_time > 1.0:
                    log_err(
                        f"emergency stop: {base_status.parking_stop_detail}"
                    )
                    self.__last_warning_time = start_time
                # try to clear parking stop
                msg = self.construct_clear_parking_stop_message()
                await self.send_down_message(msg)

            # check if vehicle is initialized
            if base_status.api_control_initialized == False:
                msg = self.construct_init_message()
                await self.send_down_message(msg)

            # Sending control message. Check if simple control mode is enabled.
            if self.vehicle._simple_control_mode == False:
                targets = self.vehicle.get_motor_targets()
                msg = self.construct_wheel_control_message(
                    self.__control_mode, targets)
                await self.send_down_message(msg)
            elif self.vehicle._simple_control_mode == True:
                targets = self.vehicle.get_target_vehicle_speed()
                msg = self.construct_simple_control_message(targets)
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
        You can use '_parse_wheel_data' to parse the raw data.
        """
        if len(self.__api_data) == 0:
            return (None, 0)
        return (self.__api_data.pop(0), len(self.__api_data))