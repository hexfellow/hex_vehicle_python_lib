#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################

from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from .generated.public_api_types_pb2 import EmergencyStopCategory as EmergencyStopCategory

from .utils import is_valid_ws_url, InvalidWSURLException
from .vehicle import Vehicle

import string
from copy import deepcopy
from numpy import PI
import asyncio
import threading
import time

import websockets

class PublicAPI:
    def __init__(self, ws_url: string):

        try:
            self.ws_url: string = is_valid_ws_url(ws_url)
        except InvalidWSURLException as e:
            print(e)

        self.tx = None
        self.rx = None
        self.loop = threading.Thread(target=self.loop_start)
        self.stop_event = threading.Event()
        self.last_data_frame_time = None
        self.vehicle = None

    def construct_control_message(self, control_mode: str, data: list) -> public_api_down_pb2.APIDown:
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

    def construct_clear_emergency_stop_message(self):
        """
        @brief: For constructing a clear_emergency_stop message.
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        base_command.clear_emergency_stop = True
        msg.base_command.CopyFrom(base_command)
        return msg

    def construct_set_emergency_stop_message(self, reason: str, category: EmergencyStopCategory, is_remotely_clearable: bool) -> public_api_down_pb2.APIDown:
        """
        @brief: For constructing a set_emergency_stop message.
        @params:
            reason: what caused the emergency stop
            category: emergency stop category, values can be :
                EmergencyStopCategory.EmergencyStopButton,
                EmergencyStopCategory.MotorHasError,
                EmergencyStopCategory.BatteryFail
                EmergencyStopCategory.GamepadTriggered,
                EmergencyStopCategory.UnknownEmergencyStopCategory,
                EmergencyStopCategory.APICommunicationTimeout
            is_remotely_clearable: whether the emergency stop can be cleared remotely
        """
        msg = public_api_down_pb2.APIDown()
        base_command = public_api_types_pb2.BaseCommand()
        emergency_stop_detail = public_api_types_pb2.EmergencyStopDetail()
        emergency_stop_category = category

        emergency_stop_detail.reason = reason
        emergency_stop_detail.category = emergency_stop_category
        emergency_stop_detail.is_remotely_clearable = is_remotely_clearable

        base_command.trigger_emergency_stop.CopyFrom(emergency_stop_detail)
        msg.base_command.CopyFrom(base_command)
        return msg

    async def send_down_message(self, data: public_api_down_pb2.APIDown):
        msg = data.SerializeToString()
        if self.tx is None:
            raise AttributeError("send_down_message: websocket tx is None")
        else:
            await self.tx.send(msg)

    async def connect_ws(self):
        """
        @brief: Connect to the WebSocket server, used by "initialize" function.
        """
        try:
            async with websockets.connect(self.ws_url) as websocket:
                print("Connected to WebSocket server")
                return websocket
        except Exception as e:
            print(f"Failed to open WebSocket connection: {e}")
            raise

    async def init_websocket(self):
        """
        Initialize the Public API by connecting to the WebSocket server.
        :return: None
        """
        websocket = await self.connect_ws()
        self.tx = websocket.send
        self.rx = websocket.recv
        print("tx get:", self.tx)
        print("rx get:", self.rx)


    def loop_start(self):
        asyncio.run(self.main_loop())

    def close(self):
        print("Received Ctrl+C, closing...")
        self.stop_event.set()

    async def capture_data_frame(self) -> public_api_up_pb2.APIUp:
        timeout = 0.2 # seconds
        while True:
            try:
                if self.stop_event.is_set():
                    break

                message = await asyncio.wait_for(self.rx(), timeout)
                self.last_data_frame_time = time.time()
                api_up_message = public_api_up_pb2.APIUp()
                api_up_message.ParseFromString(message)
                
            except asyncio.TimeoutError:
                print(f"Timeout occurred while waiting for message (after {timeout} seconds).")
                continue


    async def capture_first_frame(self):
        api_up = await self.capture_data_frame()
        self.vehicle = Vehicle(api_up.base_status.type)
        print("**Vehicle is ready to use**")


    async def main_loop(self):
        print("Public API started.")
        await self.init_websocket()
        await self.capture_first_frame()

        while True:
            if self.stop_event.is_set():
                break
            else:
                api_up = await self.capture_data_frame()
                print(f"Received message: {api_up}")
 
                vv = []
                tt = []
                pp = []

                for motor_status in self.api_up.motor_status:
                    # parser motor data
                    # TODO: also have other data can be parser
                    torque = motor_status.torque   #Nm
                    speed = motor_status.speed * motor_status.wheel_radius  #m/s
                    position = motor_status.position / motor_status.pulse_per_rotation * 2.0 * PI * motor_status.wheel_radius  #m

                    tt.push(torque)
                    vv.push(speed)
                    pp.push(position)

                # 调整轮子编号
                v = []
                p = []

                v[0] = vv[1]
                v[1] = vv[0]
                v[2] = vv[2]

                p[0] = pp[1]
                p[1] = pp[0]
                p[2] = pp[2]

                with self.vehicle.lock:
                    self.vehicle.vehicle_state = api_up.base_status
                    self.vehicle.read_spd_x, self.vehicle.read_spd_y, self.vehicle.read_spd_z = self.vehicle.forward_kinematic(v)
                    self.vehicle.wheel_speeds = vv
                    self.vehicle.wheel_torques = tt
                    self.vehicle.wheel_positions = pp
                

        print("Public API stopped. Exiting...")


    def periodic_state_checker(self):
        # TODO: Write sending logic
        pass

