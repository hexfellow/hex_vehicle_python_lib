#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################
from .generated.public_api_types_pb2 import RobotType, BaseStatus
from typing import Type
import time
import threading
from typing import Tuple
from copy import deepcopy

SQRT_3 = 1.7320508075688772

class Vehicle:
    def __init__(self, base_type):

        # chassis type
        self.base_type = base_type
        self.motor_cnt = 0

        if base_type == RobotType.Unknown:
            print("Vehicle base type is unknown.")
        elif base_type == RobotType.TripleOmniWheelLRDriver:
            # 对于三角底盘，track_width为底盘圆心半径
            self.__track_width = 0.3274
            self.motor_cnt = 3
            print("Vehicle base type is TripleOmniWheelLRDriver.")
        elif base_type == RobotType.SteeringWheel3Module:
            self.motor_cnt = 6
            print("Vehicle base type is SteeringWheel3Module.")
        elif base_type == RobotType.SteeringWheel4Module:
            self.motor_cnt = 8
            print("Vehicle base type is SteeringWheel4Module.")
        elif base_type == RobotType.SteeringWheelSingleAModule:
            print("Vehicle base type is SteeringWheelSingleAModule.")
        elif base_type == RobotType.SteeringWheelSingleBModule:
            print("Vehicle base type is SteeringWheelSingleBModule.")
        elif base_type == RobotType.Mark1DiffBBDriver:
            print("Vehicle base type is Mark1DiffBBDriver.")
        elif base_type == RobotType.Mark1McnmBBDriver:
            print("Vehicle base type is Mark1McnmBBDriver.")
        elif base_type == RobotType.LinearLiftISVLDriver:
            print("Vehicle base type is LinearLiftISVLDriver.")
        elif base_type == RobotType.PureForwardOnly:
            print("Vehicle base type is PureForwardOnly.")

        # model data
        self.__data_lock = threading.Lock()
        self.__has_new = False
        # vehicle data read from websocket
        self.__base_status = None
        # motor data, not change id noumber
        self.__wheel_torques = []    # Nm
        self.__wheel_velocity = []   # m/s
        self.__wheel_positions = []  # m

        self.read_battery_voltage = None # unit: V
        self.read_remoter_info = None
        self.read_error = None

        self.__command_lock = threading.Lock()
        # The last time ros write the target.
        self.last_target_write_time = None
        self.target_torques = []

    def forward_kinematic(self, v: list) -> Tuple[float, float, float]:
        if self.base_type == RobotType.TripleOmniWheelLRDriver:
            if len(v)!= 3:
                raise ValueError("forward_kinematic: wheel_speeds length error")
            d_inv = 1.0 / self.__track_width  / 3.0
            spd_x = -(SQRT_3 / 3.0) * v[0] + (SQRT_3 / 3.0) * v[2]
            spd_y = (1.0 / 3.0) * v[0] + -(2.0 / 3.0) * v[1] + (1.0 / 3.0) * v[2]
            spd_r = d_inv * v[0] + d_inv * v[1] + d_inv * v[2]
            return (spd_x, spd_y, spd_r)
        else:
            raise NotImplementedError("forward_kinematic not implemented for base_type: ", self.base_type)

    def inverse_kinematic(self, v_x, v_y, v_r) -> list:
        if self.base_type == RobotType.TripleOmniWheelLRDriver:
            d = self.__track_width
            v = [0.0, 0.0, 0.0]
            v[0] = -(SQRT_3 / 2.0) * v_x + 0.5 * v_y + v_r * d
            v[1] = -v_y + v_r * d
            v[2] = (SQRT_3 / 2.0) * v_x + 0.5 * v_y + v_r * d
            return v
        else:
            raise NotImplementedError("inverse_kinematic not implemented for base_type: ", self.base_type)

    def set_vehicle_speed(self, target_spd_x, target_spd_y, target_spd_r):
        """
        calc wheel speed from vehicle speeds and set motor velocity.
        target_spd_x,target_spd_y   unit: m/s
        target_spd_r                unit: rad/s.
        """
        if self.base_type == RobotType.TripleOmniWheelLRDriver:
            v = self.inverse_kinematic(target_spd_x, target_spd_y, target_spd_r)
            self.set_motor_velocity(v)
        else:
            raise NotImplementedError("set_vehicle_speed not implemented for base_type: ", self.base_type)
        
        with self.__command_lock:
            self.last_target_write_time = time.time()
            self.set_vehicle_speed(v)

    def get_vehicle_speed(self) -> Tuple[float, float, float]:
        """ calc vehicle speed from wheel speeds """
        with self.__data_lock:
            self.__has_new = False
            wheel_speeds = self.__wheel_velocity
        if self.base_type == RobotType.TripleOmniWheelLRDriver:
            if len(wheel_speeds) != 3:
                return (0.0, 0.0, 0.0)
            return self.forward_kinematic(wheel_speeds)
        else:
            raise NotImplementedError("get_vehicle_speed not implemented for base_type: ", self.base_type)
        
    def update_wheel_data(self, base_status, wheel_torques: list, wheel_velocity: list, wheel_positions: list):
        with self.__data_lock:
            self.__has_new = True
            self.__base_status = base_status
            self.__wheel_torques = wheel_torques
            self.__wheel_velocity = wheel_velocity
            self.__wheel_positions = wheel_positions

    def set_motor_torque(self, torques: list):
        if len(torques) != self.motor_cnt:
            raise ValueError("set_motor_torque: torques length error")
        with self.__command_lock:
            self.last_target_write_time = time.time()
            self.target_torques = deepcopy(torques)

    def get_target_torque(self) -> list:
        with self.__command_lock:
            if self.last_target_write_time is None or time.time() - self.last_target_write_time > 0.2:
                return [0.0 for i in range(self.motor_cnt)]
            return deepcopy(self.target_torques)
        
    def set_motor_velocity(self, velocity: list):
        # calculate wheel torque from motor velocity
        if len(velocity) != self.motor_cnt:
            raise ValueError("set_motor_velocity: velocity length error")
        with self.__command_lock:
            self.last_target_write_time = time.time()

    def set_motor_position(self, position: list):
        # calculate wheel velocity from motor position
        if len(position) != self.motor_cnt:
            raise ValueError("set_motor_position: position length error")
        with self.__command_lock:
            self.last_target_write_time = time.time()

    def get_base_status(self) -> BaseStatus:
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__base_status)

    def get_motor_torque(self) -> list:
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__wheel_torques)
        
    def get_motor_position(self) -> list:
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__wheel_positions)
    
    def get_motor_velocity(self) -> list:
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__wheel_velocity)

    def has_new_data(self) -> bool:
        with self.__data_lock:
            return self.__has_new