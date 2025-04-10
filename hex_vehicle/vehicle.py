#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################
from .generated.public_api_types_pb2 import RobotType

import time
import threading
from copy import deepcopy

SQRT_3 = 1.7320508075688772

class Vehicle:
    def __init__(self, base_type):

        # chassis type
        self.base_type = base_type

        if base_type == RobotType.Unknown:
            print("Vehicle base type is unknown.")
        elif base_type == RobotType.TripleOmniWheelLRDriver:
            print("Vehicle base type is TripleOmniWheelLRDriver.")
        elif base_type == RobotType.SteeringWheel3Module:
            print("Vehicle base type is SteeringWheel3Module.")
        elif base_type == RobotType.SteeringWheel4Module:
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
        self.lock = threading.Lock()
        self.has_new = False
        # 对于三角底盘，track_width为底盘圆心半径
        self.track_width = 0.3274

        # vehicle data read from websocket
        self.vehicle_state = None
        # motor data, not change id noumber
        self.wheel_speeds = []
        self.wheel_torques = []
        self.wheel_positions = []
        # unit: V
        self.read_battery_voltage = None
        self.read_remoter_info = None
        self.read_error = None

        # calculate data
        self.read_spd_x = None
        self.read_spd_y = None
        self.read_spd_r = None

        # The last time ros write the target speed.
        self.last_target_spd_write_time = None
        # unit: m/s
        self.target_spd_x: 0.0
        self.target_spd_y: 0.0
        # unit: rad/s.
        self.target_spd_r: 0.0

        self.target_torques = []

    def forward_kinematic(self, v: list) -> tuple:
        if self.base_type == RobotType.TripleOmniWheelLRDriver:
            if len(v)!= 3:
                raise ValueError("forward_kinematic: wheel_speeds length error")
        
            d_inv = 1.0 / self.track_width  / 3.0

            spd_x = -(SQRT_3 / 3.0) * v[0] + (SQRT_3 / 3.0) * v[2]
            spd_y = (1.0 / 3.0) * v[0] + -(2.0 / 3.0) * v[1] + (1.0 / 3.0) * v[2]
            spd_r = d_inv * v[0] + d_inv * v[1] + d_inv * v[2]
            return (spd_x, spd_y, spd_r)

    def inverse_kinematic(self, v_x, v_y, v_r) -> list:
        if self.base_type == RobotType.TripleOmniWheelLRDriver:
            d = self.track_width
            v = [0.0, 0.0, 0.0]
            v[0] = -(SQRT_3 / 2.0) * v_x + 0.5 * v_y + v_r * d
            v[1] = -v_y + v_r * d
            v[2] = (SQRT_3 / 2.0) * v_x + 0.5 * v_y + v_r * d