#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################
from .generated import public_api_types_pb2
from .generated.public_api_types_pb2 import RobotType, BaseStatus
from .utils import log_warn, log_info, log_err, log_common
from typing import Type
import time
import threading
from typing import Tuple
from copy import deepcopy
import numpy as np

POS_KP: float = 1.0
POS_KD: float = 0.0
VEL_KP: float = 1.0
VEL_KD: float = 0.0

TIME_OUT: float = 0.2


class Vehicle:
    def __init__(self, base_type, control_mode, motor_cnt: int = 8):

        # chassis type
        self.base_type = base_type
        self.motor_cnt = motor_cnt

        if base_type == RobotType.RtTripleOmniWheelLRDriver:
            self.motor_cnt = 3
            log_info("Vehicle base type is RtTripleOmniWheelLRDriver.")
        elif base_type == RobotType.RtPcwVehicle:
            log_info("Vehicle base type is RtPcwVehicle.")
        elif base_type == RobotType.RtCustomPcwVehicle:
            log_info("Vehicle base type is RtCustomPcwVehicle.")
        elif base_type == RobotType.RtMark1DiffBBDriver:
            log_info("Vehicle base type is RtMark1DiffBBDriver.")
        elif base_type == RobotType.RtMark1McnmBBDriver:
            log_info("Vehicle base type is RtMark1McnmBBDriver.")
        else:
            raise ValueError(f"Unsupported robot type: {base_type}")

        self.__data_lock = threading.Lock()
        self.last_data_time = None
        self.__has_new = False
        # vehicle data read from websocket
        self.__base_status = None
        # motor data, not change id noumber
        self.__wheel_torques = []    # Nm
        self.__wheel_velocity = []   # rad/s
        self.__wheel_positions = []  # radian
        self.__wheel_errors = []
        # vehicle data
        self.__vehicle_speed = (0.0, 0.0, 0.0)
        self.__vehicle_position = (0.0, 0.0, 0.0)
        # Initialize origin position as identity transformation matrix
        self.__vehicle_origin_position = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        # imu data
        self.has_imu = False
        self.imu_acc = (0.0, 0.0, 0.0)
        self.imu_angular_velocity = (0.0, 0.0, 0.0)
        self.imu_quaternion = (0.0, 0.0, 0.0, 0.0)

        self.__read_battery_voltage = None # unit: V
        self.__read_remoter_info = None
        self.__read_error = None

        # control mode
        self.__control_mode = control_mode
        # if you want to change or read the target, you need to lock the command_lock
        self.__command_lock = threading.Lock()
        # The last time write the target, if the target is not written for 0.2s, the target will be set to 0.0
        self.last_target_write_time = None

        # target torque or velocity for motor
        self.__motor_targets = []
        # simple control mode, true means control vehicle speed, false means control motor target.
        self._simple_control_mode = None
        # target vehicle speed
        self.__target_vehicle_speed = (0.0, 0.0, 0.0)

         # pid params
        self.last_error = None

    def set_target_vehicle_speed(self, target_spd_x, target_spd_y, target_spd_r):
        """
        Use simple control mode to set vehicle speed.
        target_spd_x,target_spd_y   unit: m/s
        target_spd_r                unit: rad/s.
        """
        if self._simple_control_mode == False:
            raise NotImplementedError("set_vehicle_speed not implemented for _simple_control_mode: False")
        elif self._simple_control_mode == None:
            self._simple_control_mode = True

        if self.base_type == RobotType.RtPcwVehicle or self.base_type == RobotType.RtCustomPcwVehicle:
            with self.__command_lock:
                self.last_target_write_time = time.time()
                self.__target_vehicle_speed = (target_spd_x, target_spd_y, target_spd_r)
        else:
            raise NotImplementedError("set_vehicle_speed not implemented for base_type: ", self.base_type)

    def get_target_vehicle_speed(self) -> Tuple[float, float, float]:
        """ get target vehicle speed """
        with self.__command_lock:
            if self.last_target_write_time is None or time.time() - self.last_target_write_time > TIME_OUT:
                return (0.0, 0.0, 0.0)
            return deepcopy(self.__target_vehicle_speed)
        
    def get_vehicle_speed(self) -> Tuple[float, float, float]:
        """ get vehicle speed """
        if self.base_type == RobotType.RtPcwVehicle or self.base_type == RobotType.RtCustomPcwVehicle:
            with self.__data_lock:
                self.__has_new = False
                return deepcopy(self.__vehicle_speed)
        else:
            raise NotImplementedError("get_vehicle_speed not implemented for base_type: ", self.base_type)

    def get_vehicle_position(self) -> Tuple[float, float, float]:
        """ get vehicle position
        Odometry position, unit: m
        """
        if self.base_type == RobotType.RtPcwVehicle or self.base_type == RobotType.RtCustomPcwVehicle:
            with self.__data_lock:
                self.__has_new = False
                
                # Convert current position to transformation matrix
                x, y, yaw = self.__vehicle_position
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)
                current_matrix = np.array([
                    [cos_yaw, -sin_yaw, x],
                    [sin_yaw,  cos_yaw, y],
                    [0.0,      0.0,     1.0]
                ])
                
                # Calculate relative transformation: current * inverse(origin)
                origin_inv = np.linalg.inv(self.__vehicle_origin_position)
                relative_matrix = origin_inv @ current_matrix
                
                # Extract position and orientation from relative matrix
                relative_x = relative_matrix[0, 2]
                relative_y = relative_matrix[1, 2]
                relative_yaw = np.arctan2(relative_matrix[1, 0], relative_matrix[0, 0])
                
                return (relative_x, relative_y, relative_yaw)
        else:
            raise NotImplementedError("get_vehicle_position not implemented for base_type: ", self.base_type)

    def reset_vehicle_position(self):
        """ reset odometry position """
        with self.__data_lock:
            x, y, yaw = self.__vehicle_position
            # Convert (x, y, yaw) to 2D transformation matrix
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            self.__vehicle_origin_position = np.array([
                [cos_yaw, -sin_yaw, x],
                [sin_yaw,  cos_yaw, y],
                [0.0,      0.0,     1.0]
            ])
        
    def update_vehicle_data(self, vehicle_speed: Tuple[float, float, float], vehicle_position: Tuple[float, float, float]):
        with self.__data_lock:
            self.__has_new = True
            self.last_data_time = time.time()
            self.__vehicle_speed = vehicle_speed
            self.__vehicle_position = vehicle_position
        
    def update_wheel_data(self, base_status, wheel_torques: list, wheel_velocity: list, wheel_positions: list, wheel_errors: list):
        with self.__data_lock:
            self.__has_new = True
            self.last_data_time = time.time()
            self.__base_status = base_status
            if base_status.parking_stop_detail.IsInitialized() == False:
                self.__base_status.parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
            self.__wheel_torques = wheel_torques
            self.__wheel_velocity = wheel_velocity
            self.__wheel_positions = wheel_positions
            self.__wheel_errors = wheel_errors

    def update_imu_data(self, acc: Tuple[float, float, float], angular_velocity: Tuple[float, float, float], quaternion: Tuple[float, float, float, float]):
        with self.__data_lock:
            self.has_imu = True
            self.imu_acc = acc
            self.imu_angular_velocity = angular_velocity
            self.imu_quaternion = quaternion

    def __set_motor_targets(self, target: list):
        '''
        set motor target
        '''
        if len(target) != self.motor_cnt:
            raise ValueError("__set_motor_targets: target length error")
        with self.__command_lock:
            self.last_target_write_time = time.time()
            self.__motor_targets = deepcopy(target)

    def get_motor_targets(self) -> list:
        with self.__command_lock:
            if self.last_target_write_time is None or time.time() - self.last_target_write_time > TIME_OUT:
                return [0.0 for i in range(self.motor_cnt)]
            return deepcopy(self.__motor_targets)
    
    def set_motor_velocity(self, velocity: list):
        """
        If in "speed" mode, the velocity is the target velocity, if in "torque" mode, the output is the pid calculated torque.
        """
        # check if the velocity is valid
        if self.last_error is None:
            self.last_error = [0.0 for i in range(self.motor_cnt)]
        if self.last_data_time is None:
            self.last_data_time = time.time()
        if len(velocity) != self.motor_cnt:
            raise ValueError("set_motor_velocity: velocity length error")
            
        if self._simple_control_mode == True:
            raise NotImplementedError("set_motor_velocity not implemented for _simple_control_mode: True")
        elif self._simple_control_mode == None:
            self._simple_control_mode = False

        # if in "speed" mode, set the target velocity
        if self.__control_mode == "speed":
            self.__set_motor_targets(velocity)
        # if in "torque" mode, calculate the torque
        elif self.__control_mode == "torque":
            np_target_velocity = np.array(velocity)
            now_velocity = self.get_motor_velocity()
            np_velocity = np.array(now_velocity)
            error = np_target_velocity - np_velocity
            derivate = (error - self.last_error) / (self.last_data_time - time.time())
            self.last_error = error
            output = VEL_KP * error + VEL_KD * derivate
            self.__set_motor_targets(output.tolist())

    def set_motor_position(self, position: list):
        '''
        use torque to control motor position, not implemented yet.
        '''
        raise NotImplementedError("set_motor_position: not implemented")

        # calculate wheel velocity from motor position
        self.__set_motor_targets(position)

    def get_base_status(self) -> BaseStatus:
        '''
        Get current base status 
        '''
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__base_status)

    def get_motor_torque(self) -> list:
        '''
        Get motor real motor torque, unit: Nm
        '''
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__wheel_torques)
        
    def get_motor_position(self) -> list:
        '''
        Get motor real motor position, unit: radian
        '''
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__wheel_positions)
    
    def get_motor_velocity(self) -> list:
        '''
        Get motor real motor velocity, unit: rad/s
        '''
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__wheel_velocity)

    def get_motor_error(self) -> list:
        '''
        Get motor real motor error
        '''
        with self.__data_lock:
            self.__has_new = False
            return deepcopy(self.__wheel_errors)

    def get_motor_cnt(self) -> int:
        '''
        Get motor count
        '''
        return self.motor_cnt

    def get_imu_data(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float, float]]:
        '''
        Get imu data
        '''
        with self.__data_lock:
            return deepcopy(self.imu_acc), deepcopy(self.imu_angular_velocity), deepcopy(self.imu_quaternion)

    def has_new_data(self) -> bool:
        '''
        Check if there is any raw data in the buffer that has not been read.
        '''
        with self.__data_lock:
            return self.__has_new