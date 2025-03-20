#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################
from .generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
import string


class PublicAPI:
    def __init__(self, chassis_type, ws_url: string):
        self.chassis_type = chassis_type
        self.ws_url = ws_url
        

    def construct_down_data():
        
        pass


    def send_down_message(data):
        
        pass
    


    

