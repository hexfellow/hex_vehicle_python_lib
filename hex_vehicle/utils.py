#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune jecjune@qq.com
# Date  : 2025-3-20
################################################################
import re
import time
import asyncio

class InvalidWSURLException(Exception):
    """自定义异常，用于表示 WebSocket URL 无效"""
    pass

def is_valid_ws_url(url: str) -> str:
    # WebSocket URL 正则表达式，支持 ws:// 或 wss:// 协议
    ws_url_pattern = re.compile(r'^(ws|wss)://([a-zA-Z0-9.-]+)(?::(\d+))?$')

    match = ws_url_pattern.match(url)
    if not match:
        raise InvalidWSURLException(f"Invalid WebSocket URL: {url}")

    # 提取 URL 中的各部分
    protocol, host, port_str = match.groups()

    # 如果没有提供端口号，默认设置为 8080
    if not port_str:
        port_str = '8080'

    # 检查端口是否合法（如果有提供端口）
    try:
        port = int(port_str)
        # 端口号必须在 0 到 65535 之间
        if not (0 <= port <= 65535):
            raise InvalidWSURLException(f"Invalid port number in URL: {url}")
    except ValueError:
        raise InvalidWSURLException(f"Invalid port number in URL: {url}")

    # 返回修正后的 WebSocket URL
    return f"{protocol}://{host}:{port_str}"


async def delay(start_time, ms):
    end_time = start_time + ms / 1000
    now = time.perf_counter()
    await asyncio.sleep(end_time - now)