# Hex Vehicle Python Library

A Python library for controlling hex vehicles through a WebSocket-based API.

## Overview

This library provides a simple interface for communicating with and controlling hex vehicles. It uses Protocol Buffers for message serialization and WebSocket for real-time communication.

## Clone
```
git clone https://github.com/hexfellow/hex_lift_python_lib.git
git submodule update --init
```

## Prerequisites

- **Python 3.8.10 or higher**
- Anaconda Distribution (recommended for beginners) - includes Python, NumPy, and commonly used scientific computing packages

## Quickstart

### Option 1: Direct Usage (No Installation)

If you prefer to run the library without installing it in your Python environment:

1. **Compile Protocol Buffer messages:**
   ```bash
   mkdir ./hex_vehicle/generated
   protoc --proto_path=proto-public-api --python_out=hex_vehicle/generated proto-public-api/*.proto
   ```

2. **Add the library path to your script:**
   ```python
   import sys
   sys.path.insert(1, '<your project path>/hex_vehicle_python_lib')
   sys.path.insert(1, '<your project path>/hex_vehicle_python_lib/hex_vehicle/generated')
   ```

3. **Install dependencies**
   ```bash
   pip install websockets
   pip install pygame
   ```

4. **Run your test script:**
   ```bash
   python3 tests/<your_script>.py
   ```

### Option 2: Package Installation

To install the library in your Python environment:

1. **Build the package:**
   ```bash
   python3 -m pip install .
   ```

2. **Run your test script:**
   ```bash
   python3 tests/<your_script>.py
   ```

## Usage

All vehicle control interfaces are provided through the `VehicleAPI` class:

```python
from hex_vehicle import PublicAPI as VehicleAPI

# Initialize the API
api = VehicleAPI(ws_url="ws://your_vehicle_ip:8439", control_hz=200, control_mode="speed")

# Get vehicle interface
vehicle = api.vehicle

# Control the vehicle
vehicle.set_target_vehicle_speed(x_speed, y_speed, rotation_speed)
```

## Architecture

The library consists of three main modules:

### 1. public_api
- **Network Manager**: Handles Protocol Buffer message construction and WebSocket communication
- Manages data transmission to and from the vehicle

### 2. vehicle  
- **Vehicle Data Manager**: Continuously updates vehicle data in a loop
- Provides interfaces for:
  - Obtaining chassis status
  - Controlling vehicle movement
  - Reading motor data (velocity, torque, position)

### 3. utils
- **General Tools**: Parameter management and common utility functions
- Provides helper functions for various operations

## Examples

Please refer to the `tests/` directory for example usage:
- `main.py` - Basic vehicle control example
- `main_pygame.py` - Joystick control example using pygame.
- `vehicle_calc.py` - A simple kinematics solution.

Additional application projects can be found here:
[tidybot2](https://github.com/hexfellow/tidybot2/blob/main/base_controller_pygame.py)

## Requirements

- numpy>=1.17.4
- protobuf
- websockets
- pygame (for joystick control examples)