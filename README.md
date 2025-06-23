# Hex Vehicle Package
This is a simple package for control vehicle.

If you don't have your python environment yat and want the simplest way to get started, we recommend you use the Anaconda Distribution - it includes Python, NumPy, and many other commonly used packages for scientific computing and data science.

# Do not install and run
You can choose not to install and execute this software package directly in the local environment.
1. Compile the proto message
How to compile the proto message in local files:
```shell
mkdir ./hex_vehicle/generated && protoc --proto_path=proto-public-api --python_out=hex_vehicle/generated proto-public-api/*.proto
```
2. Add a path to your running script
```python
import sys
sys.path.insert(1, '<your project path>/hex_vehicle_python_lib')
sys.path.insert(1, '<your project path>/hex_vehicle/generated')
```

# Install and run
You can also install this package in your python environment.
## Build the package
```shell
python3 -m build
```
## Install the package
```shell
pip3 install dist/hex_vehicle-0.0.1-py3-none-any.whl
```


# Details
All interfaces will be provided from the Vehicle API class.

...



This lib contain Three modules:
1. public_api
- network manager: Reference proto package, construct data to send and receive data
2. vehicle
- vehicle data manager: Loop to update vehicle data. Implement interfaces for obtaining chassis status and controlling the chassis
3. utils
- General tools: Manage parameters and provide some common functionality.