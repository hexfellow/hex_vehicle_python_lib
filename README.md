# Hex Vehicle Package
This is a simple package for control vehicle.

# Installation
If you don't have your python environment yat and want the simplest way to get started, we recommend you use the Anaconda Distribution - it includes Python, NumPy, and many other commonly used packages for scientific computing and data science.

# Details
...


How to compile the proto message in local files:
```shell
mkdir ./hex_vehicle/generated && protoc --proto_path=proto-public-api --python_out=hex_vehicle/generated proto-public-api/*.proto
```


This lib contain Three modules:
1. public_api
- network manager: Reference proto package, construct data to send and receive data
2. vehicle
- vehicle data manager: Loop to update vehicle data. Implement interfaces for obtaining chassis status and controlling the chassis
3. utils
- General tools: Manage parameters and provide some common functionality.