import sys
sys.path.insert(1, '/home/jecjune/docker_link/python/hex_vehicle_python_lib')
sys.path.insert(1, '/home/jecjune/docker_link/python/hex_vehicle_python_lib/hex_vehicle/generated')

from hex_vehicle import PublicAPI as VehicleAPI
import time

def main():
    # 创建 PublicAPI 实例并初始化
    # api = VehicleAPI(ws_url = "ws://172.18.2.66:8439", control_hz = 200)
    # api = VehicleAPI(ws_url = "ws://10.1.1.1:8439", control_hz = 200)
    api = VehicleAPI(ws_url = "ws://127.0.0.1:8439", control_hz = 200)

    # 获取车辆接口
    velocity_interface = api.vehicle

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                data, count = api._get_raw_data()
                if data != None:
                    print("count = ", count)

                if velocity_interface.has_new_data():
                    velocity = velocity_interface.get_motor_velocity()
                    print("velocity:", velocity)
                    tor = velocity_interface.get_motor_torque()
                    print("tor:", tor)

                # velocity_interface.set_motor_torque([0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                
            time.sleep(0.0005)
            
    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)

if __name__ == "__main__":
    main()