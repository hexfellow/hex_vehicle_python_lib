import sys
sys.path.insert(1, '/home/jecjune/docker_link/python/hex_vehicle_python_lib')
sys.path.insert(1, '/home/jecjune/docker_link/python/hex_vehicle_python_lib/hex_vehicle/generated')

from hex_vehicle import PublicAPI as VehicleAPI
import time

def main():
    # 创建 PublicAPI 实例并初始化
    api = VehicleAPI(ws_url = "ws://0.0.0.0:8439", control_hz = 100)

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                data, count = api._get_raw_data()
                print("data:", data)
                print("count:", count)

                if api.has_new_data():
                    velocity = api.get_motor_velocity()
                    print("velocity:", velocity)
                    tor = api.get_motor_torque()
                    print("tor:", tor)

                api.set_speed(0.1, 0.1, 0.1)
                
            time.sleep(0.0001)
            
    except KeyboardInterrupt:
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)

if __name__ == "__main__":
    main()