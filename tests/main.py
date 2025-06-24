import sys
sys.path.insert(1, '<your project path>/hex_vehicle_python_lib')
sys.path.insert(1, '<your project path>/hex_vehicle_python_lib/hex_vehicle/generated')

from hex_vehicle import PublicAPI as VehicleAPI
import time

def main():
    # Init VehicleAPI
    api = VehicleAPI(ws_url = "ws://172.18.2.66:8439", control_hz = 200, control_mode = "speed")

    # Get velocity interface
    velocity_interface = api.vehicle

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                data, count = api._get_raw_data()
                if data != None:
                    pass
                    # print("count = ", count)

                if velocity_interface.has_new_data():
                    velocity = velocity_interface.get_motor_velocity()
                    print("velocity:", velocity)
                    tor = velocity_interface.get_motor_torque()
                    print("tor:", tor)
                    position = velocity_interface.get_motor_position()
                    print("position:", position)
                    error = velocity_interface.get_motor_error()
                    print("error:", error)

                # velocity_interface.set_target_vehicle_speed(0.0, 0.0, 1.0)
                velocity_interface.set_motor_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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