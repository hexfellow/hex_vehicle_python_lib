import sys
import random
import time

sys.path.insert(1, '/home/jecjune/docker_link/python/hex_vehicle_python_lib')
sys.path.insert(1, '/home/jecjune/docker_link/python/hex_vehicle_python_lib/hex_vehicle/generated')

from hex_vehicle import PublicAPI as VehicleAPI

def main():
    # Init VehicleAPI
    api = VehicleAPI(ws_url="ws://172.18.2.66:8439", control_hz=200, control_mode="speed")

    # Get velocity interface
    velocity_interface = api.vehicle

    # Initialize previous velocity
    prev_velocity = [0.0] * 8  # 8 motor velocities initialized to 0

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            else:
                data, count = api._get_raw_data()
                if data is not None:
                    pass
                    # print("count = ", count)

                if velocity_interface.has_new_data():
                    velocity = velocity_interface.get_motor_velocity()
                    print("velocity:", velocity)
                    tor = velocity_interface.get_motor_torque()
                    print("tor:", tor)
                    position = velocity_interface.get_motor_position()
                    print("position:", position)

                # Generate new velocity values with random changes within range [-0.5, 0.5]
                new_velocity = [
                    max(min(prev_velocity[i] + random.uniform(-0.5, 0.5), 1.0), -1.0)
                    for i in range(8)
                ]
                velocity_interface.set_motor_velocity(new_velocity)
                prev_velocity = new_velocity  # Update the previous velocity

            time.sleep(1.0 / 100.0)

    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        pass

    print("Resources have been cleaned up.")
    exit(0)

if __name__ == "__main__":
    main()
