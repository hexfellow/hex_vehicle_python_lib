from hex_vehicle import public_api_types_pb2, PublicAPI

def main():
    state = public_api_types_pb2.BaseState.Parked
    print("state",state)
    api = PublicAPI(1, "ws://localhost:8080")

    msg = api.construct_control_message("speed", [100, 150])
    print(msg)

    msg = api.construct_set_emergency_stop_message("fuck", public_api_types_pb2.EmergencyStopCategory.MotorHasError, True)
    print(msg)

    msg = api.construct_clear_emergency_stop_message()
    print(msg)

if __name__ == "__main__":
    main()