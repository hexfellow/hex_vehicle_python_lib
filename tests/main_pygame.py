import sys
sys.path.insert(1, '/Downloads/python/hex_vehicle_python_lib')
sys.path.insert(1, '/Downloads/python/hex_vehicle_python_lib/hex_vehicle/generated')

from hex_vehicle import PublicAPI as VehicleAPI
import time
import pygame
import math

def init_pygame_joystick():
    """初始化pygame和手柄"""
    pygame.init()
    pygame.joystick.init()
    
    # 检查是否有可用的手柄
    if pygame.joystick.get_count() == 0:
        print("未检测到手柄，将使用默认值")
        return None
    
    # 初始化第一个手柄
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"检测到手柄: {joystick.get_name()}")
    print(f"手柄轴数: {joystick.get_numaxes()}")
    print(f"手柄按钮数: {joystick.get_numbuttons()}")
    
    return joystick

def get_joystick_values(joystick):
    """获取手柄输入值"""
    if joystick is None:
        return 0.0, 0.0, 0.0
    
    # 处理pygame事件
    for event in pygame.event.get():
        pass
    
    # 获取手柄轴值 (通常在-1.0到1.0之间)
    # 左摇杆: 轴0(左右), 轴1(上下)
    # 右摇杆: 轴2(左右), 轴3(上下)
    left_x = joystick.get_axis(0)  # 左右移动
    left_y = joystick.get_axis(1)  # 前后移动
    right_x = joystick.get_axis(2)  # 旋转
    
    # 应用死区处理，避免摇杆漂移
    deadzone = 0.1
    left_x = 0.0 if abs(left_x) < deadzone else left_x
    left_y = 0.0 if abs(left_y) < deadzone else left_y
    right_x = 0.0 if abs(right_x) < deadzone else right_x
    
    # 映射到车辆速度参数
    # left_x: 左右速度 (x方向)
    # left_y: 前后速度 (y方向) 
    # right_x: 旋转速度 (z方向)
    
    return left_x, left_y, right_x

def main():
    # 初始化pygame手柄
    joystick = init_pygame_joystick()
    
    # 创建 PublicAPI 实例并初始化
    # api = VehicleAPI(ws_url = "ws://172.18.2.66:8439", control_hz = 200)
    # api = VehicleAPI(ws_url = "ws://10.1.1.1:8439", control_hz = 200)
    api = VehicleAPI(ws_url = "ws://172.18.2.66:8439", control_hz = 200, control_mode = "speed")

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
                    pass
                    # print("count = ", count)

                if velocity_interface.has_new_data():
                    velocity = velocity_interface.get_motor_velocity()
                    # print("velocity:", velocity)
                    # tor = velocity_interface.get_motor_torque()
                    # print("tor:", tor)
                    position = velocity_interface.get_motor_position()
                    # print("position:", position)

                # 获取手柄输入值
                x_speed, y_speed, z_speed = get_joystick_values(joystick)
                
                # 应用速度限制和缩放
                max_speed = 2.0  # 最大速度限制
                x_speed *= max_speed
                y_speed *= max_speed
                z_speed *= max_speed
                
                # 打印当前速度值（可选）
                if abs(x_speed) > 0.01 or abs(y_speed) > 0.01 or abs(z_speed) > 0.01:
                    print(f"速度: X={x_speed:.2f}, Y={y_speed:.2f}, Z={z_speed:.2f}")
                
                # 使用手柄输入设置目标速度
                # velocity_interface.set_target_vehicle_speed(x_speed, y_speed, z_speed)
                # velocity_interface.set_motor_velocity([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

            time.sleep(0.0005)
            
    except KeyboardInterrupt:
        print("Received Ctrl-C.")
        api.close()
    finally:
        # 清理pygame资源
        if joystick:
            joystick.quit()
        pygame.quit()

    print("Resources have been cleaned up.")
    exit(0)

if __name__ == "__main__":
    main()