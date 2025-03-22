from hex_vehicle import PublicAPI
import time

def main():
    # 创建 PublicAPI 实例并初始化
    api = PublicAPI("ws://localhost:8765", 100)
    api.loop.start()

    try:
        while True:
            if api.is_api_exit():
                print("Public API has exited.")
                break
            time.sleep(1)
            
    except KeyboardInterrupt:
        api.close()
    finally:
        pass

    api.loop.join()  # 等待 WebSocket 循环结束
    print("Resources have been cleaned up.")
    exit(0)

if __name__ == "__main__":
    # 运行主程序
    main()