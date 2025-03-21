from hex_vehicle import public_api_types_pb2, PublicAPI
import asyncio
import websockets

def main():
    # 创建 PublicAPI 实例并初始化
    api = PublicAPI(1, "ws://localhost:8765")
    api.loop.start()

    try:
        while True:
            pass
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