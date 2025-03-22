import asyncio
import websockets
from websockets.exceptions import ConnectionClosed

# 配置参数
HOST = "localhost"
PORT = 8765
PING_INTERVAL = 10  # 服务器主动发送Ping的间隔（秒）
PING_TIMEOUT = 30   # 等待Pong响应的超时时间（秒）

async def handle_connection(websocket):
    client_ip = websocket.remote_address[0]

    try:
        while True:
            try:
                # 接收客户端消息（带超时机制）
                message = await asyncio.wait_for(websocket.recv(), timeout=PING_TIMEOUT)
                print(f"Received from {client_ip}: {message}")
                
                # 业务处理逻辑（示例：原样返回消息）
                await websocket.send(f"ACK: {message}")
                
            except asyncio.TimeoutError:
                # 定期触发Ping保持连接
                print(f"Connection with {client_ip} is alive")
                continue

    except ConnectionClosed as e:
        print(f"Connection closed with {client_ip} (Code: {e.code}, Reason: {e.reason})")

async def start_server():
    # 配置WebSocket服务参数
    server = await websockets.serve(
        handle_connection,
        HOST,
        PORT,
        ping_interval=PING_INTERVAL,  # 自动发送Ping帧
        ping_timeout=PING_TIMEOUT,     # 自动检测死连接
        close_timeout=5                # 关闭握手超时
    )

    print(f"Server started on ws://{HOST}:{PORT}")
    print(f"Ping interval: {PING_INTERVAL}s, Ping timeout: {PING_TIMEOUT}s")

    # 保持服务器永久运行
    await server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(start_server())
    except KeyboardInterrupt:
        print("\nServer shutdown by operator")