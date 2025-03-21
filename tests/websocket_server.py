import asyncio
import websockets

# 处理 WebSocket 连接
async def handle_connection(websocket):
    async for message in websocket:
        print(f"Received message: {message}")

# 启动 WebSocket 服务器
async def start_server():
    async with websockets.serve(handle_connection, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # 保持服务器运行

# 运行服务器
asyncio.run(start_server())