import rclpy
from rclpy.node import Node

from bearmax_msgs import StackCommand

import asyncio
import socketio

from asyncrcl import spin

DEFAULT_WS_URL = 'http://localhost:8080'

class StackConnector(Node):
    sio_ = socketio.AsyncClient()

    def __init__(self, sio: socketio.AsyncClient):
        super().__init__('stack_connector')

        self.sio_ = sio
        
        self.publisher_ = self.create_publisher(StackCommand, 'stackcommand', 10)

        self.declare_parameter('ws_url', DEFAULT_WS_URL)

        self.register_handlers()

        asyncio.get_event_loop().create_task(
            self.sio_.connect(self.wsl_url)
        )

    @property
    def wsl_url(self) -> str:
        return self.get_parameter('wsl_url').get_parameter_value().string_value

    async def publish_cmd(self, message: StackCommand):
        self.publisher_.publish(message)

    def register_handlers(self):
        @self.sio_.event
        async def connect():
            self.get_logger().info('Connected to WebSocket Server')

        @self.sio_.event
        async def connect_error(data):
            self.get_logger().error(
                f"Error trying to connect to WebSocket Server: {data}",
            )

        @self.sio_.event
        async def disconnect():
            self.get_logger().info('Disconnected from WebSocket Server')

        @self.sio_.event
        async def command(data):
            self.get_logger().info(f"Recieved Command: {data}")

            await self.publish_cmd(data)

async def main():

    sio = socketio.AsyncClient()

    stack_connector = StackConnector(sio)

    # create tasks for spinning and sleeping
    spin_task = asyncio.get_event_loop().create_task(spin(stack_connector))
    sleep_task = asyncio.get_event_loop().create_task(asyncio.sleep(5.0))

    # concurrently execute both tasks
    await asyncio.wait([spin_task, sleep_task], return_when=asyncio.FIRST_COMPLETED)

    # cancel tasks
    if spin_task.cancel():
        await spin_task
    if sleep_task.cancel():
        await sleep_task

    if (sio.connected):
        await sio.disconnect()

    stack_connector.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(main())
    asyncio.get_event_loop().close()
    rclpy.shutdown()
