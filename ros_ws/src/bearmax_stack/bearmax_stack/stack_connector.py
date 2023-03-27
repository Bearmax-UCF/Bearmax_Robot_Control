import rclpy
from rclpy.node import Node

from bearmax_msgs.msg import StackCommand

import asyncio
import socketio
import ssl
import aiohttp
from importlib.resources import path
from std_msgs.msg import String

from .asyncrcl import spin

DEFAULT_WS_URL = 'https://localhost:8080'


class StackConnector(Node):
    def __init__(self, sio: socketio.AsyncClient):
        super().__init__('stack_connector')

        self.sio_ = sio

        self.publisher_ = self.create_publisher(
            String, 'stackcommand', 10)

        self.subscriber_ = self.create_subscription(
            StackCommand, "to_stack", self.send_to_stack, 10)

        self._ws_url = self.declare_parameter('ws_url', DEFAULT_WS_URL)

        self.log_info(self._ws_url.value)

        self.register_handlers()

        self._event_loop = asyncio.get_running_loop()

        self._event_loop.create_task(
            self.connect()
        )

    async def send_to_stack(self, request):
        self.log_info(
            f"[Sent to Stack]: {{event: {request.event}, data: {request.data}}}")
        await self.sio_.emit(request.event, request.data)

    async def connect(self):
        await self.sio_.connect(self.wsl_url)
        await self.sio_.wait()

    @property
    def wsl_url(self) -> str:
        return self._ws_url.get_parameter_value().string_value

    def register_handlers(self):
        @self.sio_.event
        async def connect():
            self.log_info('Connected to WebSocket Server')

        @self.sio_.event
        async def connect_error(data):
            self.log_err(
                f"Error trying to connect to WebSocket Server: {data}",
            )

        @self.sio_.event
        async def disconnect():
            self.log_info('Disconnected from WebSocket Server')

        @self.sio_.event
        async def disconnecting():
            self.log_info('Disconnecting from WebSocket Server')

        @self.sio_.event
        async def emotionGame(action):
            if not action in ("start", "stop"):
                self.log_err(
                    f"emotionGame handler expected string action and got {action}")
            # "emotionStart" or "emotionStop"
            self.publisher_.publish("emotion" + action.capitalize())

        @self.sio_.event
        async def recalibrate():
            self.publisher_.publish("recalibrate")

        @self.sio_.event
        async def ping():
            self.log_info("Pong!")

    def log_info(self, msg):
        self.get_logger().info(msg)

    def log_err(self, msg):
        self.get_logger().error(msg)


async def run(args=None):
    rclpy.init(args=args)
    ssl_context = ssl.create_default_context()

    with path("bearmax_stack.ssl", "server-crt.pem") as SCERT_PATH:
        ssl_context.load_verify_locations(SCERT_PATH)
    with path("bearmax_stack.ssl", "client-crt.pem") as CCERT_PATH, path("bearmax_stack.ssl", "client-key.pem") as CKEY_PATH:
        ssl_context.load_cert_chain(certfile=CCERT_PATH, keyfile=CKEY_PATH)

    connector = aiohttp.TCPConnector(ssl=ssl_context)
    async with aiohttp.ClientSession(connector=connector) as http_session:
        sio = socketio.AsyncClient(http_session=http_session, logger=True)

        stack_connector = StackConnector(sio)

        spin_task = asyncio.get_event_loop().create_task(spin(stack_connector))

        await asyncio.gather(spin_task)

        # cancel tasks
        if spin_task.cancel():
            await spin_task

        if (sio.connected):
            await sio.disconnect()

        stack_connector.destroy_node()
    rclpy.shutdown()


def createStackCommand(event, data) -> StackCommand:
    msg = StackCommand()
    msg.event = event
    msg.data = data
    return msg


def main(args=None):
    try:
        asyncio.run(run(args=args))
    except RuntimeError:
        # FIXME: There must be a better way to handle cleanup of async stuff
        pass


if __name__ == "__main__":
    main()
