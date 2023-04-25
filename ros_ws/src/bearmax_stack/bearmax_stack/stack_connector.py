import nest_asyncio
nest_asyncio.apply()

import rclpy
from rclpy.node import Node

import asyncio
import socketio
import ssl
import aiohttp
from importlib.resources import path

from bearmax_msgs.msg import StackRequest
from std_msgs.msg import String

from .asyncrcl import spin

DEFAULT_WS_URL = 'https://carewithbearmax.com'


class StackConnector(Node):
    def __init__(self, sio: socketio.AsyncClient):
        super().__init__('stack_connector')

        self.sio_ = sio

        self.publisher_ = self.create_publisher(
            String, '/stack_out', 1)

        self.subscriber_ = self.create_subscription(
            StackRequest, "/stack_in", self.send_to_stack, 1)

        self._ws_url = self.declare_parameter('ws_url', DEFAULT_WS_URL)

        self.logger.info(self._ws_url.value)

        self.register_handlers()

        self._event_loop = asyncio.get_event_loop()

        self._event_loop.create_task(
            self.connect()
        )

    def send_to_stack(self, request):
        self.logger.info(
            f"[Sending to Stack]: {{event: {request.event}, data: {request.data}}}")
        asyncio.run_coroutine_threadsafe(self.sio_.emit(request.event, data=request.data), self._event_loop)
        self.logger.info(
            f"[Sent to Stack]: {{event: {request.event}, data: {request.data}}}")

    async def connect(self):
        await self.sio_.connect(self.wsl_url)
        await self.sio_.wait()

    def register_handlers(self):
        @self.sio_.event
        async def connect():
            self.logger.info('Connected to WebSocket Server')

        @self.sio_.event
        async def connect_error(data):
            self.logger.error(
                f"Error trying to connect to WebSocket Server: {data}",
            )

        @self.sio_.event
        async def disconnect():
            self.logger.info('Disconnected from WebSocket Server')

        @self.sio_.event
        async def disconnecting():
            self.logger.info('Disconnecting from WebSocket Server')

        @self.sio_.event
        async def speak(msg):
            self.publisher_.publish(self.to_msg(f"ACK: [{msg}]"))

        @self.sio_.event
        async def emotionGame(action, userID):
            self.logger.info(f"Got game state command: {action} | {userID}")
            if not action in ("start", "stop"):
                self.logger.error(
                    f"emotionGame handler expected string action and got {action}")
            # "emotionStart" or "emotionStop"
            self.publisher_.publish(self.to_msg(
                "emotion" + action.capitalize() + "-" + userID))

        @self.sio_.event
        async def recalibrate():
            self.publisher_.publish(self.to_msg("recalibrate"))

        @self.sio_.event
        async def ping():
            self.logger.info("Pong!")

    def to_msg(self, str_data):
        str_obj = String()
        str_obj.data = str_data
        return str_obj

    @property
    def wsl_url(self) -> str:
        return self._ws_url.get_parameter_value().string_value

    @property
    def logger(self):
        return self.get_logger()


async def run(args=None):
    rclpy.init(args=args)
    #ssl_context = ssl.create_default_context()

    #with path("bearmax_stack.ssl", "server-crt.pem") as SCERT_PATH:
    #    ssl_context.load_verify_locations(SCERT_PATH)
    #with path("bearmax_stack.ssl", "client-crt.pem") as CCERT_PATH, path("bearmax_stack.ssl", "client-key.pem") as CKEY_PATH:
    #    ssl_context.load_cert_chain(certfile=CCERT_PATH, keyfile=CKEY_PATH)

    #connector = aiohttp.TCPConnector(ssl=ssl_context)
    connector = aiohttp.TCPConnector()
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


def main(args=None):
    try:
        asyncio.run(run(args=args))
    except RuntimeError:
        # FIXME: There must be a better way to handle cleanup of async stuff
        pass


if __name__ == "__main__":
    main()
