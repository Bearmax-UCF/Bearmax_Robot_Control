import nest_asyncio
nest_asyncio.apply()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bearmax_msgs.msg import WSPSensorDataRaw, StressValue

import asyncio
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from .asyncrcl import spin

DEFAULT_ADDRESS = "D5:79:C3:1D:B0:21"
DEFAULT_NAME = "Feather nRF52840 Sense"
DEFAULT_GSR_UUID = "00000403-1212-efde-1523-785feabcd123"


class WSPConnector(Node):
    def __init__(self) -> None:
        super().__init__('wsp_connector')

        self.publisher_ = self.create_publisher(
            WSPSensorDataRaw,
            '/wspsensordata_raw',
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._address = self.declare_parameter('address', DEFAULT_ADDRESS)
        self._name = self.declare_parameter('name', DEFAULT_NAME)
        self._gsr_uuid = self.declare_parameter('gsr_uuid', DEFAULT_GSR_UUID)
        self._dev = None

        self._event_loop = asyncio.get_event_loop()

        self._event_loop.create_task(
            self.collect()
        )

    async def collect(self):
        self.logger.info("Starting Scan...")

        async def handler(_, data: bytearray):
            data = ''.join(map(chr, data))

            msg = WSPSensorDataRaw()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.gsr = int(data)

            self.publisher_.publish(msg)

            self.logger.info

        self._dev = None
        while self._dev is None:
            try:
                self._dev = await BleakScanner.find_device_by_address(
                    self.address)
                if self._dev is None:
                    self._dev = await BleakScanner.find_device_by_name(
                        self.name)
                    if self._dev is None:
                        self.logger.error(
                            "Could not find device with address or name:",
                            self.address, self.name)
                        self.logger.info("Waiting 3 seconds...")
                        await asyncio.sleep(3)
            except asyncio.CancelledError:
                self.logger.info("Cancelling Scanner")
                return

        self.logger.info("Connecting to Device ...")
        async with BleakClient(self._dev) as client:
            self.logger.info("Connected")

            svcs = client.services

            char = svcs.get_characteristic(self.gsr_uuid)
            if char is not None:
                await client.start_notify(char, handler)
            while True:
                try:
                    await asyncio.sleep(1)
                except asyncio.CancelledError:
                    self.logger.info("Cancelling and Disconnecting...")
                    break
        self.logger.info("Disconnected!!")

    @property
    def logger(self):
        return self.get_logger()

    @property
    def address(self) -> str:
        return self._address.get_parameter_value().string_value

    @property
    def name(self) -> str:
        return self._name.get_parameter_value().string_value

    @property
    def gsr_uuid(self) -> str:
        return self._gsr_uuid.get_parameter_value().string_value


async def run(args=None):
    rclpy.init(args=args)

    wsp_connector = WSPConnector()

    spin_task = asyncio.get_event_loop().create_task(spin(wsp_connector))

    await asyncio.gather(spin_task)

    if spin_task.cancel():
        await spin_task

    wsp_connector.destroy_node()
    rclpy.shutdown()


def main(args=None):
    try:
        asyncio.run(run(args=args))
    except RuntimeError:
        # FIXME: There must be a better way to handle cleanup of async stuff
        pass


if __name__ == '__main__':
    main()
