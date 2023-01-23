import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from bearmax_msgs.msg import WSPSensorDataRaw, StressValue

import asyncio
from bleak import BleakClient

#    async with BleakClient("") as client:
#        val = await client.read_gatt_char("uuid")

class WSPConnector(Node):
    def __init__(self) -> None:
        super().__init__('wsp_connector')

        self.publisher_ = self.create_publisher(WSPSensorDataRaw, 'wspsensordata_raw', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        ## Sensor data variables
        self.temperature = 0
        self.pulse = 0
        self.respiratoryrate = 0
        self.gsr = 0
        ##

    async def timer_callback(self):
        async with BleakClient("") as client:
            val = await client.read_gatt_char("uuid")

        msg = WSPSensorDataRaw()
        msg.header = Header()
        msg.header.frame_id = "wsp"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temperature = self.temperature
        msg.pulse = self.pulse
        msg.respiratoryrate = self.respiratoryrate
        msg.gsr = self.gsr

        self.publisher_.publish(msg)

        self.get_logger().info(f"Publishing WSP Data: {msg.header.stamp} {msg.header.frame_id} {msg.temperature} {msg.pulse} {msg.respiratoryrate} {msg.gsr}")

        self.temperature += 1

async def run_sensor():
    pass

def main(args=None):
    rclpy.init(args=args)

    wsp_connector = WSPConnector()

    rclpy.spin(wsp_connector)

    wsp_connector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
