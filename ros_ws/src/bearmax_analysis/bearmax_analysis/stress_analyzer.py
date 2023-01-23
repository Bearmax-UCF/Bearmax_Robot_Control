import rclpy
from rclpy.node import Node

from bearmax_msgs import WSPSensorDataRaw

from pyEDA.main import *

class StressAnalyzer(Node):
    def __init__(self):
        super().__init__('stress_analyzer')

        # Baseline temperature for user
        self.declare_parameter('base_temp', 0.0)
        # Baseline pulse for AC comp. of PPG sensor
        self.declare_parameter('base_pulse', 0.0)
        # Baseline respiratory comp. of PPG sensor
        self.declare_parameter('base_resp', 0.0)
        # Baseline skin conductance level of the user
        self.declare_parameter('base_scl', 0.0)
        # Baseline amplitude of phasic comp. to skin conductance response
        self.declare_parameter('base_scr', 0.0)
        # Baseline latency in response to a question (s)
        self.declare_parameter('base_lat', 0.0)
        # Baseline rise time of phasic comp. to SCR (s)
        self.declare_parameter('base_rise', 0.0)
        # Baseline half recovery time from SCR (s)
        self.declare_parameter('base_recov', 0.0)
        # Sampling (Hz) for the GSR
        self.declare_parameter('base_samp', 0.0)
        # Last time the measurements where calibrated
        self.declare_parameter('last_cal', 0)

        self.subscription = self.create_subscription(
            WSPSensorDataRaw,
            'wspsensordata_raw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'[Stress Analyzer]: Listen - {msg}')

def main(args=None):
    rclpy.init(args=args)

    stress_analyzer = StressAnalyzer()

    rclpy.spin(stress_analyzer)

    stress_analyzer.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
