import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from bearmax_msgs.msg import StackCommand
from bearmax_msgs.msg import Emotion
from cv_bridge import CvBridge, CvBridgeError
from bearmax_emotion.emotion_lib.src.pipelineNode import run_pipeline


class EmotionPipeline(Node):
    def __init__(self):
        super().__init__('emotion_pipeline')

        self.image_sub = self.create_subscription(
            Image,
            "/image_in",
            self.callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(
            Image,
            "/image_out",
            1)
        self.head_out_pub = self.create_publisher(
            Point,
            "/head_out",
            1)
        self.emotion_out_pub = self.create_publisher(
            Emotion,
            "/emotion_out",
            1)

        self.bridge = CvBridge()

        self.frame_count = 0
        self.tt = 0

    @property
    def logger(self):
        return self.get_logger()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


        except CvBridgeError as e:
            self.logger.error(e)

        try:
            self.frame_count += 1

            _tt, out_image, head_pos, emotion = run_pipeline(cv_image, self.frame_count, self.tt)

            self.tt = _tt

            # Publish Output Image
            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            # Publish Head Position
            head_pos_to_pub = Point()
            head_pos_to_pub.x = float(head_pos[0])
            head_pos_to_pub.y = float(head_pos[1])
            # Using size as a rudimentary depth
            head_pos_to_pub.z = float(head_pos[2])
            self.head_out_pub.publish(head_pos_to_pub)

            # Publish Detected Emotion
            emotion_to_pub = Emotion()
            emotion_to_pub.emotion = emotion
            self.emotion_out_pub.publish(emotion_to_pub)

        except CvBridgeError as e:
            self.logger.error(e)

def main(args=None):
    rclpy.init(args=args)

    emotion_pipeline = EmotionPipeline()
    while rclpy.ok():
        rclpy.spin_once(emotion_pipeline)

    emotion_pipeline.destroy_node()
    rclpy.shutdown()
