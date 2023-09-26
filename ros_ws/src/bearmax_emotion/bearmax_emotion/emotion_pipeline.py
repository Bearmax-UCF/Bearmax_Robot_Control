import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped, Pose
from visualization_msgs.msg import Marker
from bearmax_msgs.msg import StackCommand
from bearmax_msgs.msg import Emotion
from cv_bridge import CvBridge, CvBridgeError
from bearmax_emotion.emotion_lib.src.pipelineNode import run_pipeline, recalibrate
import math


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
        self.head_pose_out_pub = self.create_publisher(
            PoseStamped,
            "/detected_object",
            10)
        self.head_marker_out_pub = self.create_publisher(
            Marker,
            "/face_marker",
            10)
        self.emotion_out_pub = self.create_publisher(
            Emotion,
            "/emotion_out",
            1)
        self.stack_sub = self.create_subscription(
            String,
            "/stack_out",
            self.stackCallback,
            1
        )

        self.bridge = CvBridge()

        self.frame_count = 0
        self.tt = 0

        self.old_head_pos_x = 0.0
        self.old_head_pos_y = 0.0
        self.old_head_pos_z = 0.0

    @property
    def logger(self):
        return self.get_logger()

    def stackCallback(self, data):
        action = data.data
        self.logger.info("Recalibrating Pipeline!")
        if action == "recalibrate":
            recalibrate()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            self.logger.error(e)

        try:
            self.frame_count += 1

            _tt, out_image, head_pos, emotion = run_pipeline(
                cv_image, self.frame_count, self.tt)
            # self.logger.info(f"Detected: {emotion}")

            self.tt = _tt

            # Publish Output Image
            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

#            if (
#                    abs(self.old_head_pos_x - float(head_pos[0])) > 0.2 or
#                    abs(self.old_head_pos_y - float(head_pos[1])) > 0.2 or
#                    abs(self.old_head_pos_z - float(head_pos[2])) > 0.2
#                ):
#                self.old_head_pos_x = float(head_pos[0])
#                self.old_head_pos_y = float(head_pos[1])
#                self.old_head_pos_z = float(head_pos[2])

            if (float(head_pos[0]) != 0 and
                float(head_pos[1]) != 0 and
                    float(head_pos[2]) != 0):
                # Publish Head Position
                head_pos_to_pub = Point()
                head_pos_to_pub.x = float(head_pos[0])
                head_pos_to_pub.y = float(head_pos[1])
                # Using size as a rudimentary depth
                head_pos_to_pub.z = float(head_pos[2])
                self.head_out_pub.publish(head_pos_to_pub)

                pose = PoseStamped()
                pose.header.stamp.sec, pose.header.stamp.nanosec = self.get_clock(
                ).now().seconds_nanoseconds()
                pose.header.frame_id = "camera_optical_link"
                pose.pose.position.x = float(head_pos[0]) - 0.5
                pose.pose.position.y = (1 - float(head_pos[1])) - 0.5
                pose.pose.position.z = ((1 - float(head_pos[2])) * 0.312)
                self.head_pose_out_pub.publish(pose)

                marker = Marker()
                marker.header = pose.header
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose = pose.pose
                marker.scale.x = 0.066
                marker.scale.y = 0.066
                marker.scale.z = 0.066
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                self.head_marker_out_pub.publish(marker)

            # Publish Detected Emotion
            emotion_to_pub = Emotion()
            emotion_to_pub.emotion = emotion
            self.emotion_out_pub.publish(emotion_to_pub)

        except CvBridgeError as e:
            self.logger.error(e)


def main(args=None):
    rclpy.init(args=args)

    emotion_pipeline = EmotionPipeline()
    emotion_pipeline.logger.info("Pipeline running")

    while rclpy.ok():
        rclpy.spin_once(emotion_pipeline)

    emotion_pipeline.destroy_node()
    rclpy.shutdown()
