import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2, json

class YOLOBackground(Node):
    def __init__(self):
        super().__init__('yolo_bg')
        self.bridge = CvBridge()
        self.frame = None
        self.detections = []
        self.active = False  # flag enable/disable

        # subscription จะ subscribe แต่จะใช้ frame ก็ต่อเมื่อ active=True
        self.create_subscription(CompressedImage, '/yolo/image/compressed', self.image_cb, 10)
        self.create_subscription(String, '/yolo/detections', self.detection_cb, 10)

    def start(self):
        self.active = True
        self.get_logger().info("YOLO detection started")

    def stop(self):
        self.active = False
        self.get_logger().info("YOLO detection stopped")

    def image_cb(self, msg: CompressedImage):
        if self.active:
            self.frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_cb(self, msg: String):
        if self.active:
            try:
                self.detections = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().error("Cannot decode YOLO detection")

    def get_frame(self):
        return self.frame if self.active else None
