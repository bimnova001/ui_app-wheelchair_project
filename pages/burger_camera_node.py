# burger_camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class BurgerCamera(Node):
    def __init__(self):
        super().__init__('burger_camera')
        self.pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # USB camera
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Cannot read camera frame")
            return
        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
        self.pub.publish(msg)
        self.get_logger().info("Published camera frame")

def main(args=None):
    rclpy.init(args=args)
    node = BurgerCamera()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
