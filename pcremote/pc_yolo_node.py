# pc_yolo_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from yolov5 import YOLOv5  # ใช้ yolov5 package (pip install yolov5)

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.pub = self.create_publisher(String, '/yolo/detections', 10)
        self.bridge = CvBridge()
        self.yolo = YOLOv5("model/yolov8.pt")  # หรือ "cpu" ถ้าไม่มี GPU

    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.yolo.predict(frame)
        # ส่งเป็น string JSON format ง่าย ๆ
        det_str = results.pandas().xyxy[0].to_json(orient="records")
        self.pub.publish(String(data=det_str))
        self.get_logger().info("Published YOLO detection")

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
