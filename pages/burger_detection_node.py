# burger_detection_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class BurgerDetection(Node):
    def __init__(self):
        super().__init__('burger_detection')
        self.sub = self.create_subscription(String, '/yolo/detections', self.detection_callback, 10)

    def detection_callback(self, msg):
        det_list = json.loads(msg.data)
        if det_list:
            self.get_logger().info(f"Detected {len(det_list)} objects: {[d['name'] for d in det_list]}")
        else:
            self.get_logger().info("No objects detected")

def main(args=None):
    rclpy.init(args=args)
    node = BurgerDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
