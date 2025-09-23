import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import threading

class StatusData:
    battery_percent = 0.0

class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)

    def battery_callback(self, msg: BatteryState):
        StatusData.battery_percent = msg.percentage * 100.0

def start_status_node():
    rclpy.init()
    node = StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

threading.Thread(target=start_status_node, daemon=True).start()