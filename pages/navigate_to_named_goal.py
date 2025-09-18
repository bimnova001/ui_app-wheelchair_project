
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from yolo_bg import YoloBackground
import threading
import time

class YoloNav2Bridge(Node):
    def __init__(self):
        super().__init__('yolo_nav2_bridge')
        self.get_logger().info("[Bridge] Starting YoloNav2Bridge node")

        # YOLO background
        self.yolo = YoloBackground(log_func=self.get_logger().info)
        self.yolo.start(cam_id=0)

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscription /goal_pose
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)

        # Subscription /yolo_stop
        self.stop_flag = False
        self.create_subscription(Bool, '/yolo_stop', self.on_yolo_stop, 10)

        # Thread for YOLO monitoring
        threading.Thread(target=self.monitor_yolo, daemon=True).start()

        self.get_logger().info("[Bridge] Node initialized")

    def on_goal_pose(self, msg: PoseStamped):
        if self.stop_flag:
            self.get_logger().warn("[Bridge] Cannot send goal, YOLO stop active")
            return

        self.get_logger().info(f"[Bridge] Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[Bridge] NavigateToPose action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("[Bridge] Goal rejected")
            return
        self.get_logger().info("[Bridge] Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_cb)
        self._current_goal_handle = goal_handle

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[Bridge] Remaining: {getattr(feedback, 'distance_remaining', '?'):.2f}")

    def get_result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info("[Bridge] Navigation succeeded")
        else:
            self.get_logger().info(f"[Bridge] Navigation finished with status {status}")

    def on_yolo_stop(self, msg: Bool):
        self.stop_flag = msg.data
        if self.stop_flag and hasattr(self, "_current_goal_handle"):
            self.get_logger().warn("[Bridge] YOLO stop active, canceling current goal")
            self._current_goal_handle.cancel_goal_async()

    def monitor_yolo(self):
        """Loop to check YOLO detections"""
        rate = 5  # Hz
        while rclpy.ok():
            nearby = self.yolo.detect_nearby_objects(distance_threshold=5.0)
            stop_msg = Bool()
            stop_msg.data = len(nearby) > 0
            self.get_logger().info(f"[YOLO] Nearby objects: {len(nearby)} -> Stop: {stop_msg.data}")
            self.on_yolo_stop(stop_msg)
            time.sleep(1/rate)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
