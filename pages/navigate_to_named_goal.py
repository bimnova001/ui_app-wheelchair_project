import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_msgs.msg import String
import json
import math

class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')
        self.get_logger().info("[Bridge] Starting Nav2Bridge node")

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ตำแหน่งปัจจุบัน
        self.current_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)

        # Goal pose
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        
        self.create_subscription(String, '/yolo/detections', self.on_yolo_detection, 10)


        # Stop flag
        self.stop_flag = False
        self.create_subscription(Bool, '/yolo_stop', self.on_yolo_stop, 10)

        self.get_logger().info("[Bridge] Node initialized")

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def on_goal_pose(self, msg: PoseStamped):
        if self.stop_flag:
            self.get_logger().warn("[Bridge] Cannot send goal, stop flag active")
            return

        x_goal = msg.pose.position.x
        y_goal = msg.pose.position.y

        if self.current_pose:
            x_curr = self.current_pose.position.x
            y_curr = self.current_pose.position.y
            distance = math.sqrt((x_goal - x_curr)**2 + (y_goal - y_curr)**2)
            self.get_logger().info(
                f"[Bridge] Goal received: (x={x_goal:.2f}, y={y_goal:.2f}) "
                f"Current: (x={x_curr:.2f}, y={y_curr:.2f}) "
                f"Distance: {distance:.2f} m"
            )
        else:
            self.get_logger().warn("[Bridge] Current pose not available yet, cannot compute distance")
            distance = None

        if not self._action_client.wait_for_server(timeout_sec=10):
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
        self.get_logger().info(
            f"[Bridge] Remaining: {getattr(feedback, 'distance_remaining', float('nan')):.2f} m"
        )

    def get_result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info("[Bridge] Navigation succeeded")
        elif status == 6:
            self.get_logger().info("[Bridge] Navigation CANCELED")
        else:
            self.get_logger().info(f"[Bridge] Navigation finished with status {status}")
        self.get_logger().info(f"Result : {result}")
        
    def on_yolo_detection(self, msg: String):
        """
        msg.data = JSON list of objects with relative x,y from robot
        """
        try:
            det_list = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("[Bridge] Cannot decode YOLO detection")
            return

        stop = False
        for det in det_list:
            x = det.get('x', 100)  # fallback far away
            y = det.get('y', 100)
            distance = math.sqrt(x**2 + y**2)
            if distance <= 5.0:
                stop = True
                self.get_logger().warn(f"[Bridge] Object '{det.get('name')}' detected at {distance:.2f} m, stopping")
                break

        # Update stop flag
        self.stop_flag = stop
        if self.stop_flag and hasattr(self, "_current_goal_handle"):
            self.get_logger().warn("[Bridge] Stop flag active, canceling current goal")
            self._current_goal_handle.cancel_goal_async()

    def on_yolo_stop(self, msg: Bool):
        self.stop_flag = msg.data
        if self.stop_flag and hasattr(self, "_current_goal_handle"):
            self.get_logger().warn("[Bridge] Stop flag active, canceling current goal")
            self._current_goal_handle.cancel_goal_async()

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
