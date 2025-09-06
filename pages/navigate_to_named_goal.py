# navigate_to_named_goal.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor

class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.get_logger().info("Nav2Bridge ready, waiting for /goal_pose messages")

    def on_goal_pose(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}")
        # รอ action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected :(")
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_cb)

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        # feedback contains distance_remaining etc
        self.get_logger().info(f"Feedback: remaining {getattr(feedback, 'distance_remaining', '?')}")

    def get_result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # SUCCEEDED constant from action server (client maps codes differently)
            self.get_logger().info("Navigation succeeded")
        else:
            self.get_logger().info(f"Navigation finished with status {status}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
