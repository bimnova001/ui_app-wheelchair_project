# tf_controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import tensorflow as tf

class TFController(Node):
    """
    Node นี้:
    - โหลดโมเดล Keras .h5
    - รับ /odom (ตำแหน่งปัจจุบัน) และ /goal_pose (ตำแหน่งเป้าหมาย)
    - สร้าง feature vector แล้ว model.predict -> publish /cmd_vel
    """
    def __init__(self, model_path="model.h5"):
        super().__init__('tf_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # โหลดโมเดล
        self.model = tf.keras.models.load_model(model_path)
        self.get_logger().info(f"Loaded TF model: {model_path}")

        self.current_pose = None
        self.goal_pose = None

        # timer loop รัน predict ทุก 0.1s ถ้ามีข้อมูล
        self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # แปลง quaternion -> yaw
        yaw = self.quaternion_to_yaw(q)
        self.current_pose = (p.x, p.y, yaw)

    def goal_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = self.quaternion_to_yaw(q)
        self.goal_pose = (p.x, p.y, yaw)
        self.get_logger().info(f"New goal set: {self.goal_pose}")

    def quaternion_to_yaw(self, q):
        import math
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def build_feature_vector(self):
        # ตัวอย่าง feature vector:
        # [dx, dy, distance, angle_to_goal, yaw_error]
        cx, cy, cyaw = self.current_pose
        gx, gy, gyaw = self.goal_pose
        dx = gx - cx
        dy = gy - cy
        dist = (dx**2 + dy**2)**0.5
        import math
        angle_to_goal = math.atan2(dy, dx)
        angle_err = angle_to_goal - cyaw
        # normalize angle_err to [-pi,pi]
        angle_err = (angle_err + math.pi) % (2*math.pi) - math.pi
        feat = np.array([dx, dy, dist, angle_to_goal, angle_err], dtype=np.float32)
        # ขยายมิติให้เป็น [1, N]
        return feat.reshape(1, -1)

    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None:
            return
        feat = self.build_feature_vector()
        pred = self.model.predict(feat, verbose=0)  # คาดว่า output [linear_x, angular_z]
        linear_x, angular_z = float(pred[0][0]), float(pred[0][1])
        # จำกัดความเร็วไม่ให้เกินค่า safety
        max_lin = 0.5
        max_ang = 1.0
        linear_x = max(-max_lin, min(max_lin, linear_x))
        angular_z = max(-max_ang, min(max_ang, angular_z))

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TFController(model_path="model.h5")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
