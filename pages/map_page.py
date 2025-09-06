# map_page.py
import customtkinter as ctk
from PIL import Image, ImageTk
import os, json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

POINTS_FILE = "assets/points.json"

class ROS2GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # cmd_vel publisher for manual motor control
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)

    def send_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)


class MapPage(ctk.CTkFrame):
    def __init__(self, parent, controller,
                 pgm_path="assets/map.pgm", yaml_path="assets/map.yaml", png_path="assets/map.png"):
        super().__init__(parent)
        self.controller = controller
        self.points = []

        # layout
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)

        ctk.CTkLabel(self, text="SLAM Map", font=ctk.CTkFont(size=18, weight="bold")).grid(row=0, column=0, pady=10, sticky="nw")

        self.canvas = ctk.CTkCanvas(self, width=500, height=500, bg="black")
        self.canvas.grid(row=1, column=0, padx=20, pady=20, sticky="nsew")

        # convert pgm->png if needed
        if os.path.exists(pgm_path):
            if not os.path.exists(png_path):
                try:
                    img = Image.open(pgm_path)
                    img.save(png_path)
                except Exception as e:
                    ctk.CTkMessagebox(title="Error", message=f"ไม่สามารถแปลงไฟล์ PGM เป็น PNG: {e}")
            self.load_map(png_path)
        else:
            self.canvas.create_text(250, 250, text="ไม่พบแผนที่ (PGM YAML)", font=("Arial", 16, "bold"), fill="red")

        # sidebar
        sidebar = ctk.CTkFrame(self, fg_color="#2a2a2a")
        sidebar.grid(row=1, column=1, sticky="ns", padx=10, pady=20)
        sidebar.grid_propagate(False)

        ctk.CTkLabel(sidebar, text="Selected Points:", font=ctk.CTkFont(size=14)).pack(pady=5)
        self.buttons_frame = ctk.CTkFrame(sidebar, fg_color="transparent")
        self.buttons_frame.pack(pady=5, fill="x")

        ctk.CTkButton(sidebar, text="⬅ Back to Menu", command=lambda: controller.show_frame("MenuPage")).pack(side="bottom", pady=20, fill="x")

        # ROS2 node (สร้างเมื่อเข้า MapPage)
        # rclpy.init ควรเรียกแค่ครั้งเดียวในโปรแกรมหลักก่อนสร้าง widget หลายตัว
        try:
            rclpy.get_default_context()
        except Exception:
            pass
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros2_node = ROS2GoalPublisher()

        # motor control buttons
        ctk.CTkLabel(sidebar, text="Motor Control:", font=ctk.CTkFont(size=14)).pack(pady=(20, 5))
        ctk.CTkButton(sidebar, text="Forward", command=self.motor_forward).pack(fill="x", pady=2)
        ctk.CTkButton(sidebar, text="Backward", command=self.motor_backward).pack(fill="x", pady=2)
        ctk.CTkButton(sidebar, text="Stop", command=self.motor_stop).pack(fill="x", pady=2)

        # load saved points if file exists
        self.load_points()

    def load_map(self, png_path):
        img = Image.open(png_path).resize((500, 500))
        self.map_img = ImageTk.PhotoImage(img)
        self.canvas.create_image(0, 0, anchor="nw", image=self.map_img)
        self.canvas.bind("<Button-1>", self.add_point)

    def add_point(self, event):
        x, y = event.x, event.y
        # บันทึกทั้งพิกัดผืนผ้า/พิกัดจริง: ตอนนี้เป็น pixel -> ต้องแปลงเป็น map coordinate ถ้าต้องการความแม่นยำ
        self.points.append({'x': x, 'y': y, 'name': f"Point {len(self.points)+1}"})
        self.canvas.create_oval(x-6, y-6, x+6, y+6, fill="red")
        self.refresh_buttons()
        self.save_points()

    def refresh_buttons(self):
        # เคลียร์ปุ่มเก่า
        for widget in self.buttons_frame.winfo_children():
            widget.destroy()
        # สร้างปุ่มใหม่
        for i, p in enumerate(self.points):
            name = p.get('name', f"Point {i+1}")
            btn = ctk.CTkButton(self.buttons_frame, text=name, command=lambda idx=i: self.go_to_point(idx))
            btn.pack(pady=4, fill="x")

    def go_to_point(self, index):
        p = self.points[index]
        # NOTE: ที่นี่ p.x/p.y เป็น pixel; ต้องแปลงเป็นพิกัด map จริง (meter) ก่อนส่งให้ nav
        # สมมติว่ามีฟังก์ชัน pixel_to_map(x,y) ที่แปลงพิกัด
        try:
            map_x, map_y = self.pixel_to_map(p['x'], p['y'])
            self.ros2_node.send_goal(map_x, map_y)
            ctk.CTkMessagebox(title="Goal Sent", message=f"ส่งตำแหน่งไป ROS2: x={map_x:.2f}, y={map_y:.2f}")
        except Exception as e:
            ctk.CTkMessagebox(title="Error", message=f"ส่ง goal ไม่สำเร็จ: {e}")

    # ตัวอย่างการแปลง pixel -> map coordinate (ต้องแก้ตามข้อมูล map.yaml ของคุณ)
    def pixel_to_map(self, px, py):
        # ตัวอย่างสมมติ: map image ขนาด 500x500 แสดงพื้นที่จริง 10m x 10m แก้ตามจริง!
        map_width_m = 10.0
        map_height_m = 10.0
        img_w = 500
        img_h = 500
        mx = (px / img_w) * map_width_m  # แปลงเป็นเมตร
        my = ((img_h - py) / img_h) * map_height_m  # หมุนแกน y
        return mx, my

    # motor functions (publish Twist)
    def motor_forward(self):
        self.ros2_node.send_cmd_vel(0.2, 0.0)  # เดินหน้า 0.2 m/s

    def motor_backward(self):
        self.ros2_node.send_cmd_vel(-0.1, 0.0)  # ถอยหลัง

    def motor_stop(self):
        self.ros2_node.send_cmd_vel(0.0, 0.0)

    # Save/load points to JSON
    def save_points(self):
        try:
            os.makedirs(os.path.dirname(POINTS_FILE), exist_ok=True)
            with open(POINTS_FILE, "w", encoding="utf-8") as f:
                json.dump(self.points, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print("Save points error:", e)

    def load_points(self):
        if os.path.exists(POINTS_FILE):
            try:
                with open(POINTS_FILE, "r", encoding="utf-8") as f:
                    self.points = json.load(f)
                # draw points on canvas
                for p in self.points:
                    x, y = p['x'], p['y']
                    self.canvas.create_oval(x-6, y-6, x+6, y+6, fill="red")
                self.refresh_buttons()
            except Exception as e:
                print("Load points error:", e)

    def show_point(self, x, y):
        ctk.CTkMessagebox(title="Point Selected", message=f"คุณเลือกตำแหน่ง\nx={x}, y={y}")


if __name__ == "__main__":
    ROS2GoalPublisher()