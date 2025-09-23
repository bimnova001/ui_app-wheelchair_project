# map_page.py
import customtkinter as ctk
from customtkinter import CTkInputDialog
from PIL import Image, ImageTk
import os, json, threading, yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from tkinter import messagebox

POINTS_FILE = "assets/points.json"

class ROS2GoalPublisher(Node):  
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
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

        # read map.yaml for resolution/origin
        self.resolution = 0.05  # default
        self.origin = [0,0,0]
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, "r") as f:
                    map_data = yaml.safe_load(f)
                    self.resolution = map_data.get("resolution", 0.05)
                    self.origin = map_data.get("origin", [0,0,0])
            except Exception as e:
                print("Error reading map.yaml:", e)

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
                    messagebox.showerror("Error", f"ไม่สามารถแปลงไฟล์ PGM เป็น PNG: {e}")
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
        ctk.CTkButton(sidebar, text="Scan Map", command=self.scan_map).pack(side="bottom", pady=5, fill="x")
        ctk.CTkButton(sidebar, text="Save Map", command=self.save_map).pack(side="bottom", pady=5, fill="x")

        # ROS2 node
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros2_node = ROS2GoalPublisher()

        # motor control
        ctk.CTkLabel(sidebar, text="Motor Control:", font=ctk.CTkFont(size=14)).pack(pady=(20, 5))
        ctk.CTkButton(sidebar, text="Forward", command=self.motor_forward).pack(fill="x", pady=2)
        ctk.CTkButton(sidebar, text="Backward", command=self.motor_backward).pack(fill="x", pady=2)
        ctk.CTkButton(sidebar, text="Stop", command=self.motor_stop).pack(fill="x", pady=2)

        # load saved points
        self.load_points()

    def load_map(self, png_path):
        img = Image.open(png_path).resize((500, 500))
        self.map_img = ImageTk.PhotoImage(img)
        self.canvas.create_image(0, 0, anchor="nw", image=self.map_img)
        self.canvas.bind("<Button-1>", self.add_point)

    def add_point(self, event):
        x, y = event.x, event.y
        map_x, map_y = self.pixel_to_map(x, y)

        # ถามชื่อ point
        dialog = CTkInputDialog(text="กรุณาตั้งชื่อ Point:", title="Add Point")
        name = dialog.get_input()
        if not name:  # ถ้า user กด cancel หรือไม่ใส่ชื่อ
            return

        self.points.append({
            'x': x, 'y': y,
            'map_x': map_x, 'map_y': map_y,
            'name': name
        })
        self.canvas.create_oval(x-6, y-6, x+6, y+6, fill="red")
        self.refresh_buttons()
        self.save_points()


    def refresh_buttons(self):
        for widget in self.buttons_frame.winfo_children():
            widget.destroy()

        for i, p in enumerate(self.points):
            name = p.get('name', f"Point {i+1}")

            row = ctk.CTkFrame(self.buttons_frame, fg_color="transparent")
            row.pack(pady=2, fill="x")

            # ปุ่มไปยังจุด
            btn = ctk.CTkButton(row, text=name, command=lambda idx=i: self.go_to_point(idx))
            btn.pack(side="left", fill="x", expand=True, padx=(0, 5))

            # ปุ่มลบ
            del_btn = ctk.CTkButton(row, text="✕", width=30, fg_color="#b71c1c",
                                    command=lambda idx=i: self.delete_point(idx))
            del_btn.pack(side="right")


    def delete_point(self, index):
        self.points.pop(index)
        self.refresh_buttons()
        self.save_points()
        self.redraw_points()

    def redraw_points(self):
        self.canvas.delete("all")
        self.load_map("assets/map.png")
        for p in self.points:
            x, y = p['x'], p['y']
            self.canvas.create_oval(x-6, y-6, x+6, y+6, fill="red")

    def go_to_point(self, index):
        p = self.points[index]
        def send_goal_thread():
            try:
                self.ros2_node.send_goal(p['map_x'], p['map_y'])
                messagebox.showinfo("Goal Sent", f"ส่งตำแหน่งไป ROS2: x={p['map_x']:.2f}, y={p['map_y']:.2f}")
            except Exception as e:
                messagebox.showerror("Error", f"ส่ง goal ไม่สำเร็จ: {e}")
        threading.Thread(target=send_goal_thread, daemon=True).start()

    def pixel_to_map(self, px, py):
        img_w, img_h = 500, 500
        mx = self.origin[0] + px * self.resolution
        my = self.origin[1] + (img_h - py) * self.resolution
        return mx, my

    def motor_forward(self):
        threading.Thread(target=lambda: self.ros2_node.send_cmd_vel(0.2, 0.0), daemon=True).start()

    def motor_backward(self):
        threading.Thread(target=lambda: self.ros2_node.send_cmd_vel(-0.1, 0.0), daemon=True).start()

    def motor_stop(self):
        threading.Thread(target=lambda: self.ros2_node.send_cmd_vel(0.0, 0.0), daemon=True).start()

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
                self.redraw_points()
                self.refresh_buttons()
            except Exception as e:
                print("Load points error:", e)
    def get_points_list(self):
        # คืน list ของ points พร้อม map_x, map_y
        return self.points


if __name__ == "__main__":
    ROS2GoalPublisher()
