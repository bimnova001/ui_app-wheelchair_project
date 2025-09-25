import customtkinter as ctk
from PIL import Image, ImageTk
import cv2
import threading
import roslibpy  # ต้องติดตั้ง roslibpy

class CameraPage(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        # Title
        ctk.CTkLabel(self, text="Camara Ai", font=ctk.CTkFont(size=20, weight="bold")).pack(pady=10)

        # Camera selection
        self.cam_list = [0, 1, 2, 3]
        self.cam_var = ctk.IntVar(value=self.cam_list[0])
        for cam_id in self.cam_list:
            ctk.CTkRadioButton(
                self,
                text=f"Camera {cam_id}",
                variable=self.cam_var,
                value=cam_id,
                command=self.change_camera
            ).pack(anchor="w", padx=20)

        # Canvas for preview
        self.preview_canvas = ctk.CTkCanvas(self, width=640, height=480)
        self.preview_canvas.pack(pady=10)

        # Back button
        ctk.CTkButton(
            self,
            text="⬅ Back to Menu",
            command=lambda: controller.show_frame("MenuPage")
        ).pack(pady=15, side="bottom")

        # เตรียมตัวแปรสำหรับภาพและ detection
        self.frame = None
        self.detections = []

        # เริ่ม thread สำหรับ subscribe ROS2
        self.ros = roslibpy.Ros(host='localhost', port=9090)
        self.ros.run()
        self.image_topic = roslibpy.Topic(self.ros, '/yolo/image/compressed', 'sensor_msgs/CompressedImage')
        self.detection_topic = roslibpy.Topic(self.ros, '/yolo/detections', 'std_msgs/String')
        self.image_topic.subscribe(self.on_image)
        self.detection_topic.subscribe(self.on_detection)

        self.update_frame()

    def change_camera(self):
        cam_id = self.cam_var.get()
        self.yolo.change_camera(cam_id)

    def on_image(self, msg):
        import base64
        import numpy as np
        img_bytes = base64.b64decode(msg['data'])
        np_arr = np.frombuffer(img_bytes, np.uint8)
        self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def on_detection(self, msg):
        import json
        try:
            self.detections = json.loads(msg['data'])
        except Exception:
            self.detections = []

    def update_frame(self):
        frame = self.frame
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.preview_canvas.delete("all")
            self.preview_canvas.create_image(0, 0, anchor="nw", image=imgtk)
            self.preview_canvas.imgtk = imgtk

            # วาด bounding box
            for det in self.detections:
                x1, y1, x2, y2 = det["bbox"]
                cls_id = det["cls"]
                conf = det["conf"]
                self.preview_canvas.create_rectangle(
                    x1, y1, x2, y2,
                    outline="red", width=2
                )
                self.preview_canvas.create_text(
                    x1 + 5, y1 - 10,
                    text=f"ID:{cls_id} {conf:.2f}",
                    anchor="nw",
                    fill="yellow",
                    font=("Arial", 10, "bold")
                )
        self.after(30, self.update_frame)
