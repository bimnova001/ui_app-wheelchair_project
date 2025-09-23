import customtkinter as ctk
from PIL import Image, ImageTk
import cv2
import threading

class CameraPage(ctk.CTkFrame):
    def __init__(self, parent, controller, yolo_bg):
        super().__init__(parent)
        self.controller = controller
        self.yolo = yolo_bg

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

        # Start update loop
        self.update_frame()

    def change_camera(self):
        cam_id = self.cam_var.get()
        self.yolo.change_camera(cam_id)

    def update_frame(self):
        if not self.yolo.active:
            self.preview_canvas.delete("all")
            self.after(100, self.update_frame)
            return
        frame = self.yolo.get_frame()
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)

            # แสดงบน canvas
            self.preview_canvas.delete("all")
            self.preview_canvas.create_image(0, 0, anchor="nw", image=imgtk)
            self.preview_canvas.imgtk = imgtk  # ต้องเก็บ reference

            # วาด bounding box และเส้นระยะ
            for det in self.yolo.detections:
                x1, y1, x2, y2 = det["bbox"]
                cls_id = det["cls"]
                conf = det["conf"]

                # วาดกรอบสี่เหลี่ยม
                self.preview_canvas.create_rectangle(
                    x1, y1, x2, y2,
                    outline="red", width=2
                )

                # แสดง label + confidence
                self.preview_canvas.create_text(
                    x1 + 5, y1 - 10,
                    text=f"ID:{cls_id} {conf:.2f}",
                    anchor="nw",
                    fill="yellow",
                    font=("Arial", 10, "bold")
                )

                # วาดเส้นระยะ (จากกลาง frame ลงไปที่วัตถุ)
                frame_h, frame_w, _ = frame.shape
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                self.preview_canvas.create_line(
                    frame_w/2, frame_h, cx, cy,
                    fill="lime", width=2
                )

        # เรียกตัวเองซ้ำทุก 30ms
        self.after(30, self.update_frame)
