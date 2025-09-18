import threading
import cv2
from ultralytics import YOLO
import torch

class YoloBackground:
    def __init__(self, log_func=print, use_gpu=True, img_size=640):
        self.log_func = log_func
        self.img_size = img_size

        # ตรวจสอบ GPU
        self.device = 'cuda' if use_gpu and torch.cuda.is_available() else 'cpu'
        self.model = YOLO("model/yolov8n.pt")
        self.model.fuse()  # fuse conv+BN สำหรับเร็วขึ้น
        self.model.to(self.device)
        self.model.half() if self.device == 'cuda' else None  # ใช้ FP16 บน GPU

        self.cap = None
        self.cam_id = 0
        self.running = False
        self.thread = None
        self.latest_frame = None
        self.detections = []

    def start(self, cam_id=0):
        if self.running:
            return
        self.cam_id = cam_id
        self.cap = cv2.VideoCapture(self.cam_id)
        if not self.cap.isOpened():
            self.log_func(f"[YOLO] Cannot open camera {self.cam_id}")
            return

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.log_func(f"[YOLO] Started on device {self.device}")

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
            self.cap = None
        self.latest_frame = None
        self.detections = []
        self.log_func("[YOLO] Stopped")

    def change_camera(self, cam_id):
        self.stop()
        self.start(cam_id)

    def _loop(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue

            # YOLO inference
            results = self.model.predict(
                frame,
                imgsz=self.img_size,
                device=self.device,
                half=(self.device=='cuda'),
                verbose=False
            )
            annotated_frame = results[0].plot()
            self.latest_frame = annotated_frame

            # เก็บ detections
            self.detections = []
            for det in results[0].boxes.data.tolist():
                x1, y1, x2, y2, conf, cls = det
                self.detections.append({
                    "cls": int(cls),
                    "conf": conf,
                    "bbox": [x1, y1, x2, y2]
                })

    def get_frame(self):
        return self.latest_frame

    def detect_nearby_objects(self, distance_threshold=3.0):
        nearby = []
        for det in self.detections:
            x1, y1, x2, y2 = det["bbox"]
            bbox_width = x2 - x1
            distance_est = 1 / (bbox_width / 640 + 1e-6) * 2
            if distance_est < distance_threshold:
                nearby.append(det)
        return nearby
