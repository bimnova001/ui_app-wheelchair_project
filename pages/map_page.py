import customtkinter as ctk
from PIL import Image, ImageTk
import os

class MapPage(ctk.CTkFrame):
    def __init__(self, parent, controller,
                 pgm_path="assets/map.pgm", yaml_path="assets/map.yaml", png_path="assets/map.png"):
        super().__init__(parent)
        self.controller = controller
        self.points = []

        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)

        ctk.CTkLabel(self, text="SLAM Map", font=ctk.CTkFont(size=18, weight="bold")).grid(row=0, column=0, pady=10, sticky="nw")

        self.canvas = ctk.CTkCanvas(self, width=500, height=500, bg="black")
        self.canvas.grid(row=1, column=0, padx=20, pady=20, sticky="nsew")

        # ตรวจสอบไฟล์ที่มี
        if os.path.exists(pgm_path):
            if not os.path.exists(png_path):
                try:
                    from PIL import Image
                    img = Image.open(pgm_path)
                    img.save(png_path)
                except Exception as e:
                    ctk.CTkMessagebox(title="Error", message=f"ไม่สามารถแปลงไฟล์ PGM เป็น PNG: {e}")
            self.load_map(png_path)
        else:
            self.canvas.create_text(
                250, 250, text="ไม่พบแผนที่ (PGM YAML)", font=("Arial", 16, "bold"), fill="red"
            )

        # Sidebar
        sidebar = ctk.CTkFrame(self, fg_color="#2a2a2a")
        sidebar.grid(row=1, column=1, sticky="ns", padx=10, pady=20)
        sidebar.grid_propagate(False)

        ctk.CTkLabel(sidebar, text="Added Points:", font=ctk.CTkFont(size=14)).pack(pady=5)
        self.buttons_frame = ctk.CTkFrame(sidebar, fg_color="transparent")
        self.buttons_frame.pack(pady=5, fill="x")

        ctk.CTkButton(sidebar, text="⬅ Back to Menu",
                      command=lambda: controller.show_frame("MenuPage")).pack(side="bottom", pady=20, fill="x")

    def load_map(self, png_path):
        img = Image.open(png_path).resize((500, 500))
        self.map_img = ImageTk.PhotoImage(img)
        self.canvas.create_image(0, 0, anchor="nw", image=self.map_img)
        self.canvas.bind("<Button-1>", self.add_point)

    def add_point(self, event):
        x, y = event.x, event.y
        self.points.append((x, y))
        self.canvas.create_oval(x-4, y-4, x+4, y+4, fill="red")
        btn = ctk.CTkButton(self.buttons_frame, text=f"Point ({x},{y})",
                            command=lambda x=x, y=y: self.show_point(x, y))
        btn.pack(pady=2, fill="x")

    def show_point(self, x, y):
        ctk.CTkMessagebox(title="Point Selected", message=f"คุณเลือกตำแหน่ง\nx={x}, y={y}")
