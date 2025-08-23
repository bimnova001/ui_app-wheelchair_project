import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import math

# ================== Virtual Joystick ==================
class VirtualJoystick(tk.Canvas):
    def __init__(self, master, size=200, knob_size=40, callback=None, **kwargs):
        super().__init__(master, width=size, height=size, bg="#e0e0e0", bd=0, highlightthickness=0, **kwargs)
        
        self.size = size
        self.knob_size = knob_size
        self.radius = size // 2
        self.center = self.radius
        self.callback = callback
        self.current_x, self.current_y = 0, 0
        self.holding = False

        # background
        self.create_oval(5, 5, size-5, size-5, outline="gray", width=3, fill="#fafafa")
        
        # knob
        self.knob = self.create_oval(
            self.center - knob_size//2, self.center - knob_size//2,
            self.center + knob_size//2, self.center + knob_size//2,
            fill="#0078d7", outline="black"
        )

        self.bind("<Button-1>", self.start_hold)
        self.bind("<B1-Motion>", self.move_knob)
        self.bind("<ButtonRelease-1>", self.reset_knob)

    def start_hold(self, event):
        self.holding = True
        self.update_callback()

    def move_knob(self, event):
        dx, dy = event.x - self.center, event.y - self.center
        distance = math.sqrt(dx*dx + dy*dy)

        if distance > self.radius - self.knob_size//2:
            angle = math.atan2(dy, dx)
            dx = (self.radius - self.knob_size//2) * math.cos(angle)
            dy = (self.radius - self.knob_size//2) * math.sin(angle)

        self.coords(self.knob,
            self.center + dx - self.knob_size//2,
            self.center + dy - self.knob_size//2,
            self.center + dx + self.knob_size//2,
            self.center + dy + self.knob_size//2
        )

        self.current_x = dx / (self.radius - self.knob_size//2)
        self.current_y = dy / (self.radius - self.knob_size//2)

    def reset_knob(self, event):
        self.coords(self.knob,
            self.center - self.knob_size//2,
            self.center - self.knob_size//2,
            self.center + self.knob_size//2,
            self.center + self.knob_size//2
        )
        self.current_x, self.current_y = 0, 0
        self.holding = False
        if self.callback: self.callback(0, 0)

    def update_callback(self):
        if self.holding and self.callback:
            self.callback(self.current_x, self.current_y)
            self.after(50, self.update_callback)

# ================== Main App ==================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Control System")
        self.geometry("900x600")
        self.configure(bg="#202020")

        style = ttk.Style()
        style.configure("TButton", font=("Arial", 12), padding=6)

        self.container = tk.Frame(self, bg="#202020")
        self.container.pack(fill="both", expand=True)

        self.frames = {}
        for F in (MenuPage, JoystickPage, AIPage, MapPage):
            page_name = F.__name__
            frame = F(parent=self.container, controller=self)
            self.frames[page_name] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("MenuPage")

    def show_frame(self, page_name):
        frame = self.frames[page_name]
        frame.tkraise()

# ================== Menu Page ==================
class MenuPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="#202020")
        self.controller = controller

        tk.Label(self, text="🚀 Robot Control Menu", font=("Arial", 24, "bold"),
                 bg="#202020", fg="white").pack(pady=40)

        ttk.Button(self, text="🎮 Joystick Mode", 
                   command=lambda: controller.show_frame("JoystickPage")).pack(pady=10)
        ttk.Button(self, text="🤖 AI Mode", 
                   command=lambda: controller.show_frame("AIPage")).pack(pady=10)
        ttk.Button(self, text="🗺 Map (SLAM)", 
                   command=lambda: controller.show_frame("MapPage")).pack(pady=10)

# ================== Joystick Page ==================
class JoystickPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="#303030")
        self.controller = controller

        tk.Label(self, text="Joystick Control", font=("Arial", 18, "bold"),
                 bg="#303030", fg="white").pack(pady=10)

        self.status = tk.Label(self, text="x=0.00, y=0.00", font=("Arial", 14),
                               bg="#303030", fg="white")
        self.status.pack(pady=5)

        joystick = VirtualJoystick(self, size=220, knob_size=50, callback=self.update_status)
        joystick.pack(padx=20, pady=20)

        ttk.Button(self, text="⬅ Back to Menu", 
                   command=lambda: controller.show_frame("MenuPage")).pack(pady=15)

    def update_status(self, x, y):
        self.status.config(text=f"x={x:.2f}, y={y:.2f}")

# ================== AI Page (ยังไม่ทำ) ==================
class AIPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="#202020")
        tk.Label(self, text="AI Mode (ยังไม่ทำ)", font=("Arial", 18), bg="#202020", fg="white").pack(pady=30)
        ttk.Button(self, text="⬅ Back to Menu", 
                   command=lambda: controller.show_frame("MenuPage")).pack(pady=20)

# ================== Map Page ==================
class MapPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg="#202020")
        self.controller = controller
        self.points = []  # เก็บพิกัดที่เลือก

        tk.Label(self, text="SLAM Map", font=("Arial", 18, "bold"), bg="#202020", fg="white").pack(pady=10)

        # โหลดภาพ map (ใช้ภาพทดสอบแทน)
        img = Image.open("map.png").resize((500, 500))  # เตรียมไฟล์ map.png ไว้
        self.map_img = ImageTk.PhotoImage(img)

        self.canvas = tk.Canvas(self, width=500, height=500, bg="black")
        self.canvas.create_image(0, 0, anchor="nw", image=self.map_img)
        self.canvas.pack(side="left", padx=20, pady=20)

        self.canvas.bind("<Button-1>", self.add_point)

        # Sidebar
        self.sidebar = tk.Frame(self, bg="#2a2a2a")
        self.sidebar.pack(side="left", fill="y", padx=10, pady=20)

        self.point_list = tk.Label(self.sidebar, text="Added Points:", font=("Arial", 14), bg="#2a2a2a", fg="white")
        self.point_list.pack(pady=5)

        self.buttons_frame = tk.Frame(self.sidebar, bg="#2a2a2a")
        self.buttons_frame.pack(pady=5)

        ttk.Button(self.sidebar, text="⬅ Back to Menu", 
                   command=lambda: controller.show_frame("MenuPage")).pack(side="bottom", pady=20)

    def add_point(self, event):
        x, y = event.x, event.y
        self.points.append((x, y))

        # วาดจุดบน map
        self.canvas.create_oval(x-4, y-4, x+4, y+4, fill="red")

        # อัพเดตปุ่ม
        btn = ttk.Button(self.buttons_frame, text=f"Point ({x},{y})", 
                         command=lambda x=x, y=y: self.show_point(x, y))
        btn.pack(pady=2, fill="x")

    def show_point(self, x, y):
        print(f"Selected Point: x={x}, y={y}")
        tk.messagebox.showinfo("Point Selected", f"คุณเลือกตำแหน่ง\nx={x}, y={y}")

# ================== Run ==================
if __name__ == "__main__":
    app = App()
    app.mainloop()
