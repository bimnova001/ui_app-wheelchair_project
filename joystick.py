import tkinter as tk
import math

class VirtualJoystick(tk.Canvas):
    """
    Virtual joystick canvas — ปรับโทนสีได้จาก kwargs (bg, ring_fill, ring_outline, knob_color)
    callback(x, y) จะถูกเรียกขณะลาก และเมื่อปล่อยจะส่ง (0,0)
    x,y อยู่ในช่วง -1..1
    """
    def __init__(self, master, size=200, knob_size=40, callback=None,
                 bg=None, ring_fill=None, ring_outline=None, knob_color=None, **kwargs):
        bg = "#2b2b2b" if bg is None else bg
        ring_fill = "#3a3a3a" if ring_fill is None else ring_fill
        ring_outline = "#6f6f6f" if ring_outline is None else ring_outline
        knob_color = "#1e90ff" if knob_color is None else knob_color

        super().__init__(master, width=size, height=size, bg=bg, bd=0, highlightthickness=0, **kwargs)

        self.size = size
        self.knob_size = knob_size
        self.radius = size // 2
        self.center = self.radius
        self.callback = callback
        self.current_x, self.current_y = 0, 0
        self.holding = False

        pad = 6
        # outer ring (fill + outline)
        self.create_oval(pad, pad, size - pad, size - pad, outline=ring_outline, width=3, fill=ring_fill)

        # knob (circle)
        self.knob = self.create_oval(
            self.center - knob_size//2, self.center - knob_size//2,
            self.center + knob_size//2, self.center + knob_size//2,
            fill=knob_color, outline="#0c0c0c"
        )

        # events
        self.bind("<Button-1>", self.start_hold)
        self.bind("<B1-Motion>", self.move_knob)
        self.bind("<ButtonRelease-1>", self.reset_knob)

    def start_hold(self, event):
        self.holding = True
        # เรียกอัปเดตอย่างต่อเนื่องขณะ holding
        self.update_callback()

    def move_knob(self, event):
        dx, dy = event.x - self.center, event.y - self.center
        distance = math.hypot(dx, dy)
        maxd = self.radius - self.knob_size//2 - 4
        if distance > maxd:
            angle = math.atan2(dy, dx)
            dx = maxd * math.cos(angle)
            dy = maxd * math.sin(angle)

        self.coords(self.knob,
            self.center + dx - self.knob_size//2,
            self.center + dy - self.knob_size//2,
            self.center + dx + self.knob_size//2,
            self.center + dy + self.knob_size//2
        )

        # normalize to -1..1
        if maxd != 0:
            self.current_x = dx / maxd
            self.current_y = dy / maxd
        else:
            self.current_x, self.current_y = 0, 0

        if self.callback:
            # เรียก callback ทุกครั้งที่เคลื่อน
            try:
                self.callback(self.current_x, self.current_y)
            except Exception:
                pass

    def reset_knob(self, event):
        # คืนตำแหน่ง knob กลาง
        self.coords(self.knob,
            self.center - self.knob_size//2,
            self.center - self.knob_size//2,
            self.center + self.knob_size//2,
            self.center + self.knob_size//2
        )
        self.current_x, self.current_y = 0, 0
        self.holding = False
        if self.callback:
            try:
                self.callback(0, 0)
            except Exception:
                pass

    def update_callback(self):
        # เรียก callback เป็นระยะขณะ holding (polling) เพื่อให้ส่งค่าไปที่อื่นได้ต่อเนื่อง
        if self.holding and self.callback:
            try:
                self.callback(self.current_x, self.current_y)
            except Exception:
                pass
            self.after(50, self.update_callback)
