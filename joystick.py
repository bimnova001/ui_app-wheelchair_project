import tkinter as tk
import math

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
