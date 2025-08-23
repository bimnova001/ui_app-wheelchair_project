import customtkinter as ctk
from joystick import VirtualJoystick

class JoystickPage(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(self, text="Joystick Control", font=ctk.CTkFont(size=18, weight="bold")).grid(row=0, column=0, pady=10, sticky="n")
        self.status = ctk.CTkLabel(self, text="x=0.00, y=0.00", font=ctk.CTkFont(size=14))
        self.status.grid(row=1, column=0, pady=5, sticky="n")

        joystick = VirtualJoystick(self, size=220, knob_size=50, callback=self.update_status)
        joystick.grid(row=2, column=0, padx=20, pady=20, sticky="n")

        ctk.CTkButton(self, text="⬅ Back to Menu", command=lambda: controller.show_frame("MenuPage")).grid(row=3, column=0, pady=15, sticky="s")

    def update_status(self, x, y):
        self.status.configure(text=f"x={x:.2f}, y={y:.2f}")
