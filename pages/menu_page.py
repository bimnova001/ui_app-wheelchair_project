import customtkinter as ctk
from tkinter import messagebox  # เพิ่ม import นี้

class MenuPage(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        # Layout หลัก
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)

        # Sidebar
        config_sidebar = ctk.CTkFrame(self, width=200, fg_color="#18191c")
        config_sidebar.grid(row=0, column=0, sticky="nsew")
        config_sidebar.grid_propagate(False)

        ctk.CTkLabel(
            config_sidebar, text="Config",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=(20, 10))
        ctk.CTkButton(config_sidebar, text="Connect", fg_color="#b71c1c",
                      command=lambda: messagebox.showinfo("Connect", "Connect clicked")).pack(pady=10, fill="x", padx=20)
        ctk.CTkButton(config_sidebar, text="Settings", fg_color="#b71c1c",
                      command=lambda: messagebox.showinfo("Settings", "Settings clicked")).pack(pady=10, fill="x", padx=20)
        ctk.CTkButton(config_sidebar, text="Shutdown", fg_color="#b71c1c",
                      command=lambda: messagebox.showinfo("Shutdown", "Shutdown clicked")).pack(pady=10, fill="x", padx=20)
        ctk.CTkLabel(config_sidebar, text="Options:", anchor="w").pack(pady=(30, 0), padx=20, anchor="w")

        # เพิ่ม callback ให้กับ CheckBox
        
        self.cb_notify = ctk.CTkCheckBox(
            config_sidebar, text="Enable notifications",
            command=lambda: messagebox.showinfo("Checkbox", f"Enable notifications: {self.cb_notify.get()}")
        )
        self.cb_notify.pack(anchor="w", padx=30, pady=2)
        self.cb_notify.select(1) #เรา่งเลือกค่าเริ่มต้นเป็น True

        self.cb_auto = ctk.CTkCheckBox(
            config_sidebar, text="Auto-connect",
            command=lambda: messagebox.showinfo("Checkbox", f"Auto-connect: {self.cb_auto.get()}")
        )
        self.cb_auto.pack(anchor="w", padx=30, pady=2)

        self.cb_battery = ctk.CTkCheckBox(
            config_sidebar, text="Show battery",
            command=lambda: messagebox.showinfo("Checkbox", f"Show battery: {self.cb_battery.get()}")
        )
        self.cb_battery.pack(anchor="w", padx=30, pady=2)

        # Main Content
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.grid(row=0, column=1, sticky="nsew", padx=0, pady=0)
        main.grid_rowconfigure(2, weight=1)
        main.grid_rowconfigure(3, weight=2)
        main.grid_columnconfigure(0, weight=1)

        # Mode Selector
        ctk.CTkLabel(
            main, text="Mode & configuration",
            font=ctk.CTkFont(size=18, weight="bold")
        ).grid(row=0, column=0, pady=(0, 10), sticky="ew")

        self.menu = ctk.CTkSegmentedButton(
            main, values=["Joystick", "AI config", "SLAM Map"],
            command=self.select_menu
        )
        self.menu.grid(row=1, column=0, pady=(0, 20), ipadx=10, ipady=10, sticky="ew")

        # Map Points Box
        content_box = ctk.CTkFrame(main, fg_color="#232323", corner_radius=12)
        content_box.grid(row=2, column=0, sticky="nsew", padx=10, pady=(0, 10))
        content_box.grid_rowconfigure(1, weight=1)
        content_box.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(
            content_box, text="Map Points",
            font=ctk.CTkFont(size=14, weight="bold")
        ).grid(row=0, column=0, pady=(15, 5), padx=20, sticky="w")

        self.points_frame = ctk.CTkFrame(content_box, fg_color="transparent")
        self.points_frame.grid(row=1, column=0, sticky="nsew", padx=20, pady=(0, 10))
        self.update_points_list()

        # Log/Information (ล่าง)
        log_frame = ctk.CTkFrame(main, fg_color="#18191c", corner_radius=12)
        log_frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=(10, 0))
        log_frame.grid_rowconfigure(1, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(
            log_frame, text="Log / Information",
            font=ctk.CTkFont(size=14, weight="bold")
        ).grid(row=0, column=0, pady=(10, 0), padx=10, sticky="w")

        textbox = ctk.CTkTextbox(log_frame)
        textbox.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        textbox.insert("end", "ข้อความนี้สามารถเลื่อนขึ้นลงได้\n")

        self.bind("<Visibility>", lambda e: self.update_points_list())

    def update_points_list(self):
        for widget in self.points_frame.winfo_children():
            widget.destroy()
        map_page = getattr(self.controller, "frames", {}).get("MapPage")
        points = []
        if map_page and hasattr(map_page, "points"):
            points = map_page.points
        if not points:
            ctk.CTkLabel(self.points_frame, text="ไม่มีจุดที่เพิ่ม", text_color="#888").pack(anchor="w")
        else:
            for idx, (x, y) in enumerate(points):
                ctk.CTkButton(
                    self.points_frame,
                    text=f"Point {idx+1}: ({x},{y})",
                    fg_color="#b71c1c",
                    command=lambda x=x, y=y: self.show_point(x, y)
                ).pack(anchor="w", pady=2, fill="x")

    def show_point(self, x, y):
        messagebox.showinfo("Point Selected", f"คุณเลือกตำแหน่ง\nx={x}, y={y}")

    def select_menu(self, value):
        if value == "Joystick":
            self.controller.show_frame("JoystickPage")
        elif value == "AI config":
            self.controller.show_frame("AIPage")
        elif value == "SLAM Map":
            self.controller.show_frame("MapPage")