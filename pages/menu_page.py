import subprocess
import sys
import customtkinter as ctk
from tkinter import messagebox

class MenuPage(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        # กำหนด function/checkbox ที่ต้องการจัดการและแสดง status
        self.func_list = [
            {"name": "Check ros2", "cmd": ["python", "--version"]},
            {"name": "Save Map","cmd": ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "/home/user/assets/map"]}
        ]
        self.checkbox_list = [
            {"name": "Auto mode"},
            {"name": "Manual mode"},
            {"name": "Show battery"},
        ]
        
        self.func_list_start = [
            {"name": "Start LiDAR A2", "cmd": ["ros2", "launch", "rplidar_ros", "rplidar.launch.py",
                                                "serial_port:=/dev/ttyUSB0",
                                                "frame_id:=laser_frame"]},
            {"name": "Start SLAM", "cmd": ["ros2", "launch", "slam_toolbox", "online_async_launch.py"]},
            {"name": "Start Navigation", "cmd": ["ros2", "launch", "nav2_bringup", "bringup_launch.py",
                                                "map:=/home/user/assets/map.yaml"]},
        ]


        # Layout หลัก
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=0)

        # Sidebar
        config_sidebar = ctk.CTkFrame(self, width=200, fg_color="#18191c")
        config_sidebar.grid(row=0, column=0, sticky="nsew")
        config_sidebar.grid_propagate(False)

        ctk.CTkLabel(
            config_sidebar, text="Config",
            font=ctk.CTkFont(size=20, weight="bold")
        ).pack(pady=(20, 10))

        # ปุ่มฟังก์ชัน
        for func in self.func_list:
            ctk.CTkButton(
                config_sidebar, text=func["name"], fg_color="#b71c1c",
                command=lambda f=func: self.on_func_click(f)
            ).pack(pady=10, fill="x", padx=20)

        ctk.CTkLabel(config_sidebar, text="Options:", anchor="w").pack(pady=(30, 0), padx=20, anchor="w")

        # Checkbox พร้อม status
        self.checkbox_vars = {}
        self.checkbox_status = {}
        for cb in self.checkbox_list:
            var = ctk.IntVar(value=1 if cb["name"] == "Auto mode" else 0)
            frame = ctk.CTkFrame(config_sidebar, fg_color="transparent")
            frame.pack(anchor="w", padx=20, pady=2, fill="x")
            cb_widget = ctk.CTkCheckBox(
                frame, text=cb["name"], variable=var,
                command=lambda name=cb["name"]: self.on_checkbox_toggle(name)
            )
            cb_widget.pack(side="left", anchor="w")
            status = ctk.CTkLabel(frame, text="●")
            status.pack(side="right", padx=5)
            self.checkbox_vars[cb["name"]] = var
            self.checkbox_status[cb["name"]] = status


        ctk.CTkButton(config_sidebar, text="App Config",
                      fg_color="#444", command=lambda: controller.show_frame("AppConfigPage")).pack(pady=10, fill="x", padx=20)

        # Main Content (เหมือนเดิม)
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.grid(row=0, column=1, sticky="nsew", padx=0, pady=0)
        main.grid_rowconfigure(2, weight=1)
        main.grid_rowconfigure(3, weight=2)
        main.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(
            main, text="Mode & configuration",
            font=ctk.CTkFont(size=18, weight="bold")
        ).grid(row=0, column=0, pady=(0, 10), sticky="ew")

        self.menu = ctk.CTkSegmentedButton(
            main, values=["Joystick", "AI config", "SLAM Map"],
            command=self.select_menu
        )
        self.menu.grid(row=1, column=0, pady=(0, 20), ipadx=10, ipady=10, sticky="ew")

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

        # Status Panel (ขวาสุด)
        status_panel = ctk.CTkFrame(self, width=180, fg_color="#232323")
        status_panel.grid(row=0, column=2, sticky="nsew", padx=(10, 10), pady=10)
        status_panel.grid_propagate(False)

        ctk.CTkLabel(status_panel, text="Status", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(20, 10))

        # Status สำหรับฟังก์ชัน
        self.status_funcs = {}
        for func in self.func_list:
            frame = ctk.CTkFrame(status_panel, fg_color="transparent")
            frame.pack(fill="x", pady=5, padx=10)
            label = ctk.CTkLabel(frame, text=func["name"], font=ctk.CTkFont(size=14), anchor="w")
            label.pack(side="left", fill="x", expand=True)
            status = ctk.CTkLabel(frame, text="●", font=ctk.CTkFont(size=16), text_color="#44ff44")
            status.pack(side="right")
            self.status_funcs[func["name"]] = status

        # Status สำหรับ checkbox
        ctk.CTkLabel(status_panel, text="Options", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(20, 0))
        self.status_checkbox = {}
        for cb in self.checkbox_list:
            frame = ctk.CTkFrame(status_panel, fg_color="transparent")
            frame.pack(fill="x", pady=2, padx=10)
            label = ctk.CTkLabel(frame, text=cb["name"], font=ctk.CTkFont(size=12), anchor="w")
            label.pack(side="left", fill="x", expand=True)
            status = ctk.CTkLabel(frame, text="●", font=ctk.CTkFont(size=14),
                                  text_color="#44ff44" if self.checkbox_vars[cb["name"]].get() else "#ff4444")
            status.pack(side="right")
            self.status_checkbox[cb["name"]] = status
            
            
        ctk.CTkLabel(status_panel, text="Startup Nodes", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(20, 0))
        self.status_start_funcs = {}
        for func in self.func_list_start:
            frame = ctk.CTkFrame(status_panel, fg_color="transparent")
            frame.pack(fill="x", pady=2, padx=10)
            label = ctk.CTkLabel(frame, text=func["name"], font=ctk.CTkFont(size=12), anchor="w")
            label.pack(side="left", fill="x", expand=True)
            status = ctk.CTkLabel(frame, text="●", font=ctk.CTkFont(size=14), text_color="#ffcc00")
            status.pack(side="right")
            self.status_start_funcs[func["name"]] = status


        # ตั้งค่าเริ่มต้น
        for func in self.func_list:
            self.set_func_status(func["name"], True)
        for cb in self.checkbox_list:
            self.set_checkbox_status(cb["name"], self.checkbox_vars[cb["name"]].get())
            
            self.after(100, self.auto_run_commands)

    def auto_run_commands(self):
        for func in self.func_list_start:
            self.on_func_click(func)
            

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

    def set_func_status(self, func, ok=True):
        if func in self.status_funcs:
            color = "#44ff44" if ok else "#ff4444"
            self.status_funcs[func].configure(text_color=color)

    def set_checkbox_status(self, name, ok=True):
        if name in self.status_checkbox:
            color = "#44ff44" if ok else "#ff4444"
            self.status_checkbox[name].configure(text_color=color)
            
    def set_start_func_status(self, func, ok=True):
        if func in self.status_start_funcs:
            color = "#44ff44" if ok else "#ff4444"
            self.status_start_funcs[func].configure(text_color=color)

    
    def call_func_by_name(self, func_name):
        # ค้นหา dict ใน self.func_list ที่ name ตรงกับ func_name
        func = next((f for f in self.func_list if f["name"] == func_name), None)
        if func:
            self.on_func_click(func)
        else:
            messagebox.showerror("Error", f"Function '{func_name}' not found")

    def on_func_click(self, func):
        try:
            result = subprocess.run(func["cmd"], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                # อัปเดตสถานะ
                if func["name"] in self.status_funcs:
                    self.set_func_status(func["name"], True)
                elif func["name"] in self.status_start_funcs:
                    self.set_start_func_status(func["name"], True)

                messagebox.showinfo(func["name"], f"{func['name']} success:\n{result.stdout.strip()}")
            else:
                if func["name"] in self.status_funcs:
                    self.set_func_status(func["name"], False)
                elif func["name"] in self.status_start_funcs:
                    self.set_start_func_status(func["name"], False)

                messagebox.showerror(func["name"], f"{func['name']} error:\n{result.stderr.strip()}")
        except Exception as e:
            if func["name"] in self.status_funcs:
                self.set_func_status(func["name"], False)
            elif func["name"] in self.status_start_funcs:
                self.set_start_func_status(func["name"], False)

            messagebox.showerror(func["name"], f"{func['name']} error!\n{e}")


    def on_checkbox_toggle(self, name):
        # อัปเดตสถานะ checkbox
        value = self.checkbox_vars[name].get()
        self.set_checkbox_status(name, value)

        # แสดงข้อความทดสอบ
        messagebox.showinfo("Checkbox Toggled", f"{name}: {'Enabled' if value else 'Disabled'}")
        

        # เพิ่มส่วนนี้
        if name == "Auto mode" and value == 0:
            self.call_func_by_name("Connect")

