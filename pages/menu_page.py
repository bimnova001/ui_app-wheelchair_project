import subprocess, threading
import sys
import customtkinter as ctk
from tkinter import messagebox
from joystick import VirtualJoystick
#from pages.status_node import StatusData

class MenuPage(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.startup_ran = False
        self.log_textbox = None
        self.battery_percent = 0
        
        

        # กำหนด function/checkbox ที่ต้องการจัดการและแสดง status
        self.func_list = [
            {"name": "Check ros2", "cmd": ["python", "--version"]},
            {"name": "Save Map","cmd": ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "/home/user/assets/map"]}
        ]
        self.checkbox_list = [
            {"name": "Enable Ai object detection"},
            {"name": "Manual Control"},
            
            ]
        
        self.func_list_start = [
            {"name": "Start robot", "cmd": ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"]},
            {"name": "Start SLAM Mapping", "cmd": ["ros2", "launch", "slam_toolbox", "online_async_launch.py", "use_sim_time:=true"]},
            {"name": "Start Nav2", "cmd": ["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=True"]},
            
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
            var = ctk.IntVar()#value=1 if cb["name"] == "Enable Ai object detection" else 0)
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
            main, values=["Camara Ai", "Menu", "SLAM Map"],
            command=self.select_menu
        )
        self.menu.grid(row=1, column=0, pady=(0, 20), ipadx=10, ipady=10, sticky="ew")
        self.menu.set("Menu")  # ตั้งค่าเริ่มต้น

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
        # ให้คอลัมน์ซ้ายกว้างกว่า ขวา
        log_frame.grid_columnconfigure(0, weight=3)
        log_frame.grid_columnconfigure(1, weight=1)
        log_frame.grid_rowconfigure(2, weight=0)

        ctk.CTkLabel(
            log_frame, text="Log / Information",
            font=ctk.CTkFont(size=14, weight="bold")
        ).grid(row=0, column=0, columnspan=2, pady=(10, 0), padx=10, sticky="w")

        # left: log textbox
        left_box = ctk.CTkFrame(log_frame, fg_color="transparent")
        left_box.grid(row=1, column=0, sticky="nsew", padx=(10, 6), pady=10)
        left_box.grid_rowconfigure(0, weight=1)
        left_box.grid_columnconfigure(0, weight=1)

        self.log_textbox = ctk.CTkTextbox(left_box)
        self.log_textbox.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        self.log_textbox.insert("end", "Welcome :D\n")
        self.log_textbox.configure(state="disabled")

        # right: joystick container (มี title + canvas + status)
        right_box = ctk.CTkFrame(log_frame, width=260, fg_color="#2a2a2a", corner_radius=8)
        right_box.grid(row=1, column=1, sticky="nsew", padx=(6, 10), pady=10)
        right_box.grid_rowconfigure(0, weight=0)
        right_box.grid_rowconfigure(1, weight=1)
        right_box.grid_rowconfigure(2, weight=0)
        right_box.grid_columnconfigure(0, weight=1)
        
        ctk.CTkLabel(right_box, text="Manual Joystick", font=ctk.CTkFont(size=13, weight="bold")).grid(row=0, column=0, pady=(8, 4))
        # สร้าง VirtualJoystick (จาก joystick.py) ให้เหมาะกับธีม
        self.joystick_widget = VirtualJoystick(
            right_box, size=240, knob_size=56, callback=self.update_joystick_status,
            bg="#2a2a2a", ring_fill="#333333", ring_outline="#6f6f6f", knob_color="#2ca3ff"
        )
        # ใช้ grid ของ tkinter canvas ได้ตรง ๆ
        self.joystick_widget.grid(row=1, column=0, padx=10, pady=6, sticky="n")

        self.joystick_status = ctk.CTkLabel(right_box, text="x=0.00, y=0.00", font=ctk.CTkFont(size=12))
        self.joystick_status.grid(row=2, column=0, pady=(6, 10))

        # เริ่มด้วยซ่อน joystick หาก Manual Control ปิด
        if not self.checkbox_vars.get("Manual Control") or self.checkbox_vars["Manual Control"].get() == 0:
            right_box.grid_remove()
        self._right_box = right_box  # เก็บ ref เพื่อแสดง/ซ่อน

        

        self.bind("<Visibility>", lambda e: self.update_points_list())
        
        status_panel = ctk.CTkFrame(self, width=180, fg_color="#232323")
        status_panel.grid(row=0, column=2, sticky="nsew", padx=(10, 10), pady=10)
        status_panel.grid_propagate(False)

        ctk.CTkLabel(status_panel, text="Status", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(20, 10))
    
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
            
            self.after_idle(self.auto_run_commands)
            self.after_idle(self.start_yolo_if_enabled)
            self.after(1000, self.update_points_list)  # อัพเดตทุกวินาที
            self.after_idle( self.navigate_to_named_goal)
            
            
        
    def update_joystick_status(self, x, y):
        self.joystick_status.configure(text=f"x={x:.2f}, y={y:.2f}")
        
    def burger_camara(self):
        threading.Thread(
            target=lambda: subprocess.Popen([sys.executable, "-u", "pages/burger_camara.py"]),
            daemon=True
        ).start()
        
    def burger_detect(self):
        threading.Thread(
            target=lambda: subprocess.Popen([sys.executable, "-u", "pages/burger_detect.py"]),
            daemon=True
        ).start()
        
        
    def navigate_to_named_goal(self):
        threading.Thread(
            target=lambda: subprocess.Popen([sys.executable, "-u", "pages/navigate_to_named_goal.py"]),
            daemon=True
        ).start()

    def auto_run_commands(self):
        
        
        if self.startup_ran:
            return
        self.startup_ran = True  # กันไม่ให้รันซ้ำ

        for func in self.func_list_start:
            self.on_func_click(func)
            
    def log(self, text):
        self.log_textbox.configure(state="normal")
        self.log_textbox.insert("end", text)
        self.log_textbox.see("end")
        self.log_textbox.configure(state="disabled")
        
    def start_yolo_if_enabled(self):
        if self.checkbox_vars["Enable Ai object detection"].get() == 1:
            cam_page = self.controller.frames.get("CameraPage")
            if cam_page:
                cam_page.yolo.start()


    def update_points_list(self):
        points_frame = getattr(self, "points_frame", None)
        if not points_frame:
            return
        # ลบ widget เก่า
        for w in points_frame.winfo_children():
            w.destroy()
        # ดึง MapPage
        map_page = getattr(self.controller, "frames", {}).get("MapPage")
        points = map_page.get_points_list() if map_page else []
        if not points:
            ctk.CTkLabel(points_frame, text="ไม่มีจุดที่เพิ่ม", text_color="#888").pack(anchor="w")
        else:
            for idx, p in enumerate(points):
                x, y = p['map_x'], p['map_y']
                btn = ctk.CTkButton(
                    points_frame,
                    text=f"{p.get('name', f'Point {idx+1}')} ({x:.2f},{y:.2f})",
                    fg_color="#b71c1c",
                    command=lambda px=x, py=y: self.send_goal_from_menu(px, py)
                )
                btn.pack(anchor="w", pady=2, fill="x")

    def send_goal_from_menu(self, x, y):
        # ส่ง ROS2 goal ผ่าน MapPage node
        map_page = getattr(self.controller, "frames", {}).get("MapPage")
        if map_page:
            threading.Thread(target=lambda: map_page.ros2_node.send_goal(x, y), daemon=True).start()
            self.log(f"Goal sent: x={x:.2f}, y={y:.2f}\n")

    def show_point(self, x, y):
        messagebox.showinfo("Point Selected", f"คุณเลือกตำแหน่ง\nx={x}, y={y}")


    def select_menu(self, value):
        if value == "Camara Ai":
            self.controller.show_frame("CameraPage")
        elif value == "SLAM Map":
            try :
                self.controller.show_frame("MapPage")
            except Exception as e:
                self.log(f"Error opening MapPage: {e}\n")
            
        self.menu.set("Menu")

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
        def run():
            try:
                self.log(f"Running: {func['name']}\n")
                proc = subprocess.Popen(func["cmd"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                out, err = proc.communicate()
                if proc.returncode == 0:
                    if func["name"] in self.status_funcs:
                        self.set_func_status(func["name"], True)
                    elif func["name"] in self.status_start_funcs:
                        self.set_start_func_status(func["name"], True)
                    self.log(f"{func['name']} success:\n{out}\n")
                    # messagebox.showinfo(func["name"], f"{func['name']} success:\n{out}")
                else:
                    if func["name"] in self.status_funcs:
                        self.set_func_status(func["name"], False)
                    elif func["name"] in self.status_start_funcs:
                        self.set_start_func_status(func["name"], False)
                    self.log(f"{func['name']} error:\n{err}\n")
                    # messagebox.showerror(func["name"], f"{func['name']} error:\n{err}")
            except Exception as e:
                if func["name"] in self.status_funcs:
                    self.set_func_status(func["name"], False)
                elif func["name"] in self.status_start_funcs:
                    self.set_start_func_status(func["name"], False)
                self.log(f"{func['name']} exception:\n{e}\n")
                # messagebox.showerror(func["name"], f"{func['name']} error!\n{e}")
                    
        threading.Thread(target=run, daemon=True).start()


    def on_checkbox_toggle(self, name):
        # อัปเดตสถานะ checkbox
        value = self.checkbox_vars[name].get()
        self.set_checkbox_status(name, value)

        # แสดงข้อความทดสอบ
        messagebox.showinfo("Checkbox Toggled", f"{name}: {'Enabled' if value else 'Disabled'}")
        

        # เพิ่มส่วนนี้
        if name == "Enable Ai object detection":
            cam_page = self.controller.frames.get("CameraPage")
            if cam_page:
                if value:
                    cam_page.yolo.start()
                else:
                    cam_page.yolo.stop()
        elif name == "Manual Control":
            # แสดง/ซ่อน joystick container (right_box)
            if value:
                try:
                    self._right_box.grid()
                except Exception:
                    pass
            else:
                try:
                    self._right_box.grid_remove()
                except Exception:
                    pass


    # ------------ Joystick callback handler ----------
    def update_joystick_status(self, x, y):
        # แสดงบน label
        try:
            self.joystick_status.configure(text=f"x={x:.2f}, y={y:.2f}")
        except Exception:
            pass
                        
        
