import customtkinter as ctk
from pages.menu_page import MenuPage
from pages.joystick_page import JoystickPage
from pages.ai_page import AIPage
#from pages.map_page import MapPage
from pages.app_config_page import AppConfigPage
from pages.yolo import CameraPage
from pages.yolo_bg import YoloBackground

ctk.set_appearance_mode("System")
ctk.set_default_color_theme("themes/red.json")

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Robot Control System")

        # ทำให้ window ขยายได้
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.iconbitmap("logo/cisat.ico")
        
        

        # container สำหรับหน้าแต่ละหน้า
        self.container = ctk.CTkFrame(self)
        self.container.grid(row=0, column=0, sticky="nsew")
        self.container.grid_rowconfigure(0, weight=1)         # เพิ่มบรรทัดนี้
        self.container.grid_columnconfigure(0, weight=1)      # เพิ่มบรรทัดนี้

        self.frames = {}
        
        self.yolo_bg = YoloBackground(log_func=self.log)
        
        #MapPage ยังไม่ทำ
        for F in (MenuPage, JoystickPage, AppConfigPage , CameraPage):
            if F == CameraPage:
                frame = F(parent=self.container, controller=self, yolo_bg=self.yolo_bg)
            else:
                frame = F(parent=self.container, controller=self)
            self.frames[F.__name__] = frame
            frame.place(relwidth=1, relheight=1)

        self.show_frame("MenuPage")

    def show_frame(self, page_name):
        frame = self.frames[page_name]
        frame.tkraise()
    def log(self, text):
        print(text)