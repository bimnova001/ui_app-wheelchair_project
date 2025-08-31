import customtkinter as ctk
from tkinter import messagebox

class AppConfigPage(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(self, text="App Configuration", font=ctk.CTkFont(size=20, weight="bold")).grid(row=0, column=0, pady=30, sticky="n")

        # ตัวอย่าง: ปุ่มเปลี่ยน Theme
        ctk.CTkButton(self, text="Load Red Theme", command=lambda: self.load_theme("themes/red.json")).grid(row=1, column=0, pady=10)
        ctk.CTkButton(self, text="Load Blue Theme", command=lambda: self.load_theme("themes/blue.json")).grid(row=2, column=0, pady=10)
        ctk.CTkButton(self, text="⬅ Back to Menu", command=lambda: controller.show_frame("MenuPage")).grid(row=3, column=0, pady=30)

    def load_theme(self, theme_path):
        try:
            ctk.set_default_color_theme(theme_path)
            messagebox.showinfo("Theme", f"Theme loaded: {theme_path}")
            # อาจจะต้องรีเฟรชหน้าต่างหรือแจ้งเตือนผู้ใช้ให้ restart app
        except Exception as e:
            messagebox.showerror("Theme", f"Error loading theme:\n{e}")