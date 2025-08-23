import customtkinter as ctk

class AIPage(ctk.CTkFrame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(self, text="AI Mode (ยังไม่ทำ)", font=ctk.CTkFont(size=18)).grid(row=0, column=0, pady=30, sticky="n")
        ctk.CTkButton(self, text="⬅ Back to Menu", command=lambda: controller.show_frame("MenuPage")).grid(row=1, column=0, pady=20, sticky="n")
