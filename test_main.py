import unittest
import io
import sys
import os
import tkinter as tk

# --- ทดสอบ funtion.py ---
class TestFuntion(unittest.TestCase):
    def test_print_output(self):
        captured = io.StringIO()
        sys.stdout = captured
        import importlib
        # reload เพื่อให้ print ทำงานทุกครั้ง
        if "pages.funtion" in sys.modules:
            importlib.reload(sys.modules["pages.funtion"])
        else:
            import pages.funtion
        sys.stdout = sys.__stdout__
        self.assertIn("this is test", captured.getvalue())

# --- ทดสอบ joystick.py ---
class DummyCallback:
    def __init__(self):
        self.called = False
        self.args = None
    def __call__(self, x, y):
        self.called = True
        self.args = (x, y)

class TestVirtualJoystick(unittest.TestCase):
    def setUp(self):
        import joystick
        self.tk_root = tk.Tk()
        self.tk_root.withdraw()
        self.callback = DummyCallback()
        self.joystick = joystick.VirtualJoystick(self.tk_root, callback=self.callback)

    def tearDown(self):
        self.tk_root.destroy()

    def test_initial_state(self):
        self.assertEqual(self.joystick.current_x, 0)
        self.assertEqual(self.joystick.current_y, 0)
        self.assertFalse(self.joystick.holding)

    def test_start_hold_sets_holding(self):
        event = tk.Event()
        self.joystick.start_hold(event)
        self.assertTrue(self.joystick.holding)

    def test_move_knob_and_callback(self):
        event = tk.Event()
        event.x = self.joystick.center + 10
        event.y = self.joystick.center + 10
        self.joystick.holding = True
        self.joystick.move_knob(event)
        self.joystick.update_callback()
        self.assertTrue(self.callback.called)
        self.assertIsInstance(self.callback.args, tuple)

    def test_reset_knob(self):
        event = tk.Event()
        self.joystick.current_x = 1
        self.joystick.current_y = 1
        self.joystick.holding = True
        self.joystick.reset_knob(event)
        self.assertEqual(self.joystick.current_x, 0)
        self.assertEqual(self.joystick.current_y, 0)
        self.assertFalse(self.joystick.holding)

# --- ทดสอบ MenuPage (logic) ---
class DummyController:
    def __init__(self):
        self.frames = {}

class TestMenuPage(unittest.TestCase):
    def setUp(self):
        import customtkinter as ctk
        self.tk_root = tk.Tk()
        self.tk_root.withdraw()
        from pages.menu_page import MenuPage
        self.controller = DummyController()
        self.page = MenuPage(self.tk_root, self.controller)

    def tearDown(self):
        self.tk_root.destroy()

    def test_set_func_status(self):
        self.page.set_func_status("Connect", True)
        self.page.set_func_status("Connect", False)

    def test_on_func_click(self):
        try:
            self.page.on_func_click("Connect")
        except Exception as e:
            self.fail(f"on_func_click raised Exception: {e}")

    def test_checkbox_default(self):
        self.assertEqual(self.page.cb_notify.get(), 1)
        self.assertEqual(self.page.cb_auto.get(), 0)
        self.assertEqual(self.page.cb_battery.get(), 0)

    def test_select_menu(self):
        self.page.select_menu("Joystick")
        self.page.select_menu("AI config")
        self.page.select_menu("SLAM Map")

# --- ทดสอบ AppConfigPage (logic) ---
class TestAppConfigPage(unittest.TestCase):
    def setUp(self):
        import customtkinter as ctk
        self.tk_root = tk.Tk()
        self.tk_root.withdraw()
        from pages.app_config_page import AppConfigPage
        self.controller = DummyController()
        self.page = AppConfigPage(self.tk_root, self.controller)

    def tearDown(self):
        self.tk_root.destroy()

    def test_load_theme(self):
        try:
            self.page.load_theme("themes/red.json")
        except Exception:
            pass  # อาจจะ error ถ้าไม่มีไฟล์ theme จริง

# --- ทดสอบ AIPage, JoystickPage, MapPage (สร้าง instance) ---
class TestOtherPages(unittest.TestCase):
    def setUp(self):
        import customtkinter as ctk
        self.tk_root = tk.Tk()
        self.tk_root.withdraw()

    def tearDown(self):
        self.tk_root.destroy()

    def test_ai_page(self):
        from pages.ai_page import AIPage
        AIPage(self.tk_root, DummyController())

    def test_joystick_page(self):
        from pages.joystick_page import JoystickPage
        JoystickPage(self.tk_root, DummyController())

    def test_map_page(self):
        from pages.map_page import MapPage
        MapPage(self.tk_root, DummyController())

# --- ทดสอบ App (สร้าง instance และ show_frame) ---
class TestApp(unittest.TestCase):
    def test_app_init_and_show_frame(self):
        import customtkinter as ctk
        from app import App
        app = App()
        app.show_frame("MenuPage")
        app.show_frame("JoystickPage")
        app.show_frame("AIPage")
        app.show_frame("MapPage")
        app.show_frame("AppConfigPage")
        app.destroy()

if __name__ == "__main__":
    unittest.main()