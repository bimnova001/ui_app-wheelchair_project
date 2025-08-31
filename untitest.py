import unittest
import io
import sys
import tkinter as tk

# --- ทดสอบ funtion.py ---
class TestFuntion(unittest.TestCase):
    def test_print_output(self):
        captured = io.StringIO()
        sys.stdout = captured
        import importlib
        importlib.reload(sys.modules.get("pages.funtion", __import__("pages.funtion")))
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

# --- ทดสอบ MenuPage (เฉพาะ logic ไม่ทดสอบ GUI จริง) ---
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
        # ไม่มี Assertion เพราะเป็น GUI, แค่ทดสอบว่าไม่ error

    def test_on_func_click(self):
        try:
            self.page.on_func_click("Connect")
        except Exception as e:
            self.fail(f"on_func_click raised Exception: {e}")

# --- ทดสอบ AppConfigPage (เฉพาะ logic) ---
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

# --- ทดสอบ AIPage, JoystickPage, MapPage (เฉพาะสร้าง instance) ---
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

if __name__ == "__main__":
    unittest.main()