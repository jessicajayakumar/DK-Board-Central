from dkb_fb_gui import Toplevel1, DKB
import tkinter as tk

# Mock DKB class for testing
class MockDKB:
    def __init__(self):
        self.ser = "MockSerial"

    def write_to_serial(self, ser, command):
        print(f"Writing to {ser}: {command}")

# Mock Tk class for testing
class MockTk:
    def __init__(self):
        self.tk = self  # Mimic the tk attribute
        self._last_child_ids = None  # Add _last_child_ids attribute

    def geometry(self, geometry_str):
        print(f"Setting geometry: {geometry_str}")

    def minsize(self, width, height):
        print(f"Setting minsize: {width}x{height}")

    def maxsize(self, width, height):
        print(f"Setting maxsize: {width}x{height}")

    def resizable(self, width, height):
        print(f"Setting resizable: {width}, {height}")

    def title(self, title_str):
        print(f"Setting title: {title_str}")

# Extend Toplevel1 to use MockDKB and MockTk
class TestToplevel1(Toplevel1):
    def __init__(self, top=None):
        if top is None:
            top = MockTk()
        super().__init__(top)
        self.dkb_instance = MockDKB()

# Create an instance of TestToplevel1
test_instance = TestToplevel1()

# Set recipient and test voltage_send_on
test_instance.recipient = 'broadcast'
test_instance.voltage_send_on()

test_instance.recipient = 'FB1'
test_instance.voltage_send_on()

test_instance.recipient = 'FB2'
test_instance.voltage_send_on()

test_instance.recipient = 'unknown'
test_instance.voltage_send_on()

# Set recipient and test voltage_send_off
test_instance.recipient = 'broadcast'
test_instance.voltage_send_off()

test_instance.recipient = 'FB1'
test_instance.voltage_send_off()

test_instance.recipient = 'FB2'
test_instance.voltage_send_off()

test_instance.recipient = 'unknown'
test_instance.voltage_send_off()