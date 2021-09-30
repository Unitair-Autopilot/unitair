"""
Definition of the Joystick class.

"""
# Math
from __future__ import division
import numpy as np

# Asynchronous buffer management
from threading import Thread
from collections import deque

# IO
import inputs  # sudo apt install python-pip && pip install --user inputs  # and sometimes a reboot is needed
from unitair.msg import Joystick as JoystickMsg
try: from importlib import reload  # needed for Python3
except ImportError: pass

##################################################

class Joystick(object):
    """
    User-interface for reading from a gamepad-style joystick.
    Call start_joystick_thread to begin filling an internal buffer with user input.
    Call get_state to execute / clear the buffer and get the current state.
    Call stop_joystick_thread when done!

    stick_deadband:   fraction of analog joystick travel that should be treated as zero
    trigger_deadband: fraction of analog trigger travel that should be treated as zero
    max_buffer_size:  maximum number of user inputs that should be stored before dropping old ones

    """
    def __init__(self, stick_deadband=0, trigger_deadband=0, max_buffer_size=20):
        self.max_buffer_size = int(max_buffer_size)

        # Valid input device names in priority order
        self.valid_device_names = ["Microsoft X-Box One S pad",
                                   "Microsoft X-Box One pad (Firmware 2015)",
                                   "Microsoft X-Box 360 pad",
                                   "PowerA Xbox One wired controller",
                                   "Logitech Gamepad F310",
                                   "Logitech Gamepad F710"]

        # Set valid input device
        reload(inputs)
        self.input_device = None
        for valid_device_name in self.valid_device_names:
            if self.input_device is not None: break
            for device in inputs.devices:
                if device.name == valid_device_name:
                    self.input_device = device.name
                    break
        if self.input_device is None:
            raise IOError("JOYSTICK FATAL: '{0}' is not a valid input device!".format(device.name))

        # Analog input characteristics
        self.max_stick = 32767
        if "Logitech Gamepad" in self.input_device: self.max_trigger = 255
        else: self.max_trigger = 1023
        self.set_stick_deadband(stick_deadband)
        self.set_trigger_deadband(trigger_deadband)

        # Internals
        self.state = None
        self.joystick_thread = None
        self.stay_alive = False
        self.buffer = deque([])
        self.buffer_size_flag = False

    #########################

    def set_stick_deadband(self, val):
        self.stick_deadband = float(val)
        self.min_stick = int(self.stick_deadband * self.max_stick)

    def set_trigger_deadband(self, val):
        self.trigger_deadband = float(val)
        self.min_trigger = int(self.trigger_deadband * self.max_trigger)

    #########################

    def get_state(self):
        """
        Executes / clears the input buffer and returns the latest state.

        """
        if self.joystick_thread is None: raise AssertionError("JOYSTICK FATAL: Cannot get_state without active joystick thread!")
        while self.buffer:
            event = self.buffer.pop()
            # Right-hand analog inputs
            if   event.code == "ABS_RY": self.state.right_stick_north = -self._stick_frac(event.state)
            elif event.code == "ABS_RX": self.state.right_stick_east = self._stick_frac(event.state)
            elif event.code == "ABS_RZ": self.state.right_trigger = self._trigger_frac(event.state)
            # Left-hand analog inputs
            elif event.code == "ABS_Y": self.state.left_stick_north = -self._stick_frac(event.state)
            elif event.code == "ABS_X": self.state.left_stick_east = self._stick_frac(event.state)
            elif event.code == "ABS_Z": self.state.left_trigger = self._trigger_frac(event.state)
            # Bumpers
            elif event.code == "BTN_TR": self.state.right_bumper = bool(event.state)
            elif event.code == "BTN_TL": self.state.left_bumper = bool(event.state)
            # Letter-buttons
            elif event.code == "BTN_NORTH": self.state.button_west = bool(event.state)  # swapped with north for some reason
            elif event.code == "BTN_SOUTH": self.state.button_south = bool(event.state)
            elif event.code == "BTN_EAST": self.state.button_east = bool(event.state)
            elif event.code == "BTN_WEST": self.state.button_north = bool(event.state)  # swapped with west for some reason
            # Toggles
            elif event.code == "BTN_START" and event.state: self.state.switch_east = not self.state.switch_east
            elif event.code == "BTN_SELECT" and event.state: self.state.switch_west = not self.state.switch_west
            # Direction-pad
            elif event.code == "ABS_HAT0Y":
                self.state.dpad_north = bool(event.state < 0)
                self.state.dpad_south = bool(event.state > 0)
            elif event.code == "ABS_HAT0X":
                self.state.dpad_east = bool(event.state > 0)
                self.state.dpad_west = bool(event.state < 0)
        return self.state

    #########################

    def start_joystick_thread(self):
        """
        Starts a thread that reads user input into the internal buffer.

        """
        if self.stay_alive:
            return
        self.state = JoystickMsg(switch_east=False, switch_west=False)
        self.stay_alive = True
        self.joystick_thread = Thread(target=self._listen)
        self.joystick_thread.daemon = True
        self.joystick_thread.start()

    def stop_joystick_thread(self):
        """
        Terminates the joystick's user input reading thread and clears the buffer.

        """
        self.stay_alive = False
        if self.joystick_thread is not None:
            self.joystick_thread.join()
            self.joystick_thread = None
        while self.buffer:
            self.buffer.pop()
        self.buffer_size_flag = False
        self.state = None

    #########################

    def _listen(self):
        try:
            while self.stay_alive:
                self.buffer.appendleft(inputs.get_gamepad()[0])  # this is blocking (hence need for threading)
                if len(self.buffer) > self.max_buffer_size:
                    if not self.buffer_size_flag:
                        self.buffer_size_flag = True
                    self.buffer.pop()
        except IOError:
            pass
        finally:
            self.joystick_thread = None

    #########################

    def _stick_frac(self, val):
        if abs(val) > self.min_stick:
            return np.clip(np.divide(val, self.max_stick, dtype=float), -1, 1)
        return float(0)

    def _trigger_frac(self, val):
        if abs(val) > self.min_trigger:
            return np.clip(np.divide(val, self.max_trigger, dtype=float), -1, 1)
        return float(0)

##################################################
