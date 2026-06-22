import glob
import os
import select
import time

import rclpy

from .terminal import RawTerminal
from .utils import _approach


class EvdevKeyboardInput:
    KEY_NAME_TO_CHAR = {
        "KEY_1": "1",
        "KEY_2": "2",
        "KEY_3": "3",
        "KEY_4": "4",
        "KEY_5": "5",
        "KEY_6": "6",
        "KEY_Q": "q",
        "KEY_W": "w",
        "KEY_E": "e",
        "KEY_R": "r",
        "KEY_T": "t",
        "KEY_Y": "y",
        "KEY_A": "a",
        "KEY_S": "s",
        "KEY_D": "d",
        "KEY_I": "i",
        "KEY_K": "k",
        "KEY_J": "j",
        "KEY_L": "l",
        "KEY_C": "c",
        "KEY_SPACE": " ",
        "KEY_ESC": "\x1b",
    }

    def __init__(self, logger, device_path=None):
        self.logger = logger
        self.device_path = device_path or os.environ.get("EASYARM_KEYBOARD_DEVICE")
        self.evdev = None
        self.device = None

    def __enter__(self):
        try:
            import evdev
        except ImportError as error:
            raise RuntimeError("python3-evdev is not installed. Install it with: sudo apt install python3-evdev") from error

        self.evdev = evdev
        self.device = self._open_keyboard_device()
        self.logger.info(f"Using keyboard event device: {self.device.path} ({self.device.name})")
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.device is not None:
            self.device.close()
            self.device = None

    def read_events(self):
        if self.device is None:
            return []
        if not select.select([self.device.fd], [], [], 0.0)[0]:
            return []

        events = []
        try:
            for event in self.device.read():
                if event.type != self.evdev.ecodes.EV_KEY:
                    continue
                key_event = self.evdev.categorize(event)
                if key_event.keystate == key_event.key_hold:
                    continue

                key_name = key_event.keycode
                if isinstance(key_name, list):
                    key_name = key_name[0]
                key = self.KEY_NAME_TO_CHAR.get(key_name)
                if key is None:
                    continue
                events.append((key, key_event.keystate == key_event.key_down))
        except BlockingIOError:
            return events
        return events

    def _open_keyboard_device(self):
        if self.device_path:
            try:
                return self.evdev.InputDevice(self.device_path)
            except PermissionError as error:
                raise RuntimeError(
                    f"Cannot read keyboard event device {self.device_path}. "
                    "Add user to input group or run with proper permissions."
                ) from error
            except OSError as error:
                raise RuntimeError(f"Cannot open keyboard event device {self.device_path}: {error}") from error

        candidates = []
        for path in sorted(glob.glob("/dev/input/event*")):
            try:
                device = self.evdev.InputDevice(path)
            except PermissionError:
                candidates.append((path, None, "permission"))
                continue
            capabilities = device.capabilities().get(self.evdev.ecodes.EV_KEY, [])
            if self.evdev.ecodes.KEY_1 in capabilities and self.evdev.ecodes.KEY_Q in capabilities:
                return device
            candidates.append((path, device.name, "not-keyboard"))
            device.close()

        permission_paths = [path for path, _, status in candidates if status == "permission"]
        if permission_paths:
            raise RuntimeError(
                "Cannot read keyboard event device. Add user to input group or run with proper permissions. "
                f"Permission denied devices: {', '.join(permission_paths)}"
            )
        raise RuntimeError(
            "No keyboard event device found. Set EASYARM_KEYBOARD_DEVICE=/dev/input/eventX if auto-detection fails."
        )


class SpeedJTeleopController:
    POSITIVE_KEYS = "123456"
    NEGATIVE_KEYS = "qwerty"

    def __init__(self, node):
        self.node = node
        self.rate_hz = 50.0
        self.dt = 1.0 / self.rate_hz
        self.max_speed = 30.0
        self.accel = 100.0
        self.decel = 120.0
        self.velocities = [0.0] * 6
        self.target_signs = [0] * 6

    def run(self) -> int:
        self.node.get_logger().info(
            "SpeedJ teleop mode: 1-6 positive, qwerty negative, Esc exits.")
        self.node.get_logger().info(
            f"Hold a key to ramp joint speed up to {self.max_speed:.1f} rad/s; release ramps quickly to zero.")

        try:
            with RawTerminal() as terminal, EvdevKeyboardInput(self.node.get_logger()) as keyboard:
                while rclpy.ok():
                    start = time.monotonic()
                    self._drain_terminal_input(terminal)
                    for key, is_pressed in keyboard.read_events():
                        if self._handle_key_event(key, is_pressed):
                            return 0
                    self._update_velocities(start)
                    self.node.speedj_pub.publish(self.node._make_joint_jog(self.velocities))
                    sleep_time = self.dt - (time.monotonic() - start)
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
        except RuntimeError as error:
            self.node.get_logger().error(str(error))
            return 1
        except KeyboardInterrupt:
            print()
        finally:
            self._halt()
            print()
        return 0

    def _drain_terminal_input(self, terminal) -> None:
        while terminal.read_key() is not None:
            pass

    def _handle_key_event(self, key, is_pressed: bool) -> bool:
        if key == "\x1b":
            return True
        if key in self.POSITIVE_KEYS:
            index = self.POSITIVE_KEYS.index(key)
            self.target_signs[index] = 1 if is_pressed else 0
        elif key in self.NEGATIVE_KEYS:
            index = self.NEGATIVE_KEYS.index(key)
            self.target_signs[index] = -1 if is_pressed else 0
        return False

    def _update_velocities(self, now: float) -> None:
        for index in range(6):
            if self.target_signs[index] != 0:
                target = self.target_signs[index] * self.max_speed
                step = self.accel * self.dt
            else:
                target = 0.0
                step = self.decel * self.dt
            self.velocities[index] = _approach(self.velocities[index], target, step)

    def _halt(self) -> None:
        interval = self.dt
        while any(abs(value) > 1e-3 for value in self.velocities):
            self.velocities = [_approach(value, 0.0, self.decel * interval) for value in self.velocities]
            self.node.speedj_pub.publish(self.node._make_joint_jog(self.velocities))
            time.sleep(interval)
        for _ in range(4):
            self.node.speedj_pub.publish(self.node._make_joint_jog([0.0] * 6))
            time.sleep(interval)


class SpeedLTeleopController:
    KEY_BINDINGS = {
        "w": (1, 1),       # y+
        "s": (1, -1),      # y-
        "a": (0, -1),      # x-
        "d": (0, 1),       # x+
        " ": (2, 1),       # z+
        "c": (2, -1),      # z-
        "q": (4, -1),      # -wy
        "e": (4, 1),       # +wy
        "i": (3, -1),      # +x clockwise pitch up.
        "k": (3, 1),       # +x counterclockwise pitch down.
        "j": (5, -1),      # +z clockwise yaw left.
        "l": (5, 1),       # +z counterclockwise yaw right.
    }

    def __init__(self, node):
        self.node = node
        self.rate_hz = 50.0
        self.dt = 1.0 / self.rate_hz
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.3
        self.linear_accel = 0.30
        self.linear_decel = 0.40
        self.angular_accel = 0.80
        self.angular_decel = 1.50
        self.twist = [0.0] * 6
        self.target_signs = [0] * 6

    def run(self) -> int:
        self.node.get_logger().info(
            "SpeedL teleop mode: wasd/space/c translate, q/e/ikjl rotate, Esc exits.")
        self.node.get_logger().info(
            "w y+, s y-, a x-, d x+, space z+, c z-.")
        self.node.get_logger().info(
            "q -wy, e +wy, i/k pitch around x, j/l yaw around z.")

        try:
            with RawTerminal() as terminal, EvdevKeyboardInput(self.node.get_logger()) as keyboard:
                while rclpy.ok():
                    start = time.monotonic()
                    self._drain_terminal_input(terminal)
                    for key, is_pressed in keyboard.read_events():
                        if self._handle_key_event(key, is_pressed):
                            return 0
                    self._update_twist(start)
                    self.node.speedl_pub.publish(self.node._make_twist(self.twist, "base_link"))
                    sleep_time = self.dt - (time.monotonic() - start)
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
        except RuntimeError as error:
            self.node.get_logger().error(str(error))
            return 1
        except KeyboardInterrupt:
            print()
        finally:
            self._halt()
            print()
        return 0

    def _drain_terminal_input(self, terminal) -> None:
        while terminal.read_key() is not None:
            pass

    def _handle_key_event(self, key, is_pressed: bool) -> bool:
        if key == "\x1b":
            return True
        binding = self.KEY_BINDINGS.get(key)
        if binding is None:
            return False
        index, sign = binding
        self.target_signs[index] = sign if is_pressed else 0
        return False

    def _update_twist(self, now: float) -> None:
        for index in range(6):
            if index < 3:
                max_speed = self.max_linear_speed
                accel = self.linear_accel
                decel = self.linear_decel
            else:
                max_speed = self.max_angular_speed
                accel = self.angular_accel
                decel = self.angular_decel

            if self.target_signs[index] != 0:
                target = self.target_signs[index] * max_speed
                step = accel * self.dt
            else:
                target = 0.0
                step = decel * self.dt
            self.twist[index] = _approach(self.twist[index], target, step)

    def _halt(self) -> None:
        interval = self.dt
        while any(abs(value) > 1e-4 for value in self.twist):
            next_twist = []
            for index, value in enumerate(self.twist):
                decel = self.linear_decel if index < 3 else self.angular_decel
                next_twist.append(_approach(value, 0.0, decel * interval))
            self.twist = next_twist
            self.node.speedl_pub.publish(self.node._make_twist(self.twist, "base_link"))
            time.sleep(interval)
        for _ in range(4):
            self.node.speedl_pub.publish(self.node._make_twist([0.0] * 6, "base_link"))
            time.sleep(interval)
