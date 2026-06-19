import time

import rclpy

from .terminal import RawTerminal
from .utils import _approach


class SpeedJTeleopController:
    POSITIVE_KEYS = "123456"
    NEGATIVE_KEYS = "qwerty"

    def __init__(self, node):
        self.node = node
        self.rate_hz = 50.0
        self.dt = 1.0 / self.rate_hz
        self.max_speed = 10.0
        self.accel = 10.0
        self.decel = 20.0
        self.key_timeout = 0.12
        self.velocities = [0.0] * 6
        self.active_until = [0.0] * 6
        self.target_signs = [0] * 6

    def run(self) -> int:
        self.node.get_logger().info(
            "SpeedJ teleop mode: 1-6 positive, qwerty negative, Esc exits.")
        self.node.get_logger().info(
            f"Hold a key to ramp joint speed up to {self.max_speed:.1f} rad/s; release ramps quickly to zero.")

        try:
            with RawTerminal() as terminal:
                while rclpy.ok():
                    start = time.monotonic()
                    if self._handle_key(terminal.read_key(), start):
                        break
                    self._update_velocities(start)
                    self.node.speedj_pub.publish(self.node._make_joint_jog(self.velocities))
                    sleep_time = self.dt - (time.monotonic() - start)
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
        except KeyboardInterrupt:
            print()
        finally:
            self._halt()
            print()
        return 0

    def _handle_key(self, key, now: float) -> bool:
        if key is None:
            return False
        if key == "\x1b":
            return True
        if key in self.POSITIVE_KEYS:
            index = self.POSITIVE_KEYS.index(key)
            self.target_signs[index] = 1
            self.active_until[index] = now + self.key_timeout
        elif key in self.NEGATIVE_KEYS:
            index = self.NEGATIVE_KEYS.index(key)
            self.target_signs[index] = -1
            self.active_until[index] = now + self.key_timeout
        return False

    def _update_velocities(self, now: float) -> None:
        for index in range(6):
            if now <= self.active_until[index]:
                target = self.target_signs[index] * self.max_speed
                step = self.accel * self.dt
            else:
                target = 0.0
                self.target_signs[index] = 0
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
        self.key_timeout = 0.12
        self.twist = [0.0] * 6
        self.active_until = [0.0] * 6
        self.target_signs = [0] * 6

    def run(self) -> int:
        self.node.get_logger().info(
            "SpeedL teleop mode: wasd/space/c translate, q/e/ikjl rotate, Esc exits.")
        self.node.get_logger().info(
            "w y+, s y-, a x-, d x+, space z+, c z-.")
        self.node.get_logger().info(
            "q -wy, e +wy, i/k pitch around x, j/l yaw around z.")

        try:
            with RawTerminal() as terminal:
                while rclpy.ok():
                    start = time.monotonic()
                    if self._handle_key(terminal.read_key(), start):
                        break
                    self._update_twist(start)
                    self.node.speedl_pub.publish(self.node._make_twist(self.twist, "base_link"))
                    sleep_time = self.dt - (time.monotonic() - start)
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
        except KeyboardInterrupt:
            print()
        finally:
            self._halt()
            print()
        return 0

    def _handle_key(self, key, now: float) -> bool:
        if key is None:
            return False
        if key in ("\x1b", "\x03"):
            return True
        binding = self.KEY_BINDINGS.get(key)
        if binding is None:
            return False
        index, sign = binding
        self.target_signs[index] = sign
        self.active_until[index] = now + self.key_timeout
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

            if now <= self.active_until[index]:
                target = self.target_signs[index] * max_speed
                step = accel * self.dt
            else:
                target = 0.0
                self.target_signs[index] = 0
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
