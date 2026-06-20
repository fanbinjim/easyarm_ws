import atexit
import os
import readline
import shlex
import subprocess
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions

from .client import EasyArmCli
from .commands import run_command
from .parser import build_parser


def configure_readline_history(node: EasyArmCli) -> None:
    executable_dir = os.path.dirname(os.path.realpath(sys.argv[0]))
    history_path = os.path.join(executable_dir, ".easyarm_shell_history")
    try:
        if os.path.exists(history_path):
            readline.read_history_file(history_path)
        readline.set_history_length(1000)
        atexit.register(readline.write_history_file, history_path)
    except OSError as exception:
        node.get_logger().warning(f"Command history disabled: {exception}")


def run_safe_shutdown_command(node: EasyArmCli, extra_args) -> int:
    command = ["ros2", "run", "easyarm_a1_bringup", "safe_shutdown.sh", *extra_args]
    node.get_logger().info("Running safe shutdown")
    node.get_logger().info("cmd: " + " ".join(shlex.quote(value) for value in command))
    try:
        return subprocess.call(command)
    except FileNotFoundError:
        node.get_logger().error("ros2 command not found")
        return 1


def shell_main() -> None:
    parser = build_parser()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = EasyArmCli()
    configure_readline_history(node)
    node.get_logger().info("EasyArm shell ready. Type 'help' for commands, 'exit' to quit.")
    try:
        while rclpy.ok():
            try:
                line = input("easyarm> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                continue

            if not line:
                continue
            if line in ("exit", "quit"):
                break
            if line in ("help", "?"):
                parser.print_help()
                continue
            if line == "speedj_teleop":
                node.run_speedj_teleop()
                continue
            if line == "speedl_teleop":
                node.run_speedl_teleop()
                continue
            if line in ("safe_shutdown", "ss") or line.startswith("safe_shutdown ") or line.startswith("ss "):
                try:
                    extra_args = shlex.split(line)[1:]
                except ValueError as exception:
                    node.get_logger().error(str(exception))
                    continue

                return_code = run_safe_shutdown_command(node, extra_args)
                if return_code == 0:
                    node.get_logger().info("Safe shutdown completed. Exiting shell.")
                    break
                node.get_logger().error(f"Safe shutdown failed with exit code {return_code}")
                continue

            try:
                args = parser.parse_args(shlex.split(line))
            except SystemExit:
                continue

            try:
                run_command(node, args)
            except KeyboardInterrupt:
                print()
                if rclpy.ok():
                    node.get_logger().warning("Command interrupted")
            except Exception as exception:  # noqa: BLE001 - keep shell alive after command errors.
                node.get_logger().error(str(exception))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
