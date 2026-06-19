import sys

import rclpy

from .client import EasyArmCli
from .commands import run_command
from .parser import build_parser
from .shell import shell_main


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    rclpy.init()
    node = EasyArmCli()
    try:
        return run_command(node, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def console_main():
    sys.exit(main())


__all__ = [
    "EasyArmCli",
    "build_parser",
    "console_main",
    "main",
    "run_command",
    "shell_main",
]
