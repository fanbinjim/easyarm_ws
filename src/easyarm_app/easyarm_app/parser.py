import argparse


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="EasyArm app CLI",
        epilog=(
            "examples:\n"
            "  movej 0.0025 0.25 2 0.1 -1.57 0.0\n"
            "  speedj_teleop    # in easyarm_shell: keyboard JointJog mode\n"
            "  movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0\n"
            "  speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50\n"
            "  speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50\n"
            "  set-mode DRAG\n"
            "  set-mode POSITION\n"
            "  speedl_teleop    # in easyarm_shell: keyboard Cartesian teleop mode\n"
            "  ss               # in easyarm_shell: safe_shutdown alias"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    subparsers = parser.add_subparsers(dest="command", required=True)

    movej = subparsers.add_parser(
        "movej",
        help="Call /easyarm/movej",
        epilog=(
            "example:\n"
            "  movej 0.0025 0.25 2 0.1 -1.57 0.0"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    movej.add_argument("joints", nargs=6, type=float, metavar="J")
    movej.add_argument("--velocity-scale", type=float, default=0.2)
    movej.add_argument("--acceleration-scale", type=float, default=0.2)
    movej.add_argument("--plan-only", dest="execute", action="store_false")
    movej.set_defaults(execute=True)

    movel = subparsers.add_parser(
        "movel",
        help="Call /easyarm/movel",
        epilog="example:\n  movel 0.25 0.0 0.25 0.0 0.0 0.0 1.0",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    movel.add_argument("x", type=float)
    movel.add_argument("y", type=float)
    movel.add_argument("z", type=float)
    movel.add_argument("qx", type=float)
    movel.add_argument("qy", type=float)
    movel.add_argument("qz", type=float)
    movel.add_argument("qw", type=float)
    movel.add_argument("--frame-id", default="base_link")
    movel.add_argument("--velocity-scale", type=float, default=0.1)
    movel.add_argument("--acceleration-scale", type=float, default=0.1)
    movel.add_argument("--plan-only", dest="execute", action="store_false")
    movel.set_defaults(execute=True)

    set_mode = subparsers.add_parser(
        "set-mode",
        help="Call /easyarm/set_mode",
        epilog="examples:\n  set-mode DRAG\n  set-mode POSITION",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    set_mode.add_argument("mode", choices=["POSITION", "IDLE", "DRAG", "position", "idle", "drag"])

    subparsers.add_parser("stop", help="Call /easyarm/stop")
    subparsers.add_parser("get-state", help="Call /easyarm/get_state")
    subparsers.add_parser("get-joints", help="Call /easyarm/get_joints")

    get_pose = subparsers.add_parser("get-pose", help="Call /easyarm/get_pose")
    get_pose.add_argument("--target-frame", default="base_link")
    get_pose.add_argument("--source-frame", default="Link6")

    speedj = subparsers.add_parser(
        "speedj",
        help="Publish MoveIt Servo JointJog commands",
        epilog="example:\n  speedj 0 0.05 0 0 0 0 --duration 1.0 --rate 50",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    speedj.add_argument("velocities", nargs=6, type=float, metavar="V")
    speedj.add_argument("--duration", type=float, default=1.0)
    speedj.add_argument("--rate", type=float, default=50.0)
    speedj.add_argument("--halt-count", type=int, default=4)

    speedl = subparsers.add_parser(
        "speedl",
        help="Publish MoveIt Servo TwistStamped commands",
        epilog="example:\n  speedl 0.01 0 0 0 0 0 --duration 1.0 --rate 50",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    speedl.add_argument("vx", type=float)
    speedl.add_argument("vy", type=float)
    speedl.add_argument("vz", type=float)
    speedl.add_argument("wx", type=float)
    speedl.add_argument("wy", type=float)
    speedl.add_argument("wz", type=float)
    speedl.add_argument("--frame-id", default="base_link")
    speedl.add_argument("--duration", type=float, default=1.0)
    speedl.add_argument("--rate", type=float, default=50.0)
    speedl.add_argument("--halt-count", type=int, default=4)

    subparsers.add_parser(
        "speedj_teleop",
        help="Run keyboard SpeedJ teleoperation in easyarm_shell",
    )

    subparsers.add_parser(
        "speedl_teleop",
        help="Run keyboard SpeedL teleoperation in easyarm_shell",
    )

    safe_shutdown = subparsers.add_parser("safe_shutdown", help="Run safe shutdown and exit shell")
    safe_shutdown.add_argument("args", nargs=argparse.REMAINDER, help="Arguments passed to safe_shutdown.sh")

    ss = subparsers.add_parser("ss", help="Alias for safe_shutdown in easyarm_shell")
    ss.add_argument("args", nargs=argparse.REMAINDER, help="Arguments passed to safe_shutdown.sh")

    return parser
