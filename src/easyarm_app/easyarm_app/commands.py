def run_command(node, args) -> int:
    if args.command == "safe_shutdown":
        node.get_logger().error("safe_shutdown is only supported by easyarm_shell")
        return 1
    if args.command == "movej":
        return node.movej(args)
    if args.command == "movel":
        return node.movel(args)
    if args.command == "set-mode":
        return node.set_mode(args)
    if args.command == "stop":
        return node.stop(args)
    if args.command == "get-state":
        return node.get_state(args)
    if args.command == "get-joints":
        return node.get_joints(args)
    if args.command == "get-pose":
        return node.get_pose(args)
    if args.command == "speedj":
        return node.speedj(args)
    if args.command == "speedl":
        return node.speedl(args)
    if args.command == "speedj_teleop":
        node.get_logger().error("speedj_teleop is only supported by easyarm_shell")
        return 1
    if args.command == "speedl_teleop":
        node.get_logger().error("speedl_teleop is only supported by easyarm_shell")
        return 1
    raise RuntimeError(f"unknown command {args.command}")
