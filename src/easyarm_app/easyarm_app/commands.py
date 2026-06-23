def run_command(node, args) -> int:
    if args.command in ("safe_shutdown", "ss"):
        node.get_logger().error(f"{args.command} is only supported by easyarm_shell")
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
    if args.command == "servoj":
        return node.servoj(args)
    if args.command == "servol":
        return node.servol(args)
    if args.command == "speedj_teleop":
        node.get_logger().error("speedj_teleop is only supported by easyarm_shell")
        return 1
    if args.command == "speedl_teleop":
        node.get_logger().error("speedl_teleop is only supported by easyarm_shell")
        return 1
    if args.command == "servoj_teleop":
        node.get_logger().error("servoj_teleop is only supported by easyarm_shell")
        return 1
    if args.command == "servol_teleop":
        node.get_logger().error("servol_teleop is only supported by easyarm_shell")
        return 1
    raise RuntimeError(f"unknown command {args.command}")
