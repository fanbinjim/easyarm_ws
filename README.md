## TODO

- 将当前 `easyarm_hardware` 内的 `ControlMode` 控制逻辑上提到独立 `easyarm_controller`。
- 为 `IDLE`、`POSITION`、`DRAG` 设计正式 controller/service/action 接口。
- 将模式切换从 hardware service 迁移到 controller 层，避免长期在 `SystemInterface` 内承载控制策略。
- 增加拖拽示教轨迹录制节点，订阅 `/joint_states` 保存关节轨迹。
- 增加拖拽轨迹回放节点，将记录结果发送到 `FollowJointTrajectory`。
- 将 gravity scale、drag kd、torque limit 等调试参数改为运行时可调。
- 增加 mock hardware 下的模式切换和重力补偿测试。
