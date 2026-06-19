# EasyArm 控制系统技术路线

## Summary

本文档用于记录 EasyArm 后续控制系统技术路线，目标是把规划式运动、实时伺服、拖拽、动力学补偿和硬件适配的职责拆清楚。

核心结论：

- `DRAG`、`MOVE`、`SERVO` 应作为平级 robot control mode。
- `MoveJ/MoveL` 属于 `MOVE`，输入是单点或低频目标，继续适合使用 JTC。
- `SpeedJ/SpeedL` 属于 `SERVO`，输入是高频连续速度命令，不应继续沿 JTC 路线打磨。
- `ServoJ/ServoL` 尚未实现，后续也应归入 `SERVO`，走实时链路。
- `easyarm_hardware` 长期应变轻，只保留硬件适配、CAN 协议、方向/offset/限幅、安全保护、状态反馈和硬件模式管理。
- 重力补偿和更高级的动力学控制长期应迁移到自定义 `ros2_control` controller，由 controller 调用 `easyarm_dynamics`。

## Current State

当前已经实现：

- `MoveJ/MoveL`
  - `easyarm_app` 调用 `easyarm_motion_server`。
  - `easyarm_motion_server` 调用 MoveIt/Pilz。
  - MoveIt 输出轨迹给 `arm_controller`。
  - `arm_controller` 是 `joint_trajectory_controller/JointTrajectoryController`。

- `SpeedJ/SpeedL`
  - `easyarm_app` 发布 `/easyarm/speedj_cmd` 和 `/easyarm/speedl_cmd`。
  - `easyarm_motion_server` 接收 EasyArm 自己的 speed topic。
  - motion server 负责切换 `arm_controller` / `servo_position_controller`。
  - motion server 负责启动 MoveIt Servo，并转发到 `/servo_node/delta_joint_cmds` 和 `/servo_node/delta_twist_cmds`。
  - MoveIt Servo 当前输出 `std_msgs/Float64MultiArray` 到 `/servo_position_controller/commands`。
  - `servo_position_controller` 是 `position_controllers/JointGroupPositionController`。
  - `publish_period` 当前配置为 `0.005s`，目标是 200Hz position-only 流式输出。
  - 该链路已在虚拟/仿真环境验证正常，尚未做真机测试。
  - 当前仍未输出 velocity / effort，后续如果要给电机前馈速度和动力学 effort，需要迁移到 `MultiInterfaceForwardCommandController` 或自定义 controller。

- `ServoJ/ServoL`
  - 尚未实现。
  - 后续不建议继续使用 JTC 单点短轨迹方式实现。

- `DRAG`
  - 已在 `easyarm_hardware` 中实现。
  - 当前逻辑是 `kp=0`、`kd=drag_kd`、`velocity=0`、`torque=gravity(q) * drag_gravity_scale`。
  - 该功能已经真机可用，短期不迁移。

- Gravity compensation
  - 当前 `easyarm_hardware` 直接依赖 `easyarm_dynamics`。
  - `easyarm_hardware::write()` 中调用 `RobotModel::gravity(q)`。
  - 重力补偿结果会混入电机 `torque` 字段。
  - 这让 hardware 层承担了控制算法职责，长期不够清晰。

## Control Mode Model

后续建议把上层 robot control mode 表达为：

```text
IDLE
  无运动输入，硬件保持安全阻尼或停止状态。

DRAG
  无上层运动输入。
  输入来源是人手外力。
  控制目标是重力补偿 + 阻尼，让机械臂可被安全拖动。

MOVE
  单点或低频目标输入。
  典型接口：MoveJ / MoveL。
  系统规划或生成完整轨迹，再交给轨迹控制器执行。

SERVO
  高频连续输入。
  典型接口：SpeedJ / SpeedL / ServoJ / ServoL。
  输入停止后应快速 stop 或 hold。
```

当前 `POSITION` 模式混合了硬件电机控制语义和上层任务模式语义。后续架构中应逐步区分：

```text
hardware / motor mode:
  motion_control
  position_csp
  idle / disabled

robot control mode:
  IDLE
  DRAG
  MOVE
  SERVO
```

## Controller Architecture Direction

规划式运动和实时运动使用不同控制器：

```text
MOVE:
  MoveIt / Pilz
    -> arm_controller (JTC)
    -> easyarm_hardware

SERVO:
  MoveIt Servo / easyarm_motion_server
    -> servo_position_controller / servo_forward_controller / EasyArmServoController
    -> easyarm_hardware
```

`arm_controller` 继续使用 `joint_trajectory_controller/JointTrajectoryController`，只服务：

- `MoveJ/MoveL`
- Pilz/MoveIt 规划轨迹
- 低频规划式运动
- 安全位运动和关机流程中需要的轨迹执行

`SERVO` 链路转向 forward controller 或自定义 streaming controller：

- 当前第一阶段已经改为 `position_controllers/JointGroupPositionController`。
- MoveIt Servo 输出已经改为 `std_msgs/Float64MultiArray`。
- 输出 topic 当前指向 `/servo_position_controller/commands`。
- 目标是支持 200Hz 级别的高刷新率实时控制链路。
- 后续继续评估 `forward_command_controller/MultiInterfaceForwardCommandController`，用于同时输出 `position + velocity`。

控制器候选优先级：

1. `forward_command_controller/MultiInterfaceForwardCommandController`
   - 目标是同时向 hardware command interfaces 写入 `position + velocity`。
   - 最符合当前电机控制希望同时收到位置和速度命令的需求。
   - 需要验证 Humble 版本的参数格式、命令数组 layout，以及 6 个关节双接口映射是否稳定。

2. `position_controllers/JointGroupPositionController`
   - 只转发 position command。
   - 当前第一阶段已经采用该方案。
   - 对 `SpeedJ/SpeedL` 来说，需要依赖 MoveIt Servo 内部积分后的 joint position 输出。

3. `velocity_controllers/JointGroupVelocityController`
   - 只转发 velocity command。
   - 语义最接近 `SpeedJ/SpeedL`。
   - 需要确认 `easyarm_hardware` 的 velocity command 在真实硬件当前模式下是否安全、有效。

如果现有 forward controllers 无法满足 `position + velocity` 同时下发，则后续考虑新增 EasyArm 自定义 streaming controller。

## Motion Server Direction

`easyarm_motion_server` 后续应成为 EasyArm 运动能力的统一服务层。

长期目标：

- `easyarm_app` 不直接调用 MoveIt Servo 原生接口。
- `easyarm_app` 不直接依赖：
  - `/servo_node/start_servo`
  - `/servo_node/delta_joint_cmds`
  - `/servo_node/delta_twist_cmds`
- `easyarm_app` 只调用 EasyArm 自己暴露的接口。
- `easyarm_motion_server` 负责：
  - MOVE / SERVO / DRAG 模式协调。
  - controller 切换。
  - MoveIt Servo 启停。
  - stop / hold / timeout 处理。
  - 避免 JTC 旧目标残留导致回跳。

当前 `SpeedJ/SpeedL` 已经迁移为：

```text
easyarm_app
  -> easyarm_motion_server
    -> MoveIt Servo
    -> servo controller
    -> easyarm_hardware
```

## Dynamics / Gravity Compensation Direction

当前 `easyarm_hardware` 内部集成了重力补偿，短期可以保留，因为：

- `DRAG` 已经依赖该能力。
- 真机拖拽模式已经可用。
- 贸然迁移会影响安全和手感。

但长期更合理的边界是：

```text
controller
  -> 读取 joint state
  -> 接收目标 position / velocity
  -> 调用 easyarm_dynamics
  -> 输出 position / velocity / effort command
  -> easyarm_hardware
```

`easyarm_hardware` 长期只负责：

- command interfaces 到电机协议的转换。
- motor direction / offset / protocol range clamp。
- CAN 发送和反馈解析。
- state interfaces 发布。
- 电机 enable / disable / mode switch。
- 最后一层安全限幅和故障保护。

需要注意的是，重力补偿从 hardware 迁出不能一步完成。只要 `MoveJ/MoveL` 仍然使用 JTC，JTC 本身不会替 EasyArm 计算 `gravity(q)`，因此 hardware 内部 gravity compensation 仍然承担着 `MOVE` 链路的重力前馈职责。

当前 `SpeedJ/SpeedL` 第一阶段使用 `JointGroupPositionController`，也只输出 position，不输出 effort。因此这一阶段仍可以让 hardware 内部 gravity compensation 继续工作。真正需要关闭或旁路 hardware 内部 gravity compensation 的时刻，是后续自定义 controller 或 multi-interface controller 已经开始输出 `effort` / 动力学前馈时。

不建议为了这个过渡在 hardware 内新增 `SERVO` mode。`SERVO` 是上层 robot control mode，不是底层 hardware mode。更合理的底层抽象是：

```text
effort / feedforward source:
  internal_gravity
  controller_effort
  none
```

因此过渡期 hardware 可能会短暂变重：它需要兼容旧的 `MoveJ/MoveL/DRAG` 链路，也要允许新的 SERVO controller 逐步接管实时控制。这个复杂度应被明确标记为临时兼容层，最终目标仍然是让 hardware 变轻。

控制算法应逐步迁到 controller 层：

- gravity compensation
- velocity feedforward
- acceleration feedforward
- inverse dynamics
- friction compensation
- impedance / damping control
- streaming command smoothing

建议新增控制包，例如：

```text
easyarm_control
  EasyArmServoController
  EasyArmDragController     # 后续评估，不急
  EasyArmTrajectoryController # 后续评估，不急
```

优先级：

1. `EasyArmServoController`
   - 服务 `SpeedJ/SpeedL/ServoJ/ServoL`。
   - 高频流式控制。
   - 内部调用 `easyarm_dynamics`。
   - 写入 `position + velocity + effort` command interfaces。

2. `EasyArmDragController`
   - 只有在 `EasyArmServoController` 稳定后再评估。
   - 迁移前保留 hardware 内现有 `DRAG`。

3. `EasyArmTrajectoryController`
   - 只有当 JTC 无法满足 `MoveJ/MoveL` 需求时再考虑。
   - 目前 `MoveJ/MoveL` 可以继续使用 JTC。

## Reference Notes / 控制概念参考

本节记录架构讨论中的概念边界，用作后续设计参考，不代表当前都已经实现。

### Mode / Strategy / Skill 分层

后续讨论中建议区分三层：

```text
Robot Control Mode                  # 机器人控制模式：决定当前谁在控制机械臂、输入频率是什么
  IDLE                              # 空闲/阻尼/停止状态
  DRAG                              # 拖拽模式：人手施加外力，系统做重力补偿和阻尼
  MOVE                              # 规划运动模式：接收单点或低频目标，执行完整轨迹
  SERVO                             # 实时伺服模式：接收高频连续输入，实时跟随

Control Strategy                    # 控制策略：某个模式内部使用的控制算法
  position control                  # 位置控制
  velocity control                  # 速度控制
  gravity compensation              # 重力补偿
  impedance control                 # 阻抗控制：位置误差到力/力矩
  admittance control                # 导纳控制：外力到位置/速度
  hybrid force-position control     # 混合力位控制：部分方向控位置，部分方向控力
  collision detection / protection  # 碰撞检测/保护

Task Skill / Interface              # 任务技能或对外接口：用户真正调用的动作能力
  MoveJ                             # 关节空间规划运动
  MoveL                             # 笛卡尔空间直线规划运动
  SpeedJ                            # 关节空间速度伺服
  SpeedL                            # 笛卡尔空间速度伺服
  ServoJ                            # 关节空间位置伺服，尚未实现
  ServoL                            # 笛卡尔空间位置伺服，尚未实现
  drag teaching                     # 拖拽示教
  constant-force press              # 恒力按压
  surface following                 # 沿面跟随
  grinding / polishing              # 打磨/抛光
  insertion                         # 插孔/装配
```

`DRAG`、`MOVE`、`SERVO` 是控制模式；阻抗、导纳、重力补偿、混合力位控制是控制策略；`MoveJ`、
`SpeedJ`、恒力按压、打磨、插孔等是接口或任务技能。

### 输入频率分类

`DRAG`、`MOVE`、`SERVO` 的主要差异可以从输入频率理解：

- `DRAG`
  - 无上层运动输入。
  - 输入来源是人手外力。
  - 当前实现是重力补偿 + 阻尼。
  - 人手拖到哪里，机械臂停在哪里。

- `MOVE`
  - 单点或低频目标输入。
  - 典型接口是 `MoveJ/MoveL`。
  - 特点是基础、确定、结果明确，适合 JTC。
  - 适合非接触运动、回零、预设位、大范围移动。

- `SERVO`
  - 高频连续输入。
  - 典型接口是 `SpeedJ/SpeedL/ServoJ/ServoL`。
  - 更适合遥操、视觉伺服、接触任务、力反馈和在线微调。
  - 输入停止后应快速 stop 或 hold。

`MOVE` 和 `SERVO` 不是替代关系，而是配合关系：`MOVE` 负责大范围、确定地到附近，`SERVO`
负责最后一段、实时、柔顺、精细地操作。

### 阻抗控制

阻抗控制可以理解为“位置误差到力/力矩”：

```text
tau = K(q_des - q)
    + D(qd_des - qd)
    + tau_ff
```

- `K` 是刚度，类似 `kp`。
- `D` 是阻尼，类似 `kd`。
- `tau_ff` 是力矩前馈。

只把 `kp` 调低只能让位置控制“软一点”，但不是完整阻抗控制。完整阻抗还需要阻尼、重力补偿和必要的力矩前馈。

当前电机 `MotionControl` 模型接近：

```text
tau_motor = kp * (pos_des - pos)
          + kd * (vel_des - vel)
          + torque
```

如果 `torque` 中放入 `gravity(q)`，就接近关节空间阻抗中的重力补偿版本。

### 力矩前馈

力矩前馈是“不等误差出现，先根据模型或任务需求把预计需要的力矩加进去”。

常见前馈项：

```text
tau_ff = gravity(q)
       + C(q, qd)qd
       + M(q)qdd_des
       + friction
```

- `gravity(q)` 是重力前馈，当前系统已经使用了这一类。
- `C(q, qd)qd` 是速度相关的科氏/离心项，低速时可先忽略，高速轨迹时有帮助。
- `M(q)qdd_des` 是惯量/加速度前馈，用于提前提供加速所需力矩。
- `friction` 是摩擦前馈，有助于低速爬行和换向。

如果想让末端在接触面上施加一个力，需要先把末端 wrench 转成关节力矩：

```text
tau_force = J(q)^T * wrench
```

其中：

```text
wrench = [Fx, Fy, Fz, Tx, Ty, Tz]
```

注意：如果没有接触面，力矩前馈会让机械臂沿该方向运动；如果没有力传感器闭环，实际接触力不一定等于给定前馈力。

### 导纳控制

导纳控制可以理解为“外力到位置/速度”：

```text
M * xdd + D * xd + K * (x - x_ref) = F_ext
```

外力撤掉后是否回去，取决于虚拟弹簧 `K` 和参考点策略：

- `K > 0` 且 `x_ref` 固定：松手后会回到参考位置。
- `K = 0`：只有质量/阻尼，松手后会停在新位置，更像拖拽示教。
- `K > 0` 但 `x_ref` 跟随更新：可能只回一部分，或停在更新后的参考点。

当前 `DRAG` 更接近 `K = 0` 的重力补偿 + 阻尼拖拽。

### 只用关节力矩做导纳

只用电机反馈的关节力矩可以先做关节空间导纳，但精度和稳定性不如末端六维力传感器。

第一版可以估计外力矩：

```text
tau_ext = tau_measured - tau_model
```

低速时可先近似：

```text
tau_ext ~= tau_measured - gravity(q)
```

再做关节空间导纳：

```text
M_a * qdd_cmd + D_a * qd_cmd + K_a * (q_cmd - q_ref) = tau_ext
```

更保守的第一版可以做纯阻尼导纳：

```text
qd_cmd = clamp(tau_ext / D_a, -qd_max, qd_max)
q_cmd += qd_cmd * dt
```

这适合拖拽示教、关节柔顺和人手推动机械臂。后续如果要做末端导纳，可以通过
`tau_ext = J(q)^T * wrench_ext` 反推末端 wrench，但会受到奇异点、模型误差、摩擦和力矩噪声影响。

### 后续能力层级

阻抗/导纳不是终点，它们是机械臂开始安全接触世界的基础能力。后续可逐步发展：

- 外力估计和碰撞检测
  - 用 `tau_ext = tau_measured - tau_model` 估计外部力矩。
  - 用于碰撞检测、人碰到就停、异常接触报警和拖拽意图识别。

- 关节空间导纳
  - 优先基于现有电机 torque feedback 和 `easyarm_dynamics::gravity(q)`。
  - 更适合第一版拖拽/柔顺能力。

- 笛卡尔阻抗 / 导纳
  - 让末端表现成一个虚拟弹簧阻尼系统。
  - 可以让某些方向硬、某些方向软，更符合接触任务直觉。

- 混合力位控制
  - 例如 x/y 控位置，z 控压力。
  - 适合擦拭、打磨、沿面跟随和插孔装配。

- 接触任务技能
  - 恒力按压。
  - 沿面跟随。
  - 打磨 / 抛光。
  - 插孔 / 装配。
  - 接触后自动切换模式。

## Staged Roadmap

### Stage 1: 现状稳定和文档化

- 保留现有 `MoveJ/MoveL`、`SpeedJ/SpeedL`、`DRAG` 行为。
- 记录当前架构限制。
- 不修改真实硬件默认行为。
- 不修改 motor ID、direction、offset、joint limit、CAN 参数、control gain 或 `use_mock_hardware` 默认值。

### Stage 2: MoveIt Servo + JGPC Position-only 验证

- 已在 MoveIt config 中新增 `servo_position_controller`。
- `servo_position_controller` 启动时保持 inactive。
- 默认仍保持 `arm_controller` active。
- MoveIt Servo 输出 `std_msgs/Float64MultiArray` 到 `/servo_position_controller/commands`。
- `publish_period` 配置为 `0.005s`。
- 当前已在虚拟/仿真环境验证 `SpeedJ/SpeedL` 正常，尚未做真机测试。

### Stage 3: easyarm_motion_server 封装 SpeedJ/SpeedL

- 已完成第一版。
- `easyarm_motion_server` 增加 `MoveItServoExecutor`。
- 进入 SERVO 前停止 MoveIt 执行，并切换 `arm_controller` / `servo_position_controller`。
- 启动 MoveIt Servo。
- `/easyarm/stop` 同时停止 MoveIt Servo 输出并发送 zero command。
- speed command timeout 后发送 zero command 并切回 `arm_controller`。
- `easyarm_app` 改为发布 `/easyarm/speedj_cmd` 和 `/easyarm/speedl_cmd`。

后续需要继续打磨：

- 真机低速验证。
- 失败恢复和状态上报。
- controller 切换耗时和边界条件。
- `SERVO` 与 hardware `POSITION/IDLE/DRAG` 历史模式的命名解耦。

### Stage 4: 引入 EasyArmServoController

- 新增 `easyarm_control` 或类似控制包。
- 实现 `EasyArmServoController`。
- controller 内调用 `easyarm_dynamics`。
- 输出 `position + velocity + effort`。
- 逐步从普通 forward controller 迁移到 EasyArm 自定义 streaming controller。

### Stage 5: 评估 DRAG 和 MOVE 是否迁移

- `DRAG` 当前已经可用，短期不迁移。
- 等 `EasyArmServoController` 稳定后，再评估是否新增 `EasyArmDragController`。
- `MoveJ/MoveL` 继续使用 JTC，除非后续证明 JTC 无法满足轨迹执行和动力学补偿需求。

## Test Strategy

Mock 阶段：

```bash
colcon build --packages-select easyarm_a1_moveit_config easyarm_a1_bringup easyarm_motion_server easyarm_app
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true moveit_servo:=true
ros2 control list_controllers
ros2 topic info /easyarm/speedj_cmd -v
ros2 topic info /easyarm/speedl_cmd -v
ros2 topic hz /servo_position_controller/commands
```

功能回归：

- `MoveJ/MoveL` 在 JTC 模式下仍可正常规划和执行。
- `SpeedJ/SpeedL` 在 forward controller 模式下运动连续，无明显 JTC 单点轨迹颗粒感。
- controller 切换时不会出现旧 JTC 目标残留导致的回跳。
- `/easyarm/stop` 可以停止 MoveIt Servo 输出，并让机械臂进入安全 hold。
- `safe_shutdown.sh` 不受影响，仍可回到安全位并 deactivate hardware。
- `DRAG` 模式保持现有手感和安全行为。
- gravity compensation 拆分前后不能出现双重补偿。

真实硬件阶段：

- 先只测 mock，再测真实硬件。
- 真实硬件先测小速度、小幅度。
- 每次测试前确认：
  - 当前 active controller。
  - 当前 robot control mode。
  - 当前 hardware mode。
  - `/joint_states` 新鲜度。
  - `safe_shutdown.sh` 可用。
- 如果引入 controller 层 gravity compensation，必须先关闭 hardware 内同类补偿，避免 torque 叠加。

## Assumptions

- 本计划记录后续架构方向，不代表当前已全部实现。
- `ServoJ/ServoL` 尚未实现。
- `DRAG` 已实现且短期保留在 `easyarm_hardware`。
- `MoveJ/MoveL` 长期优先继续使用 JTC。
- `SpeedJ/SpeedL/ServoJ/ServoL` 走实时链路，不再继续沿 JTC 路线打磨。
- Forward controller 方案先在 mock 中验证，再进入真实硬件。
- 自定义 controller 是中长期方向，不作为第一步直接替换现有硬件行为。
