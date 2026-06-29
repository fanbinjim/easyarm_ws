# EasyArm 控制系统技术路线

## Summary

本文档用于记录 EasyArm 后续控制系统技术路线，目标是把规划式运动、实时伺服、拖拽、动力学补偿和硬件适配的职责拆清楚。

核心结论：

- `FREE_DRIVE`、`MOVE`、`SERVO` 应作为平级 robot control mode；旧 hardware `DRAG` 模式已经删除。
- `MoveJ/MoveL` 属于 `MOVE`，输入是单点或低频目标，继续适合使用 JTC。
- `SpeedJ/SpeedL` 属于 `SERVO`，输入是高频连续速度命令，不应继续沿 JTC 路线打磨。
- `ServoJ/ServoL` 已实现第一版位置伺服，归入 `SERVO`，通过 motion server 转换为 MoveIt Servo 速度输入。
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
  - motion server 负责切换 `arm_controller` / `easyarm_servo_controller`。
  - motion server 负责启动 MoveIt Servo，并转发到 `/servo_node/delta_joint_cmds` 和 `/servo_node/delta_twist_cmds`。
  - MoveIt Servo 当前输出 `trajectory_msgs/JointTrajectory` 到 `/easyarm_servo_controller/joint_trajectory`。
  - `easyarm_servo_controller` 是 `easyarm_controller/EasyArmServoController`。
  - `publish_period` 当前配置为 `0.005s`，目标是 200Hz 流式输出。
  - MoveIt Servo 在 Humble 的 `Float64MultiArray` 输出模式要求 position 或 velocity 二选一；因此默认链路改用 `JointTrajectory`，同时输出 position / velocity / acceleration。
  - `easyarm_servo_controller` 当前把 `position + velocity + kp + kd + effort` 写给 hardware；`JointTrajectory` 输入带 velocity 时会继续传给 hardware，controller effort 目前只包含 gravity feedforward。
  - `easyarm_servo_controller` 同时保留 `/easyarm_servo_controller/joint_positions` 作为 `Float64MultiArray` position-only 兼容输入。
  - `EasyArmServoController` 激活时用当前 joint state 初始化 hold position；输入 timeout 后保持上一条有效 position，不清零。
  - 该链路已完成真机测试，`SpeedJ/SpeedL/ServoJ/ServoL` 基本可用；ServoL 在奇异点附近仍会受 MoveIt Servo 降速保护影响。
  - 当前 velocity command 已在 `JointTrajectory` 输入带 velocity 时传递到 hardware；effort 先实现为 `gravity(q_target)`，后续可继续扩展为完整动力学前馈。

- `ServoJ/ServoL`
  - 已实现第一版。
  - `ServoJ` 接收关节位置目标，经 `PositionServoExecutor` 转换为 `JointJog`。
  - `ServoL` 接收末端位姿目标，经 `PositionServoExecutor` 转换为 `TwistStamped`。
  - 二者继续复用 MoveIt Servo 的碰撞、奇异点、joint limit 缩放和 controller 切换链路。
  - `ServoL` 当前不保证严格直线；严格直线仍使用 `MoveL`。

- `FREE_DRIVE`
  - 已新增 `easyarm_controller/EasyArmFreedriveController`，并通过 `/easyarm/set_mode FREE_DRIVE` 进入。
  - 第一阶段逻辑对齐旧 hardware `DRAG`：`kp=0`、`kd=drag_kd`、`velocity=0`、`torque=gravity(q) * drag_gravity_scale`。
  - 旧 hardware `DRAG` 模式已经删除，不再作为 motion server 对外接口。

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

FREE_DRIVE
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
  FREE_DRIVE
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
    -> EasyArmServoController / servo_forward_controller
    -> easyarm_hardware
```

`arm_controller` 继续使用 `joint_trajectory_controller/JointTrajectoryController`，只服务：

- `MoveJ/MoveL`
- Pilz/MoveIt 规划轨迹
- 低频规划式运动
- 安全位运动和关机流程中需要的轨迹执行

`SERVO` 链路转向 forward controller 或自定义 streaming controller：

- 当前第一阶段已经改为自定义 `easyarm_controller/EasyArmServoController`。
- MoveIt Servo 输出已经改为 `trajectory_msgs/JointTrajectory`。
- 输出 topic 当前指向 `/easyarm_servo_controller/joint_trajectory`。
- 目标是支持 200Hz 级别的高刷新率实时控制链路。
- 后续继续评估 `forward_command_controller/MultiInterfaceForwardCommandController` 或继续扩展自定义 controller，用于同时输出完整关节运控 command。

控制器候选优先级：

1. `forward_command_controller/MultiInterfaceForwardCommandController`
   - 目标是同时向 hardware command interfaces 写入 `position + velocity`。
   - 最符合当前电机控制希望同时收到位置和速度命令的需求。
   - 如果后续不用自定义 controller，需要验证 Humble 版本的参数格式、命令数组 layout，以及 6 个关节双接口映射是否稳定。

2. `easyarm_controller/EasyArmServoController`
   - 已经实现 `position + velocity + kp + kd + effort` 完整 command，effort 内容当前是 gravity feedforward。
   - 当前已经采用该方案作为 SERVO 主线。
   - controller 同时支持 `/easyarm_servo_controller/joint_positions` 的 `Float64MultiArray` position-only 输入和 `/easyarm_servo_controller/joint_trajectory` 的 `JointTrajectory` 输入。
   - `JointTrajectory` 输入中的 velocity 已传给 hardware，acceleration 已解析和缓存，后续可用于加速度前馈和完整动力学。
   - 无效输入只 warn 并保持上一条有效 command，timeout 后保持 hold position。

3. `velocity_controllers/JointGroupVelocityController`
   - 只转发 velocity command。
   - 语义最接近 `SpeedJ/SpeedL`。
   - 需要确认 `easyarm_hardware` 的 velocity command 在真实硬件当前模式下是否安全、有效。

如果现有 forward controllers 无法满足 `position + velocity` 同时下发，则后续考虑新增 EasyArm 自定义 streaming controller。

## Startup Safety Risks

### 上电反馈同步竞态

当前 `easyarm_hardware::on_activate()` 中已有“读取当前电机位置并同步为目标命令”的逻辑：

```text
enable motors in damping mode
  -> read()
  -> sync_states_to_commands()
```

但该逻辑依赖 `read()` 当时已经从 `RobstrideCanDriver` 收到每个电机的有效反馈。如果启动时某些电机第一帧反馈尚未到达，`feedback.is_valid == false` 时 `hw_positions_` 会保留初始化值；当前 `initial_positions.yaml` 默认为全 0，因此 `sync_states_to_commands()` 可能把 0 同步成 position command。表现上可能像“上电后自动往 home/零位拉”。重新上电后问题可能消失，因为电机/CAN 反馈已经稳定，缓存更快变为有效。

该风险与前端无关；`web:=false` 启动时仍可能出现。它属于硬件 activation 时序和反馈有效性检查问题。

后续修复方向：

- 在 hardware activation 阶段等待所有关节 fresh feedback 后再执行 `sync_states_to_commands()`。
- 如果超时未收到完整反馈，保持阻尼/IDLE 或直接 activation 失败，禁止进入 POSITION hold。
- 增加启动反馈合理性检查：上电后如果读到的电机位置全部是绝对 0，应视为高风险无效读数。真实电机反馈通常会有微小抖动或非零小数，不应所有关节严格等于 0；该判断可作为 `feedback.is_valid` / fresh feedback 检查之外的额外保护。
- 在同步成功时打印各关节启动反馈位置和 command position，便于现场判断同步到的是当前位姿还是初始化值。
- 评估 bringup 默认是否应先加载但不激活 `arm_controller`，待 joint state 稳定后再显式进入 MOVE/POSITION。

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
  - MOVE / SERVO / FREE_DRIVE 模式协调。
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

- 旧 hardware `DRAG` 曾依赖该能力；现在拖拽已迁移到 `EasyArmFreedriveController`。
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

当前 `SpeedJ/SpeedL` 使用 `EasyArmServoController` 输出完整关节运控 command。SERVO 路径已经开始由 controller 提供 `kp/kd/effort`，其中 effort 当前为 gravity feedforward；`MoveJ/MoveL` 仍继续依赖 hardware 内部 gravity compensation；`FREE_DRIVE` 由 controller 提供 gravity feedforward。

不建议为了这个过渡在 hardware 内新增 `SERVO` mode。`SERVO` 是上层 robot control mode，不是底层 hardware mode。更合理的底层抽象是：

```text
effort / feedforward source:
  internal_gravity
  controller_effort
  none
```

当前实现中该 source 不作为 ROS 参数暴露，而是由 hardware 根据 `effort` command interface 是否被 controller claim 自动判断，避免人为误切导致双重补偿或缺失补偿。

因此过渡期 hardware 可能会短暂变重：它需要兼容旧的 `MoveJ/MoveL` 链路，也要允许新的 SERVO controller 逐步接管实时控制。这个复杂度应被明确标记为临时兼容层，最终目标仍然是让 hardware 变轻。

控制算法应逐步迁到 controller 层：

- gravity compensation
- velocity feedforward
- acceleration feedforward
- inverse dynamics
- friction compensation
- impedance / damping control
- streaming command smoothing

当前已经新增控制包：

```text
easyarm_controller
  EasyArmServoController
  EasyArmFreedriveController       # 已新增，FREE_DRIVE 入口使用
  EasyArmTrajectoryController # 后续评估
```

优先级：

1. `EasyArmServoController`
   - 已实现第一版。
   - 服务 `SpeedJ/SpeedL/ServoJ/ServoL`。
   - 高频流式控制。
   - 当前接收 `JointTrajectory`，解析 position / velocity / acceleration。
   - 当前已经写入 `position + velocity + kp + kd + effort` command interfaces。
   - `JointTrajectory` velocity 已经传递到 hardware；acceleration 先缓存，后续用于完整动力学 effort。
   - controller 已调用 `easyarm_dynamics` 计算 gravity feedforward，后续可扩展为速度/加速度/摩擦等完整前馈。

2. `EasyArmFreedriveController`
   - 对外入口统一为 `/easyarm/set_mode FREE_DRIVE`。
      - 第一阶段输出 `position=current`、`velocity=0`、`kp=0`、`kd=drag_kd`、`effort=gravity(q) * drag_gravity_scale`。
   - 旧 hardware `DRAG` 模式已经删除，后续继续验证 FREE_DRIVE 的长期稳定性。

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
  FREE_DRIVE                        # 拖拽模式：人手施加外力，系统做重力补偿和阻尼
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
  ServoJ                            # 关节空间位置伺服，已实现第一版
  ServoL                            # 笛卡尔空间位置伺服，已实现第一版
  drag teaching                     # 拖拽示教
  constant-force press              # 恒力按压
  surface following                 # 沿面跟随
  grinding / polishing              # 打磨/抛光
  insertion                         # 插孔/装配
```

`FREE_DRIVE`、`MOVE`、`SERVO` 是控制模式；阻抗、导纳、重力补偿、混合力位控制是控制策略；`MoveJ`、
`SpeedJ`、恒力按压、打磨、插孔等是接口或任务技能。

### 输入频率分类

`FREE_DRIVE`、`MOVE`、`SERVO` 的主要差异可以从输入频率理解：

- `FREE_DRIVE`
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

当前 `FREE_DRIVE` 更接近 `K = 0` 的重力补偿 + 阻尼拖拽。

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

- 保留现有 `MoveJ/MoveL`、`SpeedJ/SpeedL`、`FREE_DRIVE` 行为。
- 记录当前架构限制。
- 不修改真实硬件默认行为。
- 不修改 motor ID、direction、offset、joint limit、CAN 参数、control gain 或 `use_mock_hardware` 默认值。

### Stage 2: MoveIt Servo + EasyArmServoController 验证

- 已在 MoveIt config 中新增 `easyarm_servo_controller`。
- `easyarm_servo_controller` 启动时保持 inactive。
- 默认仍保持 `arm_controller` active。
- MoveIt Servo 输出 `trajectory_msgs/JointTrajectory` 到 `/easyarm_servo_controller/joint_trajectory`。
- `publish_period` 配置为 `0.005s`。
- `EasyArmServoController` 兼容 `/easyarm_servo_controller/joint_positions` 的 position-only `Float64MultiArray` 输入。
- `EasyArmServoController` 当前写入 `position + velocity + kp + kd + effort`，`JointTrajectory` 中的 velocity 已传给 hardware，acceleration 已解析并缓存。
- effort 内容当前是 gravity feedforward，后续扩展到速度前馈、加速度前馈和完整动力学。
- 当前已完成真机验证：`SpeedJ/SpeedL/ServoJ/ServoL` 基本可用。

### Stage 3: easyarm_motion_server 封装 SERVO 接口

- 已完成第一版。
- `easyarm_motion_server` 增加 `MoveItServoRuntime` 和 `PositionServoExecutor`。
- 进入 SERVO 前停止 MoveIt 执行，并切换 `arm_controller` / `easyarm_servo_controller`。
- 启动 MoveIt Servo。
- `/easyarm/stop` 同时停止 MoveIt Servo 输出并发送 zero command。
- speed command timeout 后发送 zero command 并切回 `arm_controller`。
- `easyarm_app` 改为发布 `/easyarm/speedj_cmd` 和 `/easyarm/speedl_cmd`。
- `ServoJ/ServoL` 通过 `PositionServoExecutor` 转换为 MoveIt Servo 的 `JointJog/TwistStamped` 输入。

后续需要继续打磨：

- 继续完善失败恢复和状态上报。
- controller 切换耗时和边界条件。
- hardware mode 已收敛到 `POSITION/IDLE`；对外拖拽入口统一为 `FREE_DRIVE`。

### Stage 4: 扩展 EasyArmServoController

- 在现有 `easyarm_controller/EasyArmServoController` 内继续扩展。
- controller 内调用 `easyarm_dynamics`。
- 从 gravity-only effort 扩展到使用 velocity / acceleration 前馈。
- 后续把 acceleration 用于加速度前馈，并把 effort 从 gravity-only 扩展为 full dynamics effort。
- 逐步把 gravity compensation、feedforward、impedance/admittance 从 hardware 迁移到 EasyArm 自定义 streaming controller。

### Stage 5: EasyArmFreedriveController / FREE_DRIVE

- 已新增 `EasyArmFreedriveController`，对外入口统一为 `FREE_DRIVE`；旧 hardware `DRAG` 模式已经删除。
- 第一阶段让 `easyarm_freedrive_controller` 默认 inactive，通过 `/easyarm/set_mode FREE_DRIVE` 由 motion server 切换进入。
- 目标行为对齐原 hardware DRAG 手感：`kp=0`、`kd=freedrive kd`、`velocity=0`、`effort=gravity(q) * gravity_scale`。
- 验证 `MOVE -> FREE_DRIVE`、`SERVO -> FREE_DRIVE`、`FREE_DRIVE -> MOVE/SERVO` 切换不会回跳、冲击或残留旧目标。
- 旧 hardware DRAG 已删除，后续继续验证 FREE_DRIVE 的长期稳定性。
- `MoveJ/MoveL` 继续使用 JTC，除非后续证明 JTC 无法满足轨迹执行和动力学补偿需求。

## Test Strategy

Mock 阶段：

```bash
colcon build --packages-select easyarm_a1_moveit_config easyarm_a1_bringup easyarm_motion_server easyarm_app
ros2 launch easyarm_a1_bringup bringup.launch.py use_mock_hardware:=true moveit_servo:=true
ros2 control list_controllers
ros2 topic info /easyarm/speedj_cmd -v
ros2 topic info /easyarm/speedl_cmd -v
ros2 topic hz /easyarm_servo_controller/joint_trajectory
```

功能回归：

- `MoveJ/MoveL` 在 JTC 模式下仍可正常规划和执行。
- `SpeedJ/SpeedL` 在 EasyArmServoController 模式下运动连续，无明显 JTC 单点轨迹颗粒感。
- controller 切换时不会出现旧 JTC 目标残留导致的回跳。
- `/easyarm/stop` 可以停止 MoveIt Servo 输出，并让机械臂进入安全 hold。
- `safe_shutdown.sh` 不受影响，仍可回到安全位并 deactivate hardware。
- `FREE_DRIVE` 模式保持拖拽手感和安全行为。
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
- `ServoJ/ServoL` 已实现第一版并完成真机测试。
- `FREE_DRIVE` 已接入 motion server；旧 hardware `DRAG` 已从 `easyarm_hardware` 删除。
- `MoveJ/MoveL` 长期优先继续使用 JTC。
- `SpeedJ/SpeedL/ServoJ/ServoL` 走实时链路，不再继续沿 JTC 路线打磨。
- 自定义 controller 已成为 SERVO 主线；FREE_DRIVE controller 已接入 motion server，后续继续验证 FREE_DRIVE 长期稳定性。
