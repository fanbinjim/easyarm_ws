# EasyArm Workspace 架构分析

## Summary

本文档记录当前 EasyArm workspace 的架构状态、合理性判断、主要架构债和后续收口建议。

结论：

- 当前 workspace 不需要推倒重来，整体方向是合理的。
- 当前处在“从测试工程向正式系统迁移”的过渡期。
- `MoveJ/MoveL` 主链路已经比较清晰。
- `SpeedJ/SpeedL` 仍是临时验证链路，后续应收口到 `easyarm_motion_server` 和 SERVO 专用 controller。
- `easyarm_hardware` 当前承担了部分控制算法职责，短期为了真机稳定可以保留，长期应变轻。
- 后续新增实时控制能力时，优先考虑新增 `easyarm_control`，而不是继续往 `easyarm_hardware` 或 `easyarm_motion_server` 塞实时控制细节。

## Current Package Roles

当前 workspace 里主要包的职责可以理解为：

| Package | 当前角色 | 架构判断 |
|---|---|---|
| `robstride_can` | 当前真实使用的 Robstride CAN 电机驱动 | 合理，属于底层硬件协议库 |
| `easyarm_can` | 通用 CAN 电机抽象，支持多 vendor | 方向可以，但需要明确和 `robstride_can` 的关系 |
| `easyarm_hardware` | `ros2_control` `SystemInterface`，连接 controller 和真实电机 | 必要，但当前偏重 |
| `easyarm_dynamics` | Pinocchio/Eigen 动力学模型 | 合理，应保持纯动力学库 |
| `easyarm_description` | URDF、mesh、RViz 资源 | 合理 |
| `easyarm_a1_moveit_config` | MoveIt 和 ros2_control 配置 | 合理，应保持配置包定位 |
| `easyarm_a1_bringup` | EasyArm A1 主启动入口 | 合理，应作为正式启动包 |
| `easyarm_interfaces` | EasyArm 自定义 action/srv/msg | 合理 |
| `easyarm_motion_server` | EasyArm 运动能力统一服务层 | 方向正确，是后续 API 核心 |
| `easyarm_app` | CLI、shell、上层测试入口 | 合理，应用层不应直接碰硬件 |
| `easyarm_move_task` | 早期测试和任务工具包 | 历史包，建议冻结并逐步迁移 |
| `easyarm_utils` | 辅助脚本和工具 | 可保留，但要避免变成正式控制入口 |
| `easyarm_calib` | 早期实现的标定工具 | 可保留在运行链路外，但部分机器人状态读取和模型路径仍是旧实现风格 |

## Main Move Chain

当前 `MoveJ/MoveL` 主链路是：

```text
easyarm_app
  -> easyarm_motion_server
    -> MoveIt / Pilz
      -> arm_controller (JTC)
        -> easyarm_hardware
          -> robstride_can
            -> motors
```

这条链路是合理的：

- `easyarm_app` 是用户入口，不直接依赖 MoveIt 和硬件细节。
- `easyarm_motion_server` 作为统一运动服务层，封装 `MoveJ/MoveL`、模式切换、stop、状态查询。
- `MoveIt/Pilz` 负责规划。
- `JointTrajectoryController` 适合执行规划式轨迹。
- `easyarm_hardware` 负责把 ros2_control command interface 转成电机协议。

`MoveJ/MoveL` 属于 `MOVE` 类运动：输入是单点或低频目标，规划后执行完整轨迹。因此继续使用 JTC 是合理的。

## Current Servo Chain

当前 `SpeedJ/SpeedL` 链路是：

```text
easyarm_app
  -> /servo_node/start_servo
  -> /servo_node/delta_joint_cmds 或 /servo_node/delta_twist_cmds
    -> MoveIt Servo
      -> /arm_controller/joint_trajectory
        -> arm_controller (JTC)
          -> easyarm_hardware
```

这条链路目前可以作为测试链路，但不应作为最终架构。

当前问题：

- `easyarm_app` 直接调用 MoveIt Servo 原生接口，绕过了 `easyarm_motion_server`。
- MoveIt Servo 输出仍接到 JTC。
- `publish_period` 当前被压到 `0.02s`，也就是 50Hz，目的是避免 JTC 高频短轨迹抖动。
- 这没有发挥 MoveIt Servo 高频实时输出能力。
- `SpeedJ/SpeedL` 是 `SERVO` 类能力，应该走 forward controller 或自定义 streaming controller，而不是继续沿 JTC 路线打磨。

后续目标链路应调整为：

```text
easyarm_app
  -> easyarm_motion_server
    -> MoveIt Servo
      -> servo_forward_controller 或 EasyArmServoController
        -> easyarm_hardware
```

## Bringup And Config Boundary

当前 `easyarm_a1_bringup` 作为主启动入口是合理的。

它负责启动：

- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `arm_controller`
- `move_group`
- `easyarm_motion_server`
- 可选 `servo_node`
- 可选 RViz

`easyarm_a1_moveit_config` 继续作为配置来源也是合理的。MoveIt config 包不应该承担完整系统 bringup 职责。

需要注意：

- `easyarm_motion_server/launch/h0616.launch.py` 和 `easyarm_a1_bringup/launch/bringup.launch.py` 存在一定职责重叠。
- 长期应明确：正式启动用 `easyarm_a1_bringup`；`easyarm_motion_server` 自带 launch 只作为开发或单节点测试入口。

## Hardware Layer Assessment

`easyarm_hardware` 当前承担：

- `ros2_control` `SystemInterface`
- CAN 驱动初始化和反馈读取
- command/state interface 暴露
- 电机 enable / disable / mode switch
- `IDLE/POSITION/DRAG` 模式管理
- 重力补偿
- velocity feedforward
- command smoothing/filter
- debug logging

这里的关键问题不是单个 `.cpp` 文件超过 1000 行，而是 hardware 层承担了控制算法职责。

短期保留是合理的，因为：

- `DRAG` 已经真机可用。
- 重力补偿已经服务于当前拖拽手感。
- 贸然迁移会影响安全和稳定性。

长期边界应调整为：

```text
controller
  -> 读取 joint state
  -> 接收 position / velocity / effort 目标
  -> 调用 easyarm_dynamics
  -> 输出 position / velocity / effort command
  -> easyarm_hardware
```

`easyarm_hardware` 长期只负责：

- command interfaces 到电机协议的转换
- motor direction / offset / protocol range clamp
- CAN 发送和反馈解析
- state interfaces 发布
- 电机 enable / disable / mode switch
- 最后一层安全限幅和故障保护

应逐步迁到 controller 层的内容：

- gravity compensation
- velocity feedforward
- acceleration feedforward
- inverse dynamics
- friction compensation
- impedance / damping control
- streaming command smoothing

## Controller Direction

后续建议新增 `easyarm_control` 包，用于承载正式 controller：

```text
easyarm_control
  EasyArmServoController
  EasyArmDragController       # 后续评估
  EasyArmTrajectoryController # 后续评估
```

优先级建议：

1. `EasyArmServoController`
   - 服务 `SpeedJ/SpeedL/ServoJ/ServoL`。
   - 高频流式控制。
   - 接收来自 MoveIt Servo 或 motion server 的实时目标。
   - 调用 `easyarm_dynamics`。
   - 输出 `position + velocity + effort` command interfaces。

2. `EasyArmDragController`
   - 当前 `DRAG` 已在 hardware 中可用，短期不迁移。
   - 等 SERVO controller 稳定后，再评估是否迁移拖拽模式。

3. `EasyArmTrajectoryController`
   - 当前 `MoveJ/MoveL` 使用 JTC 是合理的。
   - 只有 JTC 无法满足后续需求时，再考虑自定义轨迹 controller。

## CAN Layer Assessment

当前存在两套 CAN 相关包：

- `robstride_can`
- `easyarm_can`

`robstride_can` 是当前真实链路中使用的 Robstride 私有协议驱动。

`easyarm_can` 看起来是更通用的 CAN 电机抽象，支持多 vendor，例如 xhumanoid、ti5robot、jxservo。

这两个包可以并存，但需要明确定位：

- 如果 `easyarm_can` 是未来统一电机抽象层，应规划 `robstride_can` 如何接入或迁移。
- 如果 `easyarm_can` 只是实验包，应避免它进入正式控制链路。
- 不建议长期保留两套都可能被上层直接调用的 CAN 入口，否则硬件适配边界会变模糊。

## Legacy And Utility Packages

`easyarm_move_task` 当前是历史任务包。

它包含：

- record / playback
- safe shutdown demo
- move to ready
- controller mode switch

这些功能以前服务于早期测试，但现在正式链路已经有：

- `easyarm_a1_bringup`
- `easyarm_motion_server`
- `easyarm_app`

建议：

- 不再向 `easyarm_move_task` 添加新正式功能。
- 已经稳定的新功能迁移到 `easyarm_app`、`easyarm_motion_server` 或 `easyarm_a1_bringup`。
- `easyarm_move_task` 保留为历史兼容或逐步拆除。

`easyarm_utils` 可以继续作为辅助工具包，但应避免承载正式控制链路。

## Calibration Package Assessment

`easyarm_calib` 是早期实现的标定工具包，目前没有调用 `easyarm_motion_server`。

当前检查结果：

- `package.xml` 没有依赖 `easyarm_interfaces`。
- 没有调用 `/easyarm/get_joints`、`/easyarm/get_pose`、`/easyarm/set_mode`、`/easyarm/movej` 等 EasyArm motion server 接口。
- `collect_joint_zero_vision.py` 直接订阅 `/joint_states`。
- `optimize_joint_zero_vision.py` 是离线优化，直接读取采集数据、本地 URDF 和 ros2_control xacro。

这说明 `easyarm_calib` 属于早期工具链风格：直接读取 ROS 原生 topic 和本地文件，而不是通过 EasyArm 自己的运动服务层。

需要分情况看：

- `camera_preview`
  - 只是相机预览和采图。
  - 不需要调用 `easyarm_motion_server`。

- `calibrate_camera`
  - 是相机内参离线标定。
  - 不需要调用 `easyarm_motion_server`。

- `optimize_joint_zero_vision`
  - 是离线优化。
  - 不需要在运行时调用 `easyarm_motion_server`。
  - 但它依赖的 URDF/xacro 路径必须和当前机器人配置保持一致。

- `collect_joint_zero_vision`
  - 同时采集图像和当前关节状态。
  - 当前直接订阅 `/joint_states` 可以工作。
  - 如果后续希望所有机器人状态读取都统一收口，可以改为调用 `/easyarm/get_joints`，或者至少把 joint state 读取封装成可替换模块。

当前比较明显的历史痕迹：

```text
src/easyarm_calib/easyarm_calib/joint_zero_vision_common.py
  URDF_PATH = Path("src/easyarm_description/urdf/easyarm_a1_h0521.urdf")
  ROS2_CONTROL_XACRO = Path("src/easyarm_a1_moveit_config/config/easyarm_a1.ros2_control.xacro")
```

其中 `easyarm_a1_h0521.urdf` 可能已经不是当前主线使用的机器人模型。这个问题比是否调用 `easyarm_motion_server`
更容易影响标定结果，因为标定优化会直接依赖 FK 几何链和当前零偏配置。

建议：

- 短期保持 `easyarm_calib` 独立，不强制接入 motion server。
- 优先检查并更新 `URDF_PATH`，确保它和当前 `easyarm_a1_moveit_config` / bringup 使用的模型一致。
- 对 `collect_joint_zero_vision`，后续可以从直接订阅 `/joint_states` 改成可配置来源：
  - 默认继续读 `/joint_states`。
  - 可选调用 `/easyarm/get_joints`。
- 标定结果仍然只生成建议值，不自动修改 xacro，避免误改硬件零偏。

## Workspace Hygiene

当前有几个非核心但值得清理的问题：

- 源码目录里存在 `__pycache__`。
- `ref/Galaxy_Linux_Python_2.4.2503.9202/.../api` 被 `colcon list` 识别成了 `gxipy` 包。
- 如果 `ref/` 只是外部参考 SDK，建议加 `COLCON_IGNORE`，避免全量构建时被意外纳入。

这些不是控制架构问题，但会影响 workspace 的可维护性和构建确定性。

## Target Architecture

一个更干净的长期架构可以是：

```text
robstride_can / easyarm_can
  电机协议和 CAN 通信

easyarm_hardware
  ros2_control SystemInterface
  只做硬件适配、状态反馈、安全限幅、enable/disable、mode switch

easyarm_control
  自定义 ros2_control controller
  负责 SERVO、gravity compensation、feedforward、impedance/admittance 等实时控制

easyarm_dynamics
  Pinocchio 动力学模型
  被 controller 调用

easyarm_description
  URDF / mesh / rviz 资源

easyarm_a1_moveit_config
  MoveIt / ros2_control 配置

easyarm_a1_bringup
  系统主启动入口

easyarm_motion_server
  EasyArm 运动能力统一 API
  MoveJ / MoveL / SpeedJ / SpeedL / ServoJ / ServoL / mode / stop / state

easyarm_app
  CLI、shell、测试工具、上层调用入口

easyarm_calib
  标定工具

easyarm_move_task
  历史任务包，逐步迁移或冻结
```

## Recommended Roadmap

### Short Term

- 保持当前 `MoveJ/MoveL` 链路稳定。
- 保持 `DRAG` 在 `easyarm_hardware` 内，避免影响已验证真机行为。
- 明确 `easyarm_a1_bringup` 是正式启动入口。
- 不再向 `easyarm_move_task` 添加新正式功能。
- 清理 `__pycache__`，并评估是否给 `ref/` 下外部 SDK 加 `COLCON_IGNORE`。

### Mid Term

- 让 `easyarm_motion_server` 封装 `SpeedJ/SpeedL`。
- `easyarm_app` 不再直接调用 MoveIt Servo 原生接口。
- 在 mock 环境验证 MoveIt Servo 输出到 forward controller。
- 评估 `MultiInterfaceForwardCommandController` 是否能稳定输出 `position + velocity`。

### Long Term

- 新增 `easyarm_control`。
- 实现 `EasyArmServoController`。
- 将 SERVO 链路从 JTC 迁移到 forward/custom streaming controller。
- 逐步把 gravity compensation、feedforward、impedance/admittance 从 hardware 层迁移到 controller 层。
- 等 SERVO controller 稳定后，再评估 `DRAG` 是否从 hardware 迁移到 `EasyArmDragController`。

## Architecture Principles

后续判断新功能放在哪里，可以遵循：

- `MOVE`：单点或低频目标，走 MoveIt/Pilz + JTC。
- `SERVO`：高频连续输入，走 MoveIt Servo + forward/custom controller。
- `DRAG`：短期保留 hardware，长期再评估 controller 化。
- `easyarm_app`：只做用户入口，不直接碰硬件，不长期直连 MoveIt Servo 原生接口。
- `easyarm_motion_server`：统一对外运动 API，负责模式协调和运动能力封装。
- `easyarm_hardware`：只做硬件适配和最后安全边界。
- `easyarm_dynamics`：只做动力学计算，不包含控制策略。
- `easyarm_a1_bringup`：负责系统启动编排。
