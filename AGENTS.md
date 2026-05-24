# AGENTS.md

## 仓库形态

- 这是 ROS 2 Humble `colcon` workspace；源码包都在 `src/` 下，且都是 `ament_cmake` 包。
- 当前 `colcon list` 输出的包：`robstride_can`、`easyarm_description`、`easyarm_hardware`、`easyarm_dynamics`、`easyarm_a1_moveit_config`。
- 根目录 `README.md` 为空；遇到冲突时优先相信 `package.xml`、`CMakeLists.txt`、launch 文件和包内 README。

## 沟通约定

- 默认用中文回复用户；代码、命令、路径、API 名称保持原文。

## 包职责边界

- `robstride_can`：独立的 Linux SocketCAN Robstride 驱动库，以及会触碰硬件的 demo：`single_motor_demo`、`discover_motors`、`set_zero_pos`；不要在这里引入 ROS/MoveIt/URDF 依赖。
- `easyarm_hardware`：ros2_control `SystemInterface` 插件，导出名为 `easyarm_hardware/EasyArmHardware`；依赖 `robstride_can`，并从 ros2_control xacro 解析电机/关节参数。
- `easyarm_dynamics`：只放 Pinocchio/Eigen 刚体动力学封装（`RobotModel` 从 URDF 加载模型；计算 gravity、NLE、mass matrix、inverse dynamics）；不要放控制逻辑、硬件 ID、MoveIt 配置、CAN 参数、offset 或 limit。
- `easyarm_description`：只放机器人描述资源（`urdf/`、`meshes/`、`rviz/`、display launch）；MoveIt 配置引用 `easyarm_description/urdf/easyarm_a1_h0521.urdf`。
- `easyarm_a1_moveit_config`：生成的 MoveIt 配置，加上本地 `move_to_ready` 可执行文件和 `safe_shutdown_demo.sh`；其 xacro 将 `EasyARM-A1` 接到 `easyarm_hardware/EasyArmHardware`。

## 常用命令

- 列出包：`colcon list`
- 按依赖顺序构建全部包：`colcon build --packages-select robstride_can easyarm_description easyarm_hardware easyarm_dynamics easyarm_a1_moveit_config`
- 构建单个包：`colcon build --packages-select <package>`
- 构建后在 workspace 根目录加载 overlay：`source install/setup.bash`
- 运行单包 lint/tests：`colcon test --packages-select <package> && colcon test-result --verbose`
- 仓库内没有本地 unit test 目录；`BUILD_TESTING` 目前只为 `robstride_can`、`easyarm_hardware`、`easyarm_dynamics` 添加 `ament_lint_auto` 检查。

## 硬件安全

- 除非用户明确要求，不要修改 motor ID、motor direction、joint limit、zero offset、CAN 参数、control gain、safe shutdown 行为或 `use_mock_hardware` 默认值。
- 硬件映射在 `src/easyarm_a1_moveit_config/config/EasyARM-A1.ros2_control.xacro`：`can_interface=can0`、`host_can_id=0xFD`、`use_mock_hardware=false`、关节 `Joint1`-`Joint6`，以及各 motor type/ID/direction。
- `ros2 launch easyarm_a1_moveit_config demo.launch.py` 默认会使用真实硬件，因为 `use_mock_hardware` 是 false。
- 不要运行 `single_motor_demo`、`discover_motors`、`set_zero_pos`、`move_to_ready` 或 `safe_shutdown_demo.sh`，除非用户确认硬件已连接且处于安全状态；这些程序会 enable/disable 电机或执行运动/零位设置。
- 真实硬件测试前必须先配置 SocketCAN，例如 `sudo ip link set can0 down`、`sudo ip link set can0 type can bitrate 1000000`、`sudo ip link set can0 up`。
- 关机优先使用根目录 wrapper：`scripts/safe_shutdown_easyarm.sh`；它会在存在时 source `install/setup.bash`，再运行 `src/easyarm_a1_moveit_config/scripts/safe_shutdown_demo.sh`。

## 风格和 API 注意事项

- C++ 标准是 C++17；活跃包使用 `-Wall -Wextra -Wpedantic` 编译。
- Public C++ header 使用中文 Doxygen block comment，并写清单位（`rad`、`rad/s`、`Nm`）；风格参考 `src/robstride_can/include/robstride_can/robstride_can_driver.hpp`。
- 保持 `easyarm_hardware` 的 plugin XML class name 和 MoveIt ros2_control plugin string 一致：`easyarm_hardware/EasyArmHardware`。

## Git

- 如需提交，commit message 使用 Conventional Commits，summary 用中文，例如 `feat(dynamics): 新增 Pinocchio 重力补偿接口`。
- 如果改动涉及硬件行为但没有做硬件测试，在 commit body 中说明。
