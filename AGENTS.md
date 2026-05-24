# AGENTS.md

## 项目概览

这是 EasyArm 的 ROS 2 开发 workspace。

主要包：

- `easyarm_description`：存放 URDF、mesh、RViz 配置和模型预览 launch。
- `easyarm_hardware`：存放 EasyArm 的 ros2_control hardware interface。
- `easyarm_dynamics`：存放 EasyArm 的刚体动力学计算封装。
- `easyarm_a1_moveit_config`：EasyARM-A1 的 MoveIt 配置。
- `robstride_can`：Robstride 电机 CAN 通信驱动库。

## Workspace 路径

主开发目录使用占位符表示，请替换为真实的路径：

```bash
WORKSPACE=<你的 EasyArm workspace 根目录>
```

源码目录：

```bash
$WORKSPACE/src
```

## 编译方式

在 workspace 根目录执行：

```bash
cd "$WORKSPACE"
colcon build --packages-select robstride_can easyarm_description easyarm_hardware easyarm_dynamics easyarm_a1_moveit_config
```

编译后加载环境：

```bash
source "$WORKSPACE/install/setup.bash"
```

## 包职责划分

### easyarm_description

这里只放机器人描述和可视化资源：

- `urdf/`
- `meshes/`
- `rviz/`
- 模型预览 launch 文件


### easyarm_hardware

这里只放 ros2_control 硬件接口相关内容：

- C++ 源码
- 头文件
- plugin XML
- hardware 相关的 CMake/package 配置

### easyarm_dynamics

这里只放机器人刚体动力学计算相关内容：

- Pinocchio / Eigen 动力学模型封装
- 重力项、科氏/离心项、质量矩阵、逆动力学等计算接口
- 面向控制层复用的 clean API

不要在这里放：

- 控制逻辑
- hardware interface 代码
- MoveIt 相关代码
- 电机 ID、方向、限位、零点或 CAN 参数管理


### easyarm_a1_moveit_config

MoveIt 配置包。

它应该引用：

- `easyarm_description` 中的机器人模型
- `easyarm_hardware/EasyArmHardware` 这个硬件插件

### robstride_can

底层 Robstride CAN 驱动库。

保持它独立，不要让它依赖 MoveIt、URDF 或 RViz 相关包。

## 命名约定

当前使用的包名：

- `easyarm_description`
- `easyarm_hardware`
- `easyarm_dynamics`
- `easyarm_a1_moveit_config`
- `robstride_can`

## 注释风格

C++ 头文件中的文件说明、类说明和对外 public API 注释，优先参考：

```text
src/robstride_can/include/robstride_can/robstride_can_driver.hpp
```

注释要求：

- 使用 Doxygen 风格块注释。
- 文件头使用 `@file` 和 `@brief`。
- 类、结构体、枚举使用 `@brief`。
- public 函数使用 `@brief`、必要的 `@param` 和 `@return`。
- 注释优先使用中文，单位和符号保持清晰，例如 `rad`、`rad/s`、`Nm`、`q`、`qd`。
- private 辅助函数不强制写注释，除非逻辑复杂或容易误用。
- 注释解释接口语义、输入输出、单位和约束，不写重复代码本身的空泛描述。

## Git 提交规范

提交代码时，commit message 使用 Conventional Commits 风格，描述内容尽量使用中文。

格式：

```text
<type>(<scope>): <简短描述>

<详细说明（可选，空一行后写）>

<footer（可选，如 BREAKING CHANGE 或关联 issue）>
```

其中：

- `type` 使用英文固定类型。
- `scope` 使用英文或包名，例如 `hardware`、`dynamics`、`can`、`docs`，可省略。
- `<简短描述>` 和详细说明优先使用中文。

要求：

- 标题用一句中文概括本次改动，且遵循 `<type>(<scope>): <简短描述>` 格式。
- 正文用中文条目说明关键修改内容。
- 如果新增功能，但未运行测试、未做功能测试或未做硬件测试，需要在正文中明确说明，文档除外。
- 避免只写 `update`、`fix`、`change` 这类信息不足的提交标题。

常用 `type`：

| type | 含义 | 示例 |
| --- | --- | --- |
| `feat` | 新功能 | `feat(dynamics): 新增 Pinocchio 重力补偿接口` |
| `fix` | 修 bug | `fix(can): 修复快速电机指令下的缓冲区越界风险` |
| `refactor` | 重构，不改变功能 | `refactor(controller): 将重力计算移出硬件接口` |
| `perf` | 性能优化 | `perf(write): 在关节循环外预计算重力向量` |
| `docs` | 文档 | `docs: 新增控制器链路架构说明` |
| `style` | 格式调整 | `style(hardware): 对硬件源码应用 clang-format` |
| `test` | 测试 | `test(dynamics): 新增 NLE 计算单元测试` |
| `chore` | 杂项、构建、依赖 | `chore: 更新 ros2_control 依赖版本` |


## 修改后检查

完成修改后，优先执行：

```bash
cd "$WORKSPACE"
colcon list
colcon build --packages-select robstride_can easyarm_description easyarm_hardware easyarm_dynamics easyarm_a1_moveit_config
```


## 安全注意事项

不要随意修改以下内容，除非用户明确要求并说明原因：

- 电机 ID
- 电机方向
- 关节限位
- 零点偏移
- safe shutdown 流程
- CAN 通信参数
