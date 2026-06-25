# 开发计划与技术债

## 当前阶段

前端已完成模块化重构，从单文件原型（main.tsx ~1040行）拆分为组件化架构。3D 查看器、Toast 通知、ConfirmDialog、SettingsDialog 均已实现。

## 近期已完成

1. 模块化重构
   - 抽离 `src/api/client.ts` - 统一 HTTP/WS 客户端
   - 抽离 `src/api/types.ts` - 所有类型定义
   - 拆出 UI 原语: Panel, Metric, StatusPill, SummaryCard, NumberGrid, PoseEditor, Range, StreamButtons
   - 拆出组件: StatusBar, RobotViewer, MotionPanel, StreamPanel, JointTable, PosePanel, ActionLog, ControllerList, RosLog
   - 拆出通用组件: ConfirmDialog, SettingsDialog, Toast
   - 拆出 hooks: useSettings, useHealth, useApiState, useTelemetry

2. 3D 机器人查看器
   - 使用 Three.js + urdf-loader
   - 从 /api/robot/model 和 /api/robot/description 获取模型
   - 从 /api/robot/assets/ 加载 mesh，支持 query token 鉴权
   - 通过 telemetry latest_joints 更新关节角度
   - 状态: loading / error / empty / ready
   - 提供 Reset Camera 按钮

3. 安全与 UX 增强
   - Stop/Cancel 按钮根据 active_action 自动选择正确接口
   - Safe Shutdown 独立按钮 + 详细后果说明
   - 执行模式切换需二次确认
   - 危险操作三级视觉分级
   - Toast 通知系统替代 inline error 堆叠
   - Settings 面板集中管理 token 和 backend URL
   - JointTable 支持 rad/deg 显示切换
   - Named States 空状态明确提示
   - Health 轮询改为 1.5s (原 0.5s)
   - Telemetry 自动重连 + 指数退避 + stale 检测

4. 后端缺口跟进
   - Gap 1 (is_mock_hardware) 已修复
   - Gap 2-4 待后端实现

## 中期计划

- 引入 mock backend 或 MSW，用于无 ROS 环境下开发 UI
- 增加 session 状态页，展示 backend version、token 状态
- 支持命令历史和 action feedback 时间线
- 支持更细粒度的 error 分类
- 根据真实联调反馈优化 MoveJ/MoveL 默认值

## 技术债

- 3D 查看器 mesh 加载未实现真正的 STL/DAE 解析（当前使用 fallback geometry）
- Telemetry 消息中的 runtime_active 字段尚未从后端获取（Gap 3）
- 未配 lint/format/unit test/E2E test
- 未做无 ROS mock 数据源
- MoveJ/MoveL 默认值硬编码在前端

## 不做事项

- 不在前端实现运动规划、动力学、控制律或硬件通信
- 不在前端维护 motor 映射、CAN 参数、zero offset、joint limit 或安全关机策略
- 不绕过 easyarm_web_bridge 直接访问 ROS graph
