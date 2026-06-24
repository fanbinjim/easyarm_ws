# 开发计划与技术债

## 当前阶段判断

前端已经具备最小可用调试能力，但仍处于原型到工程化过渡阶段。主要风险不是功能缺失，而是接口类型、UI 安全策略、错误处理和模块边界还没有沉淀成稳定结构。

## 近期计划

1. 文档与边界固化
   - 维护 `README.md`、`AGENTS.md`、`docs/api.md`、`docs/architecture.md`
   - 将前端只能修改 `web_app/` 的规则写入协作约束
   - 明确真实运动相关 UI 的确认和停止策略

2. UI 文案与编码修复
   - 修复 `src/main.tsx` 中部分中文文案编码异常
   - 统一按钮、面板、错误提示和安全确认文本
   - 为数值输入补齐单位和合理范围提示

3. 模块化重构
   - 抽离 `src/api/client.ts`
   - 抽离 `src/api/types.ts`
   - 拆出 `StatusPanel`、`MotionPanel`、`StreamPanel`、`LogPanel`
   - 保留当前功能行为，避免重构期间改变硬件相关语义

4. 安全与可用性增强
   - 对 `execute=true` 的运动命令增加更强确认态
   - 明确展示 backend 连接目标、mock/real hardware 状态和 joint state 新鲜度
   - 对 stream 控制增加发送状态、失败提示和自动 halt 可见反馈

5. 测试与质量门禁
   - 增加前端 lint/format 方案
   - 增加 API client 单元测试
   - 增加关键交互的浏览器自动化测试
   - 将 `npm run build` 作为最低提交前检查

## 中期计划

- 引入 mock backend 或 MSW，用于无 ROS 环境下开发 UI。
- 增加 session 状态页，展示 backend version、token 状态、WebSocket 重连状态。
- 支持命令历史和 action feedback 时间线。
- 支持更细粒度的 error 分类：鉴权、连接、ROS service/action timeout、参数错误、motion failure。
- 根据真实联调反馈优化 MoveJ/MoveL 默认值来源，避免前端硬编码业务参数。

## 技术债

- `src/main.tsx` 单文件过大，组件、API、类型和业务逻辑混在一起。
- 中文文案存在编码异常，影响可读性和操作者信任感。
- 前端类型与 backend Python 实现手动同步，缺少契约测试或 schema 生成。
- `localStorage` 保存 token 简单直接，但没有过期、清除和多环境提示。
- fetch 错误展示为原始字符串，缺少面向操作者的错误分级。
- WebSocket command-stream 当前没有消费返回消息，也没有展示流式命令 publish 失败。
- MoveJ、MoveL 默认值目前写在前端代码中，后续应确认是否来自 named state、backend 配置或用户 preset。
- 没有 lint、format、unit test、E2E test 配置。
- 没有无 ROS mock 数据源，导致 UI 开发依赖 backend 可用性。
- 没有明确区分 real hardware 与 mock hardware 的高可见 UI 风险提示。

## 不做事项

- 不在前端实现运动规划、动力学、控制律或硬件通信。
- 不在前端维护 motor 映射、CAN 参数、zero offset、joint limit 或安全关机策略。
- 不绕过 `easyarm_web_bridge` 直接访问 ROS graph。
- 不为了 UI 方便修改 ROS package 或硬件配置文件。
