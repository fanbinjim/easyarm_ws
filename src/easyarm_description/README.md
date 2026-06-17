# easyarm_description

`easyarm_description` 保存 EasyArm 的机器人描述资源，包括 URDF、xacro、
mesh、RViz 配置和描述参数。

## H0617 xacro

H0617 使用 `config/h0617/*.yaml` 管理 link、joint 和 inertial 参数，并通过
`urdf/easyarm_a1_h0617.urdf.xacro` 生成 plain URDF。

源码树下重新生成：

```bash
cd /home/linx/easyarm_ws
ros2 run xacro xacro \
  src/easyarm_description/urdf/easyarm_a1_h0617.urdf.xacro \
  > src/easyarm_description/urdf/easyarm_a1_h0617.urdf
```

安装后验证 xacro 能展开：

```bash
cd /home/linx/easyarm_ws
source install/setup.bash
ros2 run xacro xacro \
  "$(ros2 pkg prefix easyarm_description)/share/easyarm_description/urdf/easyarm_a1_h0617.urdf.xacro" \
  > /tmp/easyarm_a1_h0617.urdf
```

`urdf/easyarm_a1_h0617.urdf` 是由 xacro 生成的兼容文件，给 Pinocchio 或其他只接受
plain URDF 路径的工具使用；不要手工修改它，改参数请修改 `config/h0617/` 后重新生成。

## 标定参数配置

`config/h0617_cali/` 是由质量/质心标定结果导出的 H0617 配置，格式和 `config/h0617/`
一致。它只替换 `Link2`-`Link6` 的 `mass` 与 `origin.xyz`，惯量矩阵、link mesh 和 joint
参数仍来自 H0617 模板。

使用标定配置展开 URDF：

```bash
cd /home/linx/easyarm_ws
ros2 run xacro xacro \
  src/easyarm_description/urdf/easyarm_a1_h0617.urdf.xacro \
  config_root:=../config/h0617_cali \
  > /tmp/easyarm_a1_h0617_cali.urdf
```
