# MIRLab Heightmap Alignment Changes

本文记录 `go2w_policy_cusrl_mirlab` 在 StepIt/MuJoCo sim2sim 中与 MIRLab/IsaacLab 训练侧对齐的当前状态、已经做过的实验，以及当前 YAML 配置和 MIRLab 训练配置之间的差异。

## 当前有效状态

当前策略目录：

```text
/home/applepie/project_for_test/go2w_cusrl_demo/stepit/policy/go2w_policy_cusrl_mirlab
```

当前有效配置要点：

- `policy.yml` 中 actuator 保持 `bias: 0.0, scale: 1.0`。
- `policy.yml` 中没有启用 `dummy_heightmap_source`，因此 `heightmap` 由 ROS2 `heightmap_subscriber` 自动解析提供。
- `field_ops.yml` 只将 wheel joint position 置零。
- `heightmap.yml` 启用了 `clip + nan_to_num + zero_mean`。
- `actor.onnx` 内部已经包含 observation normalization、observation RMS normalization 和 action denormalization。

这意味着不要再在 `policy.yml` 或 `field_ops.yml` 外部重复添加 MIRLab 训练侧的 action scale、default joint position bias、`ang_vel * 0.25`、`joint_vel * 0.05` 等处理；当前 ONNX 已经做了这些处理。

## MIRLab 训练侧参考

MIRLab 训练侧高程图观测逻辑：

```python
height_scan = ray_hits_w[..., -1]
height_scan = clip(height_scan, -5.0, 5.0)
height_scan = nan_to_num(height_scan)
height_scan = (height_scan - height_scan.mean()) * obs_scales.height_scan
actor_obs = cat([actor_obs, height_scan], dim=-1)
critic_obs = cat([critic_obs, height_scan], dim=-1)
```

GO2W 训练配置中的关键参数：

```python
self.scene.height_scanner.enable_height_scan = True
self.scene.height_scanner.prim_body_name = "base"

self.action.joint_pos.scale = {
    ".*_hip_joint": 0.125,
    "^(?!.*_hip_joint).*": 0.25,
}
self.action.joint_vel.scale = 5.0

self.normalization.obs_scales.ang_vel = 0.25
self.normalization.obs_scales.joint_vel = 0.05
```

训练侧观测中：

- `joint_pos` 是 `q - default_q`。
- `joint_vel` 是 `dq - default_dq`，默认速度为 0。
- wheel joint position 在训练侧为 0。
- `last_action` 是 action manager 中的 action。

## StepIt 高程图配置

文件：

```text
policy/go2w_policy_cusrl_mirlab/heightmap.yml
```

当前关键配置：

```yaml
dimension: [17, 11]
grid_size: [0.1, 0.1]
x_major: true

elevation_clip_min: -5.0
elevation_clip_max: 5.0
elevation_nan_to_num: true
elevation_nan_value: 0.0
elevation_zero_mean: true
```

说明：

- `dimension: [17, 11]` 对应 `actor.yml` 中的 `heightmap: 187`。
- `x_major: true` 与当前导出的 `17 x 11` 采样展开顺序保持一致。
- `elevation_clip_min/max` 对齐 MIRLab 的 `clip(height_scan, -5.0, 5.0)`。
- `elevation_nan_to_num: true` 和 `elevation_nan_value: 0.0` 对齐 `nan_to_num(height_scan)`。
- `elevation_zero_mean: true` 对齐 `height_scan - height_scan.mean()`。
- 当前 `obs_scales.height_scan` 在导出图中等效为 1，因此 `field_ops.yml` 中没有额外的 heightmap scale/bias。

当前定位配置：

```yaml
localization_subscriber:
    robot_frame_id: "radar"
    timeout_threshold: 0.1
```

MIRLab/IsaacLab 的 height scanner 挂在 `base`。MuJoCo/StepIt 侧使用 `radar` frame；只有当 `radar` 在 MuJoCo 中挂到 base 原点时，采样中心才等效于 IsaacLab 的 `base`。

## StepIt 代码改动

修改文件：

```text
plugin/policy_neuro_ros2/include/stepit/policy_neuro_ros2/heightmap_subscriber2.h
plugin/policy_neuro_ros2/src/heightmap_subscriber2.cpp
```

新增并使用的配置项：

```yaml
elevation_nan_to_num
elevation_nan_value
elevation_clip_min
elevation_clip_max
```

当 `elevation_nan_to_num: true` 时，ROS2 heightmap subscriber 当前处理顺序为：

1. 对每个采样点执行 `clamp(elevation, elevation_clip_min, elevation_clip_max)`。
2. 将非有限值替换为 `elevation_nan_value`。
3. 对全部采样点求均值。
4. 如果 `elevation_zero_mean: true`，减去该均值。

如果未开启 `elevation_nan_to_num`，先裁剪有限值，再只对有限值求均值，并用均值填补非有限值。未设置裁剪范围时，默认范围为负无穷到正无穷，此时与旧行为一致；仅关闭该开关不会关闭已配置的有限范围裁剪。

## 当前 Policy YAML 和 MIRLab 训练的差异

表面差异：

- MIRLab 训练代码中 action scale 是 hip `0.125`、thigh/calf `0.25`、wheel velocity `5.0`。
- 当前 `policy.yml` 中 actuator 是 `bias: 0.0, scale: 1.0`。
- MIRLab 训练代码中观测做了 `ang_vel * 0.25`、`joint_pos - default_q`、`joint_vel * 0.05`。
- 当前 `field_ops.yml` 只做 wheel joint position mask。

实际语义：

当前 `actor.onnx` 已经内置这些处理。ONNX 中存在以下节点和常量：

```text
observation_normalization.mean
observation_normalization.std
observation_rms.mean
observation_rms.std
action_denormalization.mean
action_denormalization.std
```

检查到的关键常量：

```text
observation input shape: [1, 1, 244]
action output shape:     [1, 1, 16]

observation_normalization.mean[joint_pos]:
  [0.0, 0.8, -1.5] x 4, wheels [0.0, 0.0, 0.0, 0.0]

observation_normalization.std[ang_vel]:
  [4.0, 4.0, 4.0]    # 等效于外部 ang_vel * 0.25

observation_normalization.std[joint_vel]:
  [20.0] x 16        # 等效于外部 joint_vel * 0.05

observation_normalization.std[last_action]:
  [0.125, 0.25, 0.25] x 4, wheels [5.0, 5.0, 5.0, 5.0]

action_denormalization.mean:
  [0.0, 0.8, -1.5] x 4, wheels [0.0, 0.0, 0.0, 0.0]

action_denormalization.std:
  [0.125, 0.25, 0.25] x 4, wheels [5.0, 5.0, 5.0, 5.0]
```

因此当前策略要求 StepIt 传入原始物理量，再由 ONNX 内部完成归一化和动作反归一化。之前将 MIRLab 训练侧的 action scale/bias 直接写入 `policy.yml`，会造成重复反归一化，实测策略表现变差，已回退。

### `action_mean/default_action` 细节

当前 MIRLab policy 没有显式写 `action_mean` 或 `default_action`。StepIt 的 `ActionHistory` 会调用 `populateArray(default_action, action_dim)`；当 `default_action` 为空时，会填充为全 0。因此当前 policy 的 reset 初始 action history 等效于：

```yaml
action_mean: 0.0
```

这和盲走策略显式写 `action_mean: 0.0` 的行为一致。

需要注意：ONNX 中 `observation_normalization.mean[last_action]` 是默认站姿，`std[last_action]` 是 action scale。因此 reset 后第一帧的全 0 `last_action` 会被 ONNX 解释为一个非零的归一化 last action；一旦 actor 输出第一帧 action，之后 `last_action` 就来自上一帧 ONNX 输出的物理 action。这个差异主要影响 reset 后最初几帧，不太可能单独解释稳定行走时的平地 gait 差异，但可以作为后续小实验单独验证。

## 与盲走策略的对照

参考策略：

```text
policy/2026-03-18-00-01-25:new
```

盲走策略配置特点：

- `policy.yml` actuator 不设置 scale/bias，StepIt 默认 `scale=1, bias=0`。
- `field_ops.yml` 只 mask wheel joint position。
- `cmd_vel.yml` 与当前 MIRLab policy 一致。
- `actor.yml` 输入为 57 维，没有 heightmap。

当前 MIRLab policy：

- `actor.yml` 输入为 244 维，包含 187 维 heightmap。
- 使用 RNN memory 输入输出。
- `last_action` 和盲走策略中的 `action_p1` 在 StepIt 中由同一个 `ActionHistory` 模块提供，是别名关系。

结论：盲走策略支持当前 actuator/field_ops 保持简单配置，但不能直接拷贝盲走的 `actor.yml`，因为当前策略有高程图输入和不同的 ONNX 图结构。

## 全 0 高程图实验

实验方式：

在 `policy.yml` 中临时显式加入：

```yaml
modules:
  - cmd_vel_source
  - field_ops
  - dummy_heightmap_source
  - actor
```

`dummy_heightmap_source` 会强制提供全 0 的 `heightmap`，绕过 ROS2 `heightmap_subscriber`。

实验结果：

- 平地表现和真实高程图时基本一致。
- 上楼梯表现明显变差。

结论：

- 平地上“不是纯轮子滚、会出现抬腿倾向”的主因不是高程图。
- 真实高程图对上楼梯是有效输入，不能长期关闭。
- 当前配置已恢复为真实高程图路径，即 `policy.yml` 中不包含 `dummy_heightmap_source`。

## 工作流文档改动

文件：

```text
README_WORKFLOW.md
```

第二步启动策略路径为当前实际使用的 MIRLab policy：

```bash
cd /home/applepie/project_for_test/go2w_cusrl_demo/stepit/stepit_ws
./scripts/run.sh ./configs/go2w.sh -p /home/applepie/project_for_test/go2w_cusrl_demo/stepit/policy/go2w_policy_cusrl_mirlab
```

## 验证

已执行：

```bash
cd /home/applepie/project_for_test/go2w_cusrl_demo/stepit/stepit_ws
./scripts/build.sh cmake
```

构建和安装成功，`stepit_plugin_policy_neuro_ros2` 已通过编译。

本次文档更新前后确认：

- `policy.yml` 当前不启用 `dummy_heightmap_source`。
- `policy.yml` actuator 当前仍为 `bias=0, scale=1`。
- `field_ops.yml` 当前只 mask wheel joint position。
- `heightmap.yml` 当前启用 `clip + nan_to_num + zero_mean`。

## 后续排查方向

基于当前实验，平地行为差异更应优先排查：

- 当前 ONNX 在平地时输出的 16 维 action，其中 wheel velocity action 是否足够大。
- MuJoCo 中 wheel contact、friction、joint damping 和 IsaacLab 的差异。
- MuJoCo/StepIt 的 joint order、wheel velocity sign 是否与导出 ONNX 一致。
- `cmd_vel` 输入是否和训练命令范围一致。

高程图方向应继续关注：

- `radar` frame 是否始终等效于 IsaacLab 的 `base`。
- `/elevation_map` 是否只表达地形，不包含机器人自身几何。
- `heightmap_samples` 点云是否随机器人中心稳定展开，是否存在定位漂移。

## ROS1 高程预处理迁移（2026-10-28）

本节对应 `policy/go2w_policy_cusrl_mirlab_6w_ros1`。将外层 ROS2 版本的参数化高程预处理迁移到 ROS1，使 ROS1 支持相同的裁剪、缺测填补和去均值流程。

### 修改文件与内容

以下路径均相对于 `/home/applepie/project_for_test/go2w_cusrl_demo/stepit`：

| 文件 | 具体改动 |
| --- | --- |
| `plugin/policy_neuro_ros/include/stepit/policy_neuro_ros/heightmap_subscriber.h` | 增加 `<limits>`，以及 `elevation_nan_to_num_`、`elevation_nan_value_`、`elevation_clip_min_`、`elevation_clip_max_` 四个成员 |
| `plugin/policy_neuro_ros/src/heightmap_subscriber.cpp` | 构造函数读取四个 YAML 参数；`update()` 将旧的均值填补代码扩展为可配置的两条处理路径 |
| `stepit_ws/src/stepit/plugin/policy_neuro_ros/include/stepit/policy_neuro_ros/heightmap_subscriber.h` | 同步上述头文件改动，供实际工作区构建 |
| `stepit_ws/src/stepit/plugin/policy_neuro_ros/src/heightmap_subscriber.cpp` | 同步上述源文件改动，供实际工作区构建 |
| `policy/go2w_policy_cusrl_mirlab_6w_ros1/heightmap.yml` | 启用裁剪 `[-5,5]`、NaN 补 0 和单图去均值，移除“参数未实现”的旧注释 |
| `policy/go2w_policy_cusrl_mirlab_6w_ros1/README.md` | 更新预处理和部署状态，说明本项目工作区路径、重编译要求及剩余验证边界 |

外层和 `stepit_ws/src/stepit` 是两份源码。本次同步修改两份 ROS1 `.h/.cpp`；现有 CMake 构建指向 `stepit_ws/src/stepit`，只修改外层文件不会自动更新该工作区产物。

### 配置及数据流程

ROS1 策略采用：

```yaml
dimension: [17, 11]
grid_size: [0.1, 0.1]
x_major: true
elevation_clip_min: -5.0
elevation_clip_max: 5.0
elevation_nan_to_num: true
elevation_nan_value: 0.0
elevation_zero_mean: true
```

`elevation_nan_to_num=true` 时：

```text
ROS1 GridMap -> 按机器人位置和航向采样187点
-> 裁剪[-5,5] -> 将剩余非有限值替换为0
-> 计算全部187点的均值 -> 减去均值 -> 提供heightmap字段
-> actor组装244维观测 -> ONNX内部统计归一化及推理
```

NaN 会被替换为 0；由于先裁剪，`+Inf/-Inf` 会变成 `+5/-5`。补值在求均值之前执行，因此补 0 与旧的“填有效高度均值”方案会产生不同结果。例如 `[1,2,NaN]` 在这两种方案中分别得到 `[0,1,-1]` 与 `[-0.5,0.5,0]`。

`elevation_nan_to_num=false` 时：只裁剪有限高度，按有限高度计算均值，再用该均值填补 NaN/Inf。新成员默认值为 `false`、`0.0`、负无穷、正无穷；旧策略不设置新参数时保持原有处理行为。`elevation_zero_mean` 继续控制是否去均值。

### ROS1 适配细节

ROS2 使用 C++17 的 `std::clamp`；本 ROS1 工程默认 C++14，因此使用以下等价表达式，并保持 NaN 在后续填补前不被误改成边界值：

```cpp
elevation = elevation < elevation_clip_min_
                ? elevation_clip_min_
                : (elevation_clip_max_ < elevation ? elevation_clip_max_ : elevation);
```

保持原有 `sample_z_offset_ += mean`，用于还原调试点云的显示高度。ROS1 的 `ros::Subscriber`、TF 查询、`GridMapRosConverter::fromMessage`、`map_info_.header.stamp`、不确定度处理和 187 点采样顺序继续使用原实现。

当前 ROS1 配置仍订阅 `/elevation_mapping_gk/elevation_map_raw` 的 `elevation` 层，并将 `uncertainty_layer` 设为空字符串。`robot_frame_id` 仍是待确认的实机采样中心占位名称，必须按实际 TF 填写。

ONNX 中的统计归一化和动作反归一化继续由模型完成。本次实现与 ROS2 部署侧的单图处理一致；训练代码旧默认分支的 `height_scan.mean()` 是跨环境批次均值，不能据此声称已消除训练与部署的均值范围差别。

### 本次验证与剩余工作

已从实际 ROS1/ROS2 源文件提取数值处理块，分别按 C++14/C++17 编译并执行，使用独立 NumPy 结果和手算示例验证：

- 46 组场景，两个版本共执行 92 次，均符合独立预期，且 ROS1 与 ROS2 的输出逐值一致。
- 覆盖 187 点正常值、越界值、NaN、正负 Inf、全无效输入、两个开关的组合及非零替换值。
- 8 组默认配置与修改前 ROS1 实际源码的输出完全一致，4 组手算 NaN/Inf 示例通过。
- 检查去均值开关和 `sample_z_offset_`，并验证 17×11 的 `k=i*11+j` 采样顺序与 C 顺序展开一致。
- 两份 ROS1 源码和头文件分别逐字节一致；YAML 的地图话题、图层、187 点采样及待确认 TF 名称检查通过。

本次数值验证脚本、旧版源码快照和结果保存在 `/home/applepie/Documents/ChatGPT/go2w项目/stepit_ros1_validation/20261028/`。写回后的复现命令（需 Python NumPy、C++ 编译器和 Eigen）：

```bash
python '/home/applepie/Documents/ChatGPT/go2w项目/stepit_ros1_validation/20261028/verify_heightmap_preprocessing.py' \
  --ros1-root /home/applepie/project_for_test/go2w_cusrl_demo/stepit \
  --baseline-ros1-source '/home/applepie/Documents/ChatGPT/go2w项目/stepit_ros1_validation/20261028/legacy_baseline_source.cpp' \
  --work-dir /tmp/stepit-ros1-heightmap-check
```

本机 `/opt/ros` 当前只有 Jazzy，没有 Noetic；以上数值验证不等于完整 ROS1 插件编译或实机验证。部署前仍需在 ROS1 环境重新编译并加载更新后的插件，确认实际采样中心 TF、ROS 网络和 Unitree 网卡，并检查真实地图输入。该源码迁移不会使现有 Jazzy 构建产物自动变成 ROS1 程序。
