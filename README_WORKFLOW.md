# Go2W Sim2Sim Workflow

这份说明记录当前 Go2W 在 MuJoCo + StepIt 下的实际启动流程。

## 工作流概览

需要两个终端：

1. 终端 1 启动 `unitree_mujoco` 模拟器。
2. 终端 2 启动 `stepit` 控制策略。

建议顺序是：

1. 先启动模拟器。
2. 再启动 StepIt。

## 终端 1：启动 MuJoCo 模拟器

进入目录：

```bash
cd /home/applepie/project_for_test/unitree_mujoco/simulate/build
```

运行：

```bash
./unitree_mujoco
```

说明：

- 这个进程负责启动 Go2W 的 MuJoCo 仿真。
- 保持这个终端持续运行，不要关闭。

## 终端 2：启动 StepIt 策略

进入目录：

```bash
cd /home/applepie/project_for_test/go2w_cusrl_demo/stepit/stepit_ws
```

运行当前使用的策略：

```bash
./scripts/run.sh ./configs/go2w.sh -p /home/applepie/project_for_test/go2w_cusrl_demo/stepit/policy/go2w_policy_cusrl_mirlab
```

说明：

- `./configs/go2w.sh` 会加载 Go2W 相关环境，包括：
  - ROS 2 Jazzy 环境
  - `grid_map` 相关依赖
  - `STEPIT_NETIF=lo`
  - `STEPIT_DEFAULT_NNRTAPI=onnxruntime`
- `-p` 后面指定当前要运行的策略目录。

## 切换策略

如果要切换到其他策略，只需要替换 `-p` 后面的目录，例如：

```bash
./scripts/run.sh ./configs/go2w.sh -p /home/applepie/project_for_test/go2w_cusrl_demo/stepit/policy/<your_policy_dir>
```

## 当前默认使用示例

当前示例策略目录：

```bash
/home/applepie/project_for_test/go2w_cusrl_demo/stepit/policy/go2w_policy_cusrl_mirlab
```

## 常见注意事项

- 如果模拟器没有先启动，StepIt 可能无法正常接入仿真。
- 如果要换策略，先确认对应目录下至少包含：
  - `policy.yml`
  - `actor.yml`
  - `actor.onnx`
- 如果策略使用高程图输入，还需要确认对应的 `heightmap.yml`、`field_ops.yml` 等配置与模型匹配。
- 对 MIRLab 训练出的策略，`heightmap.yml` 需要保持：
  - `dimension: [17, 11]`
  - `grid_size: [0.1, 0.1]`
  - `x_major: true`
  - `elevation_clip_min: -5.0`
  - `elevation_clip_max: 5.0`
  - `elevation_nan_to_num: true`
  - `elevation_nan_value: 0.0`
  - `elevation_zero_mean: true`
- `field_ops.yml` 不要额外给 `heightmap` 加 `scale` 或 `bias`，这样才对应 MIRLab 中 `(height_scan - height_scan.mean()) * obs_scales.height_scan` 的处理。

## 最简操作版

终端 1：

```bash
cd /home/applepie/project_for_test/unitree_mujoco/simulate/build
./unitree_mujoco
```

终端 2：

```bash
cd /home/applepie/project_for_test/go2w_cusrl_demo/stepit/stepit_ws
./scripts/run.sh ./configs/go2w.sh -p /home/applepie/project_for_test/go2w_cusrl_demo/stepit/policy/go2w_policy_cusrl_mirlab
```
