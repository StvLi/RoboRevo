# exp_env_interact 使用说明（ROS2 Foxy 目标）

本包将原 ROS1 三脚本迁移为 ROS2：`Haption6DExpert`（Virtuose 专家）、`SimplifiedFrankaEnv`（Franka）、`SimplifiedRviz2Env`（无机械臂 TF 调试），并由 `exp_env_interact_node` 以定时器方式串联 **get_action → step**。

## 1. 自定义消息放置位置

在 **`src/exp_env_interact_msgs/msg/`** 下维护或替换 `.msg` 文件（例如增删字段）。修改后：

```bash
cd <工作空间根目录>   # 含 src/ 的目录
source /opt/ros/foxy/setup.bash
# 若使用 conda：conda activate ros2_env
colcon build --symlink-install
source install/setup.bash
```

若你改用**不同包名**承载消息，请同步修改 Python 中的 `from xxx_msgs.msg import ...` 以及本包 `package.xml` / `CMakeLists.txt` 依赖。

当前内置消息（可按需替换）：

- `OutVirtuosePose.msg`：`geometry_msgs/Pose virtuose_pose`
- `OutVirtuoseStatus.msg`：`int32 buttons`

## 2. 构建

**重要（Foxy + 本机同时装有 Anaconda 时）**：请让 `colcon` 与 ROS 使用 **同一套 Python**（通常为系统 **Python 3.8**）。若 Conda 的 `python3` 在 PATH 最前，消息包可能装到 `lib/python3.12/site-packages`，运行时会与 `rclpy`（3.8）不兼容。推荐构建命令：

```bash
source /opt/ros/foxy/setup.bash
conda activate ros2_env   # 若适用；若与 Foxy 冲突可仅在非 Conda shell 中构建
export PATH=/usr/bin:/usr/local/bin:$PATH   # 保证 python3 为系统 3.8
cd /path/to/workspace     # 例如本仓库 interface_test
rm -rf build install log    # 若曾用错误 Python 编过，建议清理后重编
colcon build --symlink-install --packages-up-to exp_env_interact
source install/setup.bash
```

运行 `ros2 run` 时也建议使用相同的 PATH 策略，或始终在「已 source Foxy + install」的终端中启动。

依赖：`exp_env_interact_msgs`、`rclpy`、`geometry_msgs`、`tf2_ros`、`franka_msgs`（**仅在选择 Franka 后端时需要已安装该消息包**）、`python3-numpy`、`python3-scipy`。

## 3. 运行主节点

### 3.1 Franka 后端（默认）

需已发布 `franka_state_controller/franka_states`（`FrankaState`），并已订阅 `/cartesian_impedance_example_controller/equilibrium_pose`（`PoseStamped`）。Virtuose 侧需发布 `/out_virtuose_pose`、`/out_virtuose_status`（类型见消息包）。

```bash
source install/setup.bash
ros2 run exp_env_interact exp_env_interact_node --ros-args \
  -p backend:=franka \
  -p debug:=true \
  -p control_rate_hz:=1000.0
```

### 3.2 RViz2 调试后端（无 Franka）

不加载 `franka_msgs`，在内部积分目标位姿并向 TF 广播（默认 `world` → `ee_debug`）。

```bash
source install/setup.bash
ros2 run exp_env_interact exp_env_interact_node --ros-args \
  -p backend:=rviz2 \
  -p rviz2_parent_frame:=world \
  -p rviz2_child_frame:=ee_debug \
  -p control_rate_hz:=100.0
```

启动 **RViz2**：Fixed Frame 设为 `world`（或与 `rviz2_parent_frame` 一致），添加 **TF** 显示即可观察 `ee_debug` 随专家输出运动。

### 3.3 主要 ROS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `backend` | string | `franka` | `franka` 或 `rviz2` |
| `debug` | bool | `true` | 调试日志 |
| `control_rate_hz` | double | `1000.0` | 控制循环频率 |
| `rviz2_parent_frame` | string | `world` | RViz2 后端 TF 父坐标系 |
| `rviz2_child_frame` | string | `ee_debug` | RViz2 后端 TF 子坐标系 |

话题名与 ROS1 参考脚本保持一致（可通过 `ros2 run ... --ros-args -r old:=new` 做 remap）。

## 4. 单独测试子模块（可选）

在同一工作空间 `source install/setup.bash` 后：

```bash
python3 -m exp_env_interact.haption6d_expert
python3 -m exp_env_interact.simplified_rviz2_env
```

`simplified_franka_env` 模块在导入时需要 `franka_msgs`，仅建议在已安装 Franka 栈的环境中运行。

## 5. 与参考 ROS1 脚本的对应关系

| ROS1（dated_reference/scripits） | ROS2（本包） |
|----------------------------------|--------------|
| `haption6d_expert.py` | `exp_env_interact/haption6d_expert.py` |
| `simplified_franka_env.py` | `exp_env_interact/simplified_franka_env.py` |
| `hv6d_fr3_interface.py` | `exp_env_interact/exp_env_interact_node.py` |
| （无） | `exp_env_interact/simplified_rviz2_env.py` |

## 6. 踩坑与排障

见同目录 **`pitfalls.md`**。
