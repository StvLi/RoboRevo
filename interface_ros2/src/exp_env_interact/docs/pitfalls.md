# ROS2 迁移与联调踩坑记录（exp_env_interact）

本文记录在将 `dated_reference/scripits` 中 ROS1 逻辑迁移到本 ROS2 工作空间时遇到的典型问题与对应处理，便于后续同事复现与排障。

## 1. Conda / Anaconda 与 colcon：缺少 `pkg_resources`

- **现象**：`colcon build` 在配置 `exp_env_interact_msgs` 时失败，日志含 `ModuleNotFoundError: No module named 'pkg_resources'`，且 CMake 调用的 Python 指向 `.../anaconda3/bin/python3`。
- **原因**：ament 模板脚本依赖 `setuptools` 提供的 `pkg_resources`。若 Conda 里 **setuptools ≥ 81**，`pkg_resources` 可能被移除或不再默认可用，仍会触发同样报错。
- **处理（任选其一）**：
  - 在用于构建的 Conda 环境中：`pip install 'setuptools<81'`（或按团队策略固定到含 `pkg_resources` 的版本）；
  - 或让 CMake/colcon 使用系统 Python（如 `/usr/bin/python3`）构建，并保证该解释器已安装兼容的 `setuptools`；
  - 或执行 `pip install setuptools` 后仍失败时，检查 `python3 -c "import pkg_resources"` 是否成功。

## 2. 环境与 Shell：必须先加载 ROS2 再 colcon / ros2 run

- **现象**：终端里 `colcon: command not found` 或 `ros2: command not found`。
- **原因**：未 `source` ROS2 安装空间，PATH / AMENT_PREFIX_PATH 未配置。
- **处理**：在构建与运行前执行（Foxy 示例）：

```bash
source /opt/ros/foxy/setup.bash
```

若使用 Conda 中的独立环境（如 `ros2_env`），在 `source` 之后再 `conda activate ros2_env`，并确认该环境里已安装 `colcon` 与构建依赖（`python3-colcon-common-extensions` 等）。

## 3. Foxy 下读取参数：不要假设 `Parameter.value` 行为一致

- **现象**：在部分发行版上 `get_parameter('x').value` 不可用或类型不符合预期。
- **处理**：统一使用 `get_parameter(...).get_parameter_value()`，并按 `rcl_interfaces.msg.ParameterType` 区分 `bool` / `string` / `double` / `integer`。本仓库中 `control_rate_hz` 同时兼容整型与浮点 YAML。

## 4. 自定义消息：包名、路径与 Python 导入

- **消息存放路径**：`src/exp_env_interact_msgs/msg/`（在此目录增删 `.msg` 后需重新 `colcon build`）。
- **现象**：修改 `.msg` 后 Python 仍导入旧字段。
- **处理**：对工作空间执行 `source install/setup.bash`；若使用 `--symlink-install`，仍建议对消息包做一次干净构建（必要时 `rm -rf build install log` 后全量编译）。
- **与 ROS1 差异**：原 ROS1 脚本中若使用 `virtuose_pose.translation`，而标准 `geometry_msgs/Pose` 只有 `position`，则需在消息定义与解析代码两侧对齐。本包当前 `OutVirtuosePose` 使用 `geometry_msgs/Pose`，专家节点读取 `position` / `orientation`。

## 5. FrankaState 字段命名：ROS2 Python 为蛇形

- **现象**：`AttributeError: 'FrankaState' object has no attribute 'O_T_EE'`。
- **原因**：`rosidl_generator_py` 将 `.msg` 中的 `O_T_EE` 映射为 `o_t_ee`，`K_F_ext_hat_K` 映射为 `k_f_ext_hat_k`。
- **处理**：`simplified_franka_env.py` 已按蛇形字段访问；若你使用的 `franka_msgs` 版本消息定义不同（例如改为嵌套 `Pose`），需按实际 `.msg` 改解析逻辑。

## 6. 未安装 franka_msgs 时仍想跑通节点

- **现象**：`ModuleNotFoundError: franka_msgs`。
- **原因**：工作机未安装 Franka 的 ROS2 消息/驱动栈。
- **处理**：本节点对 Franka 环境采用**延迟导入**：选择参数 `backend:=rviz2` 时**不会**加载 `SimplifiedFrankaEnv`，可在无 Franka 条件下调试 Virtuose 专家 + TF。若必须使用 Franka 后端，请安装与你发行版匹配的 `franka_msgs`（或从源码编译 `franka_ros2` 等），并保证 `FrankaState` 话题与控制器话题与代码中默认名一致（或通过 launch/remap 对齐）。

## 7. 初始化阶段 `spin_once` 等不到 FrankaState

- **现象**：日志报「超时未收到 FrankaState」，`current_pose` 一直为 `None`，`step` 持续跳过。
- **原因**：控制器未起、话题名 remap 不一致、QoS 不兼容（如一方 `BEST_EFFORT` 一方 `RELIABLE`）、或仿真未发布状态。
- **处理**：用 `ros2 topic list` / `ros2 topic echo` 核对话题名；必要时将本包中订阅话题改为与现场一致的绝对名，或在 launch 里做 `-r` remap；必要时统一 QoS（本包默认 `RELIABLE` + depth 10）。

## 8. 高频定时器与 CPU 占用

- **现象**：默认 `control_rate_hz=1000` 时 CPU 占用偏高。
- **原因**：与 ROS1 脚本一致的高频控制循环。
- **处理**：调试时可传入较低频率，例如 `ros2 run ... --ros-args -p control_rate_hz:=100.0`。

## 9. RViz2 中看不到 `ee_debug`

- **现象**：TF 树无子坐标系。
- **原因**：Fixed Frame 与 `rviz2_parent_frame` 不一致，或 RViz 未勾选 TF 显示。
- **处理**：将 RViz **Fixed Frame** 设为与参数 `rviz2_parent_frame` 一致（默认 `world`）；在 Displays 中添加 **TF**，并确认节点在运行（`ros2 topic echo /tf` 或 `/tf_static` 视广播器而定；`TransformBroadcaster` 发布在 `/tf`）。

## 10. QoS 与「有订阅无数据」

- **现象**：`ros2 topic info` 可见发布者与订阅者，但回调不触发。
- **原因**：ROS2 中发布/订阅 QoS 策略不兼容会导致不匹配。
- **处理**：先用 `ros2 topic info -v` 查看双方 QoS；将 Virtuose 驱动端或本包订阅 QoS 调整为与对端一致（常见为传感器数据使用 `BEST_EFFORT`，控制命令使用 `RELIABLE`）。

## 11. scipy / numpy 缺失

- **现象**：`ModuleNotFoundError: No module named 'scipy'`。
- **原因**：运行用 Python 与构建用环境不一致，或系统未装 `python3-scipy`。
- **处理**：在运行 `ros2 run` 的同一环境中 `pip install scipy` 或通过 apt/rosdep 安装 `python3-scipy`，并保证 `which python3` 与 ROS2 使用的解释器一致。

## 12. `package.xml` 中 `<name>` 标签

- **说明**：本工作空间使用 **package format 3**，包名使用 `<name>包名</name>`。若你使用的工具链较老，请升级到支持 format 3 的 colcon/ament。

## 13. colcon 构建顺序

- **现象**：Python 包报找不到 `exp_env_interact_msgs`。
- **原因**：消息包未先于依赖它的包完成编译与 `source`。
- **处理**：在工作空间根目录执行 `colcon build`（默认按依赖排序）；首次构建后务必 `source install/setup.bash`。

## 14. 构建消息包时 `ModuleNotFoundError: No module named 'lark'`

- **现象**：编译 `exp_env_interact_msgs` 时，`rosidl_generator_c` / `rosidl_generator_cpp` 报错找不到 `lark`。
- **原因**：ROS Foxy 自带的 `rosidl_parser` 依赖 Python 包 `lark`；若终端里 `python3` 被 **Conda 抢先**（`which python3` 指向 Anaconda），而 Conda 环境未安装 `lark`，则生成器脚本会失败。仅给系统 `/usr/bin/python3.8` 安装 `lark` 可能仍不够。
- **处理**：在 **实际参与构建的** 那个 `python3` 上执行 `pip install lark`（例如 `$(which python3) -m pip install --user lark`）；或构建前临时 `export PATH=/usr/bin:$PATH`，使 `env python3` 解析为系统 Python。

## 15. 运行时报 `UnsupportedTypeSupport` / 消息装到 `python3.12` 而 rclpy 用 `python3.8`

- **现象**：`ros2 run` 时提示无法导入 `exp_env_interact_msgs` 的 `rosidl_typesupport_c`，或 `install/.../lib/python3.12/site-packages` 与 Foxy 的 `rclpy`（Python 3.8）不一致。
- **原因**：`colcon build` 若用 **Conda 的 Python 3.12** 生成/安装消息的 Python 扩展，运行时 ROS 仍用 **系统 Python 3.8** 加载 `rclpy`，版本不匹配。
- **处理**：构建与运行前建议统一使用 Foxy 对应的解释器，例如：
  - `export PATH=/usr/bin:/usr/local/bin:$PATH`（必要时 `unset CONDA_PREFIX`），再 `colcon build`；
  - 构建后确认 `install/exp_env_interact_msgs/share/.../hook/pythonpath.sh` 中为 `lib/python3.8/site-packages`。
- **运行**：`ros2 run` 时同样建议让 `python3` 与构建时一致，或始终在 `source /opt/ros/foxy/setup.bash` 与 `source install/setup.bash` 的同一类终端环境中启动。

## 16. `Rotation` 无 `as_matrix`（旧 scipy）

- **现象**：`AttributeError: 'Rotation' object has no attribute 'as_matrix'`。
- **原因**：`scipy.spatial.transform.Rotation.as_matrix()` 在 **scipy 1.4+** 才有；旧环境使用 `as_dcm()`。
- **处理**：本包已用 `exp_env_interact/_scipy_compat.py` 中 `rotation_3x3()` 兼容两种 API；若仍报错，请升级 `python3-scipy` 或检查是否混用了非 scipy 的 `Rotation` 类型。
