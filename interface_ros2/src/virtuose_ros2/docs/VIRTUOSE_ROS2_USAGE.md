# virtuose_ros2 使用说明

ROS2（ament/colcon）版 Virtuose 节点，行为对齐 ROS1 `virtuose` + `test_*` 包，中间件为 ROS2。

## 环境要求

- ROS 2（开发与验证环境示例：Foxy；其他发行版需自行确认接口兼容）
- `colcon`、`ament_cmake`
- Haption Virtuose SDK：`libvirtuose.a` 与同目录下的 **`virtuoseAPI.so`**（运行时由 `libvirtuose` 通过 `dlopen` 加载，缺一不可）

## 获取与配置 SDK

- **推荐**：若本机已有可正常运行的 catkin 包  
  `~/Desktop/haption_test/haption_ros1_ws/src/ROS-Virtuose/src/virtuose/external/VirtuoseAPI`  
  则 **首次 CMake 配置**时，`virtuose_ros2` 会默认把 `VIRTUOSE_SDK_ROOT` 指到该目录，与 `rosrun virtuose virtuose_node` 使用同一套库。
- **备选**：将完整 SDK（含 `VirtuoseAPI.h`、`libvirtuose.a`、`virtuoseAPI.so` 等）放到本包  
  `virtuose_ros2/external/VirtuoseAPI/`。
- **手动覆盖**：  
  `colcon build --packages-select virtuose_ros2 --cmake-args -DVIRTUOSE_SDK_ROOT=/你的/VirtuoseAPI路径`

## 编译

```bash
cd ~/Desktop/haption_test/haption_ros2_ws
source /opt/ros/foxy/setup.bash
# 若使用 conda 且 colcon 报 pkg_resources 等错误，请指定系统 Python：
PYTHON_EXECUTABLE=/usr/bin/python3 colcon build --packages-select virtuose_ros2 \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 运行前

```bash
source /opt/ros/foxy/setup.bash
source ~/Desktop/haption_test/haption_ros2_ws/install/setup.bash
```

**所有相关终端必须使用相同的 `ROS_DOMAIN_ID`**（未设置则均为 0）。不一致时会出现“服务不存在”或发现极慢。

## 与 ROS1 对照的三终端用法

| ROS 1 | ROS 2 |
|--------|--------|
| 终端 A：`roscore` | 不需要独立 roscore |
| 终端 B：`rosrun virtuose virtuose_node` | `ros2 run virtuose_ros2 virtuose_node` |
| 终端 C：`rosrun test_admittance test_admittance_node` | `ros2 run virtuose_ros2 test_admittance_node` |

示例：

```bash
# 终端 A
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 run virtuose_ros2 virtuose_node

# 终端 B
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 run virtuose_ros2 test_admittance_node
```

同目录下还有：

- `test_calibration_node`：标定流程演示  
- `test_impedance_node`：阻抗模式演示（默认 IP 与导纳测试一致，可按设备修改源码中的 `ip_address`）

## 设备地址格式

与 ROS1 / Virtuose API 一致，使用字符串：

```text
<IP>#<端口>
```

例如本地仿真或转发：`127.0.0.1#5001`。

## 主要接口概览

### 话题（与 ROS1 同名）

- 发布：`out_virtuose_status`、`out_virtuose_pose`、`out_virtuose_physical_pose`、`out_virtuose_speed`、`out_virtuose_force`
- 订阅：`in_virtuose_pose`、`in_virtuose_speed`、`in_virtuose_force`

### 服务

- `virtuose_reset`
- `virtuose_calibrate`
- `virtuose_admittance`
- `virtuose_impedance`

## ROS2 与 ROS1 消息差异（导纳）

`VirtuoseAdmittance` 请求中的刚度/阻尼在 ROS2 中为 **小写字段**：

| ROS 1 | ROS 2 |
|--------|--------|
| `Ktrans`, `Btrans`, `Krot`, `Brot` | `ktrans`, `btrans`, `krot`, `brot` |

## 进一步阅读

- 构建与运行中的常见错误与对策见同目录 **`VIRTUOSE_ROS2_PITFALLS.md`**。
