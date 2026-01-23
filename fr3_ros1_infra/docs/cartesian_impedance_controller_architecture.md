# Cartesian Impedance Example Controller 架构文档

本文档详细描述了 `cartesian_impedance_example_controller.launch` 启动后的ROS系统架构，包括节点、话题、服务和Action的完整关系。

**生成时间**: 2026-01-22  
**Launch文件**: `franka_example_controllers/cartesian_impedance_example_controller.launch`  
**机器人配置**: FR3 (robot:=fr3, robot_ip:=172.16.0.2, load_gripper:=false)

---

## 1. 节点列表

系统启动后包含以下节点：

| 节点名称 | 包名 | 功能描述 |
|---------|------|---------|
| `/franka_control` | `franka_control` | Franka机器人控制节点，负责与机器人硬件通信 |
| `/state_controller_spawner` | `controller_manager` | 启动并管理 `franka_state_controller` |
| `/controller_spawner` | `controller_manager` | 启动并管理 `cartesian_impedance_example_controller` |
| `/robot_state_publisher` | `robot_state_publisher` | 发布机器人TF变换 |
| `/joint_state_publisher` | `joint_state_publisher` | 合并并发布关节状态 |
| `/interactive_marker` | `franka_example_controllers` | 提供交互式标记，用于设置平衡姿态 |
| `/rviz` | `rviz` | 3D可视化工具 |
| `/rqt_reconfigure` | `rqt_reconfigure` | 动态参数配置工具 |

---

## 2. 话题 (Topics) - 发布/订阅关系

### 2.1 控制相关话题

#### `/cartesian_impedance_example_controller/equilibrium_pose`
- **类型**: `geometry_msgs/PoseStamped`
- **发布者**: `/interactive_marker`
- **订阅者**: `/franka_control`
- **描述**: 笛卡尔阻抗控制器的平衡姿态目标，由交互式标记节点发布

#### `/franka_state_controller/franka_states`
- **类型**: `franka_msgs/FrankaState`
- **发布者**: `/franka_control`
- **订阅者**: (可能被其他节点订阅)
- **描述**: Franka机器人的完整状态信息

#### `/franka_state_controller/joint_states`
- **类型**: `sensor_msgs/JointState`
- **发布者**: `/franka_control`
- **订阅者**: `/joint_state_publisher`
- **描述**: 机器人关节状态（位置、速度、力矩）

#### `/franka_state_controller/joint_states_desired`
- **类型**: `sensor_msgs/JointState`
- **发布者**: `/franka_control`
- **订阅者**: (可能被可视化工具订阅)
- **描述**: 期望的关节状态

#### `/franka_state_controller/F_ext`
- **类型**: `geometry_msgs/WrenchStamped`
- **发布者**: `/franka_control`
- **订阅者**: (可能被其他节点订阅)
- **描述**: 外部作用力/力矩

### 2.2 TF变换话题

#### `/tf`
- **类型**: `tf2_msgs/TFMessage`
- **发布者**: 
  - `/robot_state_publisher` (机器人模型TF)
  - `/franka_control` (可能发布额外TF)
- **订阅者**: 
  - `/interactive_marker`
  - `/rviz`
- **描述**: 动态TF变换

#### `/tf_static`
- **类型**: `tf2_msgs/TFMessage`
- **发布者**: `/robot_state_publisher`
- **订阅者**: 
  - `/interactive_marker`
  - `/rviz`
- **描述**: 静态TF变换（机器人模型结构）

### 2.3 合并后的关节状态

#### `/joint_states`
- **类型**: `sensor_msgs/JointState`
- **发布者**: `/joint_state_publisher`
- **订阅者**: `/robot_state_publisher`
- **描述**: 合并后的关节状态（当没有gripper时，只包含机械臂关节）

### 2.4 可视化相关话题

#### `/equilibrium_pose_marker/update`
- **类型**: `visualization_msgs/InteractiveMarkerUpdate`
- **发布者**: `/interactive_marker`
- **订阅者**: `/rviz`
- **描述**: 交互式标记更新

#### `/equilibrium_pose_marker/update_full`
- **类型**: `visualization_msgs/InteractiveMarkerInit`
- **发布者**: `/interactive_marker`
- **订阅者**: `/rviz`
- **描述**: 交互式标记完整初始化

#### `/equilibrium_pose_marker/feedback`
- **类型**: `visualization_msgs/InteractiveMarkerFeedback`
- **发布者**: `/rviz`
- **订阅者**: `/interactive_marker`
- **描述**: 用户在RViz中操作交互式标记的反馈

#### `/clicked_point`
- **类型**: `geometry_msgs/PointStamped`
- **发布者**: `/rviz`
- **订阅者**: (可能被其他节点订阅)
- **描述**: 用户在RViz中点击的点

### 2.5 动态参数配置话题

#### `/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/parameter_descriptions`
- **类型**: `dynamic_reconfigure/ConfigDescription`
- **发布者**: `/franka_control`
- **订阅者**: (动态参数配置系统)
- **描述**: 参数描述

#### `/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/parameter_updates`
- **类型**: `dynamic_reconfigure/Config`
- **发布者**: `/franka_control`
- **订阅者**: (动态参数配置系统)
- **描述**: 参数更新

### 2.6 Action话题 (Error Recovery)

#### `/franka_control/error_recovery/goal`
- **类型**: `franka_msgs/ErrorRecoveryActionGoal`
- **发布者**: (Action客户端)
- **订阅者**: `/franka_control`
- **描述**: 错误恢复Action的目标

#### `/franka_control/error_recovery/feedback`
- **类型**: `franka_msgs/ErrorRecoveryActionFeedback`
- **发布者**: `/franka_control`
- **订阅者**: (Action客户端)
- **描述**: 错误恢复Action的反馈

#### `/franka_control/error_recovery/result`
- **类型**: `franka_msgs/ErrorRecoveryActionResult`
- **发布者**: `/franka_control`
- **订阅者**: (Action客户端)
- **描述**: 错误恢复Action的结果

#### `/franka_control/error_recovery/status`
- **类型**: `actionlib_msgs/GoalStatusArray`
- **发布者**: `/franka_control`
- **订阅者**: (Action客户端)
- **描述**: 错误恢复Action的状态

#### `/franka_control/error_recovery/cancel`
- **类型**: `actionlib_msgs/GoalID`
- **发布者**: (Action客户端)
- **订阅者**: `/franka_control`
- **描述**: 取消错误恢复Action

### 2.7 系统话题

#### `/rosout`
- **类型**: `rosgraph_msgs/Log`
- **发布者**: 所有节点
- **订阅者**: `/rosout` 节点
- **描述**: ROS日志消息

#### `/rosout_agg`
- **类型**: `rosgraph_msgs/Log`
- **发布者**: `/rosout` 节点
- **订阅者**: (日志聚合工具)
- **描述**: 聚合的ROS日志

---

## 3. 服务 (Services) - 服务器/客户端关系

### 3.1 控制器管理服务

所有控制器管理服务由 `/franka_control` 节点提供（通过controller_manager）：

#### `/controller_manager/list_controllers`
- **类型**: `controller_manager_msgs/ListControllers`
- **服务器**: `/franka_control`
- **客户端**: `/controller_spawner`, `/state_controller_spawner`, 其他管理工具
- **描述**: 列出所有已加载的控制器

#### `/controller_manager/list_controller_types`
- **类型**: `controller_manager_msgs/ListControllerTypes`
- **服务器**: `/franka_control`
- **客户端**: 管理工具
- **描述**: 列出可用的控制器类型

#### `/controller_manager/load_controller`
- **类型**: `controller_manager_msgs/LoadController`
- **服务器**: `/franka_control`
- **客户端**: `/controller_spawner`, `/state_controller_spawner`
- **描述**: 加载指定的控制器

#### `/controller_manager/unload_controller`
- **类型**: `controller_manager_msgs/UnloadController`
- **服务器**: `/franka_control`
- **客户端**: `/controller_spawner`, `/state_controller_spawner`
- **描述**: 卸载指定的控制器

#### `/controller_manager/switch_controller`
- **类型**: `controller_manager_msgs/SwitchController`
- **服务器**: `/franka_control`
- **客户端**: `/controller_spawner`, `/state_controller_spawner`
- **描述**: 切换控制器的启动/停止状态

#### `/controller_manager/reload_controller_libraries`
- **类型**: `controller_manager_msgs/ReloadControllerLibraries`
- **服务器**: `/franka_control`
- **客户端**: 管理工具
- **描述**: 重新加载控制器库

### 3.2 Franka控制服务

所有Franka控制服务由 `/franka_control` 节点提供：

#### `/franka_control/connect`
- **类型**: `std_srvs/Trigger`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 连接到Franka机器人

#### `/franka_control/disconnect`
- **类型**: `std_srvs/Trigger`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 断开与Franka机器人的连接

#### `/franka_control/set_cartesian_impedance`
- **类型**: `franka_msgs/SetCartesianImpedance`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 设置笛卡尔阻抗参数

#### `/franka_control/set_joint_impedance`
- **类型**: `franka_msgs/SetJointImpedance`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 设置关节阻抗参数

#### `/franka_control/set_EE_frame`
- **类型**: `franka_msgs/SetEEFrame`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 设置末端执行器坐标系

#### `/franka_control/set_K_frame`
- **类型**: `franka_msgs/SetKFrame`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 设置K坐标系

#### `/franka_control/set_force_torque_collision_behavior`
- **类型**: `franka_msgs/SetForceTorqueCollisionBehavior`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 设置力/力矩碰撞行为

#### `/franka_control/set_full_collision_behavior`
- **类型**: `franka_msgs/SetFullCollisionBehavior`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 设置完整碰撞行为

#### `/franka_control/set_load`
- **类型**: `franka_msgs/SetLoad`
- **服务器**: `/franka_control`
- **客户端**: 外部工具
- **描述**: 设置负载参数

### 3.3 动态参数配置服务

#### `/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters`
- **类型**: `dynamic_reconfigure/Reconfigure`
- **服务器**: `/franka_control`
- **客户端**: `/rqt_reconfigure`
- **描述**: 动态设置笛卡尔阻抗控制器的合规性参数

### 3.4 日志管理服务

每个节点都提供以下日志管理服务：
- `/<node_name>/get_loggers`: `roscpp/GetLoggers`
- `/<node_name>/set_logger_level`: `roscpp/SetLoggerLevel`

### 3.5 RViz配置服务

#### `/rviz/load_config`
- **类型**: `rviz/LoadConfig`
- **服务器**: `/rviz`
- **客户端**: 外部工具
- **描述**: 加载RViz配置

#### `/rviz/save_config`
- **类型**: `rviz/SaveConfig`
- **服务器**: `/rviz`
- **客户端**: 外部工具
- **描述**: 保存RViz配置

#### `/rviz/load_config_discarding_changes`
- **类型**: `rviz/LoadConfig`
- **服务器**: `/rviz`
- **客户端**: 外部工具
- **描述**: 加载RViz配置（丢弃当前更改）

#### `/rviz/reload_shaders`
- **类型**: `std_srvs/Empty`
- **服务器**: `/rviz`
- **客户端**: 外部工具
- **描述**: 重新加载着色器

---

## 4. 数据流图

### 4.1 控制数据流

```
用户交互 (RViz)
    ↓
/interactive_marker (发布)
    ↓
/cartesian_impedance_example_controller/equilibrium_pose
    ↓
/franka_control (订阅并处理)
    ↓
硬件通信 (172.16.0.2)
    ↓
/franka_state_controller/joint_states
    ↓
/joint_state_publisher (合并)
    ↓
/joint_states
    ↓
/robot_state_publisher (计算TF)
    ↓
/tf, /tf_static
    ↓
/rviz, /interactive_marker (可视化)
```

### 4.2 状态反馈数据流

```
/franka_control (硬件读取)
    ↓
/franka_state_controller/franka_states
/franka_state_controller/joint_states
/franka_state_controller/F_ext
    ↓
/joint_state_publisher (订阅joint_states)
    ↓
/joint_states
    ↓
/robot_state_publisher (计算TF)
    ↓
/tf, /tf_static
    ↓
可视化节点
```

### 4.3 交互式标记数据流

```
/rviz (用户操作)
    ↓
/equilibrium_pose_marker/feedback
    ↓
/interactive_marker (处理反馈)
    ↓
/cartesian_impedance_example_controller/equilibrium_pose
    ↓
/franka_control (执行控制)
```

---

## 5. 关键节点详细说明

### 5.1 `/franka_control`

**功能**: Franka机器人控制核心节点

**发布的话题**:
- `/franka_state_controller/franka_states` - 机器人状态
- `/franka_state_controller/joint_states` - 关节状态
- `/franka_state_controller/joint_states_desired` - 期望关节状态
- `/franka_state_controller/F_ext` - 外部力/力矩
- `/tf` - TF变换
- `/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/*` - 动态参数

**订阅的话题**:
- `/cartesian_impedance_example_controller/equilibrium_pose` - 平衡姿态目标
- `/franka_control/error_recovery/goal` - 错误恢复目标
- `/franka_control/error_recovery/cancel` - 取消错误恢复

**提供的服务**:
- 所有控制器管理服务
- 所有Franka配置服务
- 动态参数配置服务

**Action服务器**:
- `/franka_control/error_recovery` - 错误恢复Action

### 5.2 `/interactive_marker`

**功能**: 提供交互式标记，允许用户在RViz中拖动设置平衡姿态

**发布的话题**:
- `/cartesian_impedance_example_controller/equilibrium_pose` - 平衡姿态
- `/equilibrium_pose_marker/update` - 标记更新
- `/equilibrium_pose_marker/update_full` - 完整标记初始化

**订阅的话题**:
- `/equilibrium_pose_marker/feedback` - 用户操作反馈
- `/tf` - 获取机器人TF信息
- `/tf_static` - 获取静态TF信息

### 5.3 `/joint_state_publisher`

**功能**: 合并来自不同源的关节状态

**发布的话题**:
- `/joint_states` - 合并后的关节状态

**订阅的话题**:
- `/franka_state_controller/joint_states` - 机械臂关节状态
- (当load_gripper=true时，还会订阅 `/franka_gripper/joint_states`)

### 5.4 `/robot_state_publisher`

**功能**: 根据关节状态和URDF模型发布TF变换

**发布的话题**:
- `/tf` - 动态TF变换
- `/tf_static` - 静态TF变换

**订阅的话题**:
- `/joint_states` - 关节状态

---

## 6. 控制器说明

### 6.1 `franka_state_controller`

**类型**: `franka_control/FrankaStateController`

**功能**: 
- 读取并发布Franka机器人的状态信息
- 发布关节状态、力/力矩等信息

**管理节点**: `/state_controller_spawner`

### 6.2 `cartesian_impedance_example_controller`

**类型**: `franka_example_controllers/CartesianImpedanceExampleController`

**功能**:
- 实现笛卡尔阻抗控制
- 接收平衡姿态目标 (`/cartesian_impedance_example_controller/equilibrium_pose`)
- 根据当前状态和目标姿态计算控制输出

**管理节点**: `/controller_spawner`

**参数配置**:
- 可通过 `/cartesian_impedance_example_controller/dynamic_reconfigure_compliance_param_node/set_parameters` 动态调整
- 可通过 `/rqt_reconfigure` 图形界面调整

---

## 7. 使用建议

### 7.1 启动系统

清理进程
`pkill -9 -f "roslaunch|rosmaster|franka|rviz|rqt|controller|robot_state|joint_state|interactive" 2>/dev/null; sleep 2; echo "进程已清理"`

启动程序
`source /home/alan/miniconda3/etc/profile.d/conda.sh && conda activate fr3_ros1_infra && nohup bash -c "source /opt/ros/noetic/setup.bash && source /home/alan/fr3_ros1_infra/devel/setup.bash && roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot:=fr3 robot_ip:=172.16.0.2 load_gripper:=false" > /tmp/franka_launch.log 2>&1 &`

如果机械臂状态仍未正常显示，请检查：

1. **检查franka_control节点是否正常运行**:
   ```bash
   rosnode info /franka_control
   # 应该看到正在发布 /franka_state_controller/joint_states
   ```

2. **检查joint_state_publisher是否订阅到数据**:
   ```bash
   rostopic info /joint_states
   # 应该看到 /joint_state_publisher 作为发布者
   ```

3. **检查robot_state_publisher是否收到joint_states**:
   ```bash
   rosnode info /robot_state_publisher
   # 应该看到订阅了 /joint_states
   ```

4. **检查TF树是否完整**:
   ```bash
   rosrun tf tf_monitor
   # 应该看到从 fr3_link0 到 fr3_link8 的完整链
   ```

5. **查看启动日志**:
   ```bash
   # 检查是否有错误信息
   tail -100 ~/.ros/log/latest/roslaunch-*.log
   ```

### 7.2 控制机器人

1. **通过RViz交互式标记**: 在RViz中拖动标记设置平衡姿态
2. **通过话题发布**: 直接向 `/cartesian_impedance_example_controller/equilibrium_pose` 发布 `geometry_msgs/PoseStamped` 消息
3. **通过服务调用**: 使用 `/franka_control/set_cartesian_impedance` 设置阻抗参数

### 7.3 监控状态

- 订阅 `/franka_state_controller/franka_states` 获取完整状态
- 订阅 `/franka_state_controller/joint_states` 获取关节状态
- 订阅 `/franka_state_controller/F_ext` 获取外部力/力矩

### 7.4 错误恢复

如果机器人进入错误状态，可以通过Action调用错误恢复：

```bash
rostopic pub /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"
```

---

## 8. 注意事项

1. **实时调度**: 当前配置中 `realtime_config: ignore`，如需实时性能，需要配置实时调度权限
2. **网络连接**: 确保能够ping通机器人IP (172.16.0.2)
3. **控制器状态**: 使用 `/controller_manager/list_controllers` 检查控制器是否正常运行
4. **TF树**: 确保 `/tf` 和 `/tf_static` 正常发布，否则可视化会出现问题

---

## 9. 相关文档

- Franka ROS文档: https://frankaemika.github.io/docs/
- ROS Control文档: http://wiki.ros.org/ros_control
- Cartesian Impedance Controller源码: `src/franka_ros/franka_example_controllers/src/cartesian_impedance_example_controller.cpp`

---

**文档版本**: 1.0  
**最后更新**: 2026-01-22

