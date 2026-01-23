# InferenceClient 设计讲解

## 设计目标
创建一个与 `haption6d_expert.py` 结构兼容的推理客户端，能够通过 `get_action()` 方法生成一步动作给环境执行。

## 核心设计原则

### 1. 接口兼容性
- **相同的方法签名**：`get_action(obs)` 返回 `(action_vector, button_state)`
- **相似的状态管理**：使用 `current_action`, `current_buttons`, `fsm` 等状态变量
- **一致的错误处理**：graceful degradation 和 fallback 机制

### 2. 服务器交互模式
基于 `infer_umi.py` 的分析，保留以下核心交互：
- **HTTP REST API**：POST 请求到 `/predict_action` 端点
- **Base64图像编码**：JPEG压缩 + Base64编码，减少数据传输量
- **Receding Horizon策略**：一次推理获取多步动作序列，分步执行

### 3. 状态机设计
```
状态转移：
idle → inference: 达到时间间隔且图像源可用
inference → execution: 推理成功完成
execution → idle: 动作缓冲区耗尽

状态行为：
- idle: 生成零动作
- inference: 向服务器请求动作序列
- execution: 从缓冲区提取动作执行
```

## 关键类结构

### InferenceClient 类
```python
class InferenceClient:
    def __init__(self, debug=True, use_mock=False):
        # 配置管理
        self.config = config_manager
        self.server_ip = "127.0.0.1"
        self.server_port = 5003
        
        # 状态管理（仿照 Haption6DExpert）
        self.fsm = "idle"
        self.current_action = np.zeros(6)
        self.current_buttons = np.array([False])
        
        # 推理状态
        self.action_buffer = []  # 服务器返回的动作序列
        self.buffer_index = 0
        
        # 外部依赖
        self.image_sources = None  # 图像获取函数
        self.current_robot_state = None  # 机器人状态
```

### 核心方法
1. **`get_action(obs)`**：主接口方法，返回动作和按钮状态
2. **`update_fsm()`**：状态机更新逻辑
3. **`perform_inference()`**：与服务器通信获取动作
4. **`update_action_from_buffer()`**：从缓冲区提取下一个动作

## 动作转换流程

### 服务器动作 → 机器人动作
```
服务器格式 (7D):
[delta_x_local, delta_y_local, delta_z_local, rot_x, rot_y, rot_z, gripper_prob]

转换步骤：
1. 位置增量：Local Frame → World Frame（使用当前姿态旋转矩阵）
2. 旋转增量：直接使用（服务器返回的是旋转向量）
3. 夹爪控制：忽略或单独处理

机器人格式 (6D):
[dx_world, dy_world, dz_world, drx, dry, drz]
```

## 安全特性

### 1. 动作限制
- **最大位置步长**：限制单步位移（默认 0.05m）
- **最大旋转步长**：限制单步旋转（默认 0.1rad）
- **边界检查**：防止过大动作

### 2. 错误恢复
- **Mock模式**：服务器不可用时生成随机小动作
- **超时处理**：HTTP请求超时自动降级
- **连接重试**：网络错误时自动恢复

### 3. 性能监控
- **统计信息**：推理次数、动作次数、错误次数
- **时序记录**：推理耗时监控
- **日志分级**：debug/info/warn/error 分级日志

## 集成方式

### 与现有框架集成
```python
# 创建实例
inference_client = InferenceClient(debug=True, use_mock=False)

# 设置图像源（需要外部提供）
inference_client.set_image_sources(get_images_function)

# 在时钟回调中使用
action, buttons = inference_client.get_action(obs)
env.step(action, buttons)
```

### 配置管理
通过 ROS 参数服务器配置：
```yaml
inference:
  server_ip: "172.16.17.208"
  server_port: 5003
  img_width: 640
  img_height: 480
  exec_steps: 5
  interval: 0.2
  
safety:
  max_pos_step: 0.05
  max_rot_step: 0.1
```

## 优势与特点

### 1. 兼容性优势
- **即插即用**：可直接替换 Haption6DExpert
- **统一接口**：相同的 get_action() 方法
- **状态机一致**：相似的 FSM 设计

### 2. 功能性优势
- **多步预测**：Receding Horizon 策略提高效率
- **安全可靠**：多重安全检查和限制
- **易于调试**：详细的日志和统计信息

### 3. 扩展性优势
- **配置灵活**：ROS 参数动态配置
- **模式切换**：Mock/Real 模式运行时切换
- **易于测试**：无需真实服务器即可测试

## 使用示例
```python
# 创建推理客户端
client = InferenceClient(use_mock=True)  # 测试时使用 Mock 模式

# 设置图像源（模拟三视角）
def mock_image_source():
    return [np.random.rand(480, 640, 3) for _ in range(3)]
client.set_image_sources(mock_image_source)

# 在控制循环中使用
while not rospy.is_shutdown():
    action, buttons = client.get_action()
    # 执行动作...
    rate.sleep()
```

这个设计保持了与现有遥操作框架的兼容性，同时提供了强大的推理功能和安全保障。
