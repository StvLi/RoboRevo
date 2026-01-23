# 推理客户端服务器交互分析

基于 `tmp/reference_scripts/dated_infer/infer_umi.py` 的分析

## 核心交互流程

### 1. 请求阶段 (客户端 → 服务器)

**数据准备：**
- **图像数据**：获取多视角图像（Mid, Left, Right），转换为RGB格式，调整尺寸，编码为Base64 JPEG字符串
- **机器人状态**：获取当前末端执行器位姿 [x, y, z, qx, qy, qz, qw, gripper_width]
- **任务指令**：自然语言描述的任务指令（如 "Pick up the banana"）

**请求Payload结构：**
```json
{
  "examples": [
    {
      "image": ["base64_img1", "base64_img2", "base64_img3"],
      "lang": "Pick up the banana",
      "state": null  // 或 [x, y, z, qx, qy, qz, qw, gripper_width]
    }
  ]
}
```

**HTTP请求：**
- 方法：POST
- URL：`http://{SERVER_IP}:{SERVER_PORT}/predict_action`
- 超时：30秒
- Content-Type：application/json

### 2. 响应阶段 (服务器 → 客户端)

**响应数据结构：**
```json
{
  "data": {
    "unnormalized_actions": [
      // 形状: [Batch=1, Chunk=30, Dim=7]
      // Batch: 批次大小
      // Chunk: 预测的动作序列长度（通常30步）
      // Dim: 动作维度（7维）
      [
        [delta_x, delta_y, delta_z, rot_x, rot_y, rot_z, gripper], // 第1步
        [delta_x, delta_y, delta_z, rot_x, rot_y, rot_z, gripper], // 第2步
        ... // 共30步
      ]
    ]
  }
}
```

**动作格式说明：**
- **位置增量**：前3维，相对于当前末端执行器坐标系（Local Frame）的位移，单位：米
- **旋转增量**：中间3维，旋转向量（轴角表示），单位：弧度
- **夹爪控制**：最后1维，夹爪开合概率（0-1之间）

### 3. 动作执行阶段

**动作解析流程：**
1. **提取当前状态**：从机器人获取当前位姿
2. **坐标转换**：将Local Frame下的位置增量转换到World Frame
   - 使用当前姿态的旋转矩阵进行坐标变换
3. **姿态叠加**：将旋转增量叠加到当前姿态
   - 使用四元数乘法组合旋转
4. **夹爪映射**：将概率值映射为实际宽度
   - 阈值0.5：>0.5张开，≤0.5闭合

**执行策略：**
- **Receding Horizon**：一次推理获取30步动作序列
- **分步执行**：只执行前`EXEC_STEPS`步（默认5步）
- **链式计算**：每一步的目标位姿基于上一步的虚拟终点计算

## 关键设计模式

### 1. 数据压缩优化
- **图像压缩**：原始图像（~14MB）→ JPEG压缩（~300KB）→ Base64编码
- **状态可选**：`state`字段可为`null`，服务端自动补全为0向量

### 2. 错误处理机制
- **硬件降级**：硬件连接失败时自动切换到Mock模式
- **请求重试**：HTTP请求失败时跳过本次循环，继续尝试
- **安全检查**：单步位移过大时发出警告

### 3. 实时性保障
- **固定频率**：使用`time.sleep`维持指定Hz（默认5Hz）
- **超时控制**：图像获取和HTTP请求都有超时限制
- **异步执行**：推理和执行分离，避免阻塞

## 重构建议

### 需要保留的核心交互模式：
1. **HTTP REST API接口**：保持与现有服务器的兼容性
2. **Base64图像编码**：高效的数据传输方式
3. **动作解析逻辑**：Local Frame到World Frame的坐标转换
4. **Receding Horizon策略**：一次推理，分步执行

### 需要改进的方面：
1. **模块化设计**：将服务器交互、动作解析、硬件控制分离
2. **配置管理**：使用ROS参数服务器或配置文件
3. **错误恢复**：更完善的错误处理和重试机制
4. **状态管理**：更好的机器人状态跟踪和同步
5. **测试支持**：Mock模式和单元测试

### 需要重构的机器人接口：
- 参考脚本中的`api_eef_gripper`模块需要替换为更合理的ROS接口
- 使用标准的ROS消息和服务进行机器人控制
- 集成到`teleop_infer_infra`包的架构中

## 代码注释要点

在重构代码中需要详细注释：
1. **服务器交互协议**：请求/响应格式，字段含义
2. **坐标转换逻辑**：Local Frame与World Frame的转换关系
3. **动作执行策略**：Receding Horizon和分步执行原理
4. **错误处理机制**：各种异常情况的处理方式
5. **性能优化**：数据压缩和实时性保障措施
