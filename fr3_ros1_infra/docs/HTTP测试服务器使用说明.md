# HTTP测试服务器使用说明

## 概述

HTTP测试服务器是一个模拟VLA模型服务器的测试工具，用于验证InferenceClient的HTTP通信功能。它返回固定的动作轨迹，不依赖图像输入，专注于测试通信协议。

## 服务器特性

- ✅ 与真实VLA模型服务器接口完全兼容
- ✅ 返回固定的方形动作轨迹（25个动作）
- ✅ 不依赖图像输入，专注于测试HTTP通信
- ✅ 部署在本机，便于测试
- ✅ 提供健康检查、服务器信息等端点

## 文件位置

- 服务器脚本：`.test/cline_selfbuild_test/test_http_server.py`
- 客户端代码：`src/teleop_infer_infra/src/teleop_infer_infra/inference.py`

## 客户端启动指令（重要！）

**推理客户端标准启动命令**：
```bash
conda run -n fr3_ros1_infra bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source devel/setup.bash && \
   roslaunch teleop_infer_infra cartesian_impedance_with_inference.launch \
   robot:=fr3 robot_ip:=172.16.0.2 load_gripper:=false use_mock:=false"
```

**关键参数说明**：
- `use_mock:=false` - **必须设置为false**才能使用HTTP通信
- `robot:=fr3` - 机器人型号
- `robot_ip:=172.16.0.2` - Franka机器人IP地址
- `load_gripper:=false` - 不加载夹爪
- `debug:=true` - 可选的调试模式（默认启用）

## 快速开始

### 1. 启动测试服务器

```bash
cd .test/cline_selfbuild_test
python test_http_server.py --host 127.0.0.1 --port 5003
```

### 2. 验证服务器运行

```bash
# 健康检查
curl http://127.0.0.1:5003/health

# 服务器信息
curl http://127.0.0.1:5003/info

# 测试推理请求
curl -X POST http://127.0.0.1:5003/predict_action \
  -H 'Content-Type: application/json' \
  -d '{"examples":[{"image":null,"lang":"test instruction","state":null}]}'
```

### 3. 启动真机控制程序

```bash
conda run -n fr3_ros1_infra bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source devel/setup.bash && \
   roslaunch teleop_infer_infra cartesian_impedance_with_inference.launch \
   robot:=fr3 robot_ip:=172.16.0.2 load_gripper:=false use_mock:=false"
```

**关键参数**：`use_mock:=false`（必须设置为false才能使用HTTP通信）

## 服务器端点

### 1. 健康检查端点
- **URL**: `GET http://127.0.0.1:5003/health`
- **响应**:
  ```json
  {
    "status": "healthy",
    "request_count": 0,
    "server_type": "http_test_server",
    "description": "HTTP测试服务器，返回固定动作轨迹"
  }
  ```

### 2. 推理端点
- **URL**: `POST http://127.0.0.1:5003/predict_action`
- **请求格式**:
  ```json
  {
    "examples": [
      {
        "image": null,
        "lang": "instruction text",
        "state": null
      }
    ]
  }
  ```
- **响应格式**:
  ```json
  {
    "data": {
      "unnormalized_actions": [[
        [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        // ... 24个更多动作
      ]]
    }
  }
  ```

### 3. 服务器信息端点
- **URL**: `GET http://127.0.0.1:5003/info`
- **响应**: 服务器配置和动作序列信息

## 动作序列说明

服务器返回25个7D动作，形成方形轨迹：

### 动作格式
每个动作是7维数组：`[dx, dy, dz, rx, ry, rz, gripper]`

### 轨迹模式
1. **阶段1 (动作0-4)**: X+方向 `[0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` (向右移动)
2. **阶段2 (动作5-9)**: Y+方向 `[0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0]` (向前移动)
3. **阶段3 (动作10-14)**: X-方向 `[-0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` (向左移动)
4. **阶段4 (动作15-19)**: Y-方向 `[0.0, -0.005, 0.0, 0.0, 0.0, 0.0, 0.0]` (向后移动)
5. **阶段5 (动作20-24)**: 零动作 `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` (回到原点)

## 客户端配置

### InferenceClient参数配置

客户端通过ROS参数配置，可以在启动时或运行时设置：

#### 关键参数
```bash
# 设置服务器地址和端口
rosparam set /inference_client/server_ip "127.0.0.1"
rosparam set /inference_client/server_port 5003

# 设置推理间隔（秒）
rosparam set /inference_client/interval 0.2

# 设置执行步数（每次请求返回的动作数量）
rosparam set /inference_client/exec_steps 5

# 设置安全限制
rosparam set /inference_client/safety/max_pos_step 0.05
rosparam set /inference_client/safety/max_rot_step 0.1
```

#### 查看当前参数
```bash
# 列出所有参数
rosparam list | grep inference

# 查看具体参数
rosparam get /inference_client/server_ip
rosparam get /inference_client/server_port
rosparam get /inference_client/use_mock
```

### 启动参数说明

在启动控制程序时，关键参数：
- `use_mock:=false` - 使用真实HTTP通信（必须设置为false）
- `debug:=true` - 启用调试日志

## 故障排除

### 常见问题

#### 1. 客户端没有收到服务器响应
- 检查服务器是否运行：`curl http://127.0.0.1:5003/health`
- 检查客户端配置：`rosparam get /inference_client/use_mock`（必须为false）
- 检查网络连接：`telnet 127.0.0.1 5003`

#### 2. 服务器启动失败
- 检查端口是否被占用：`netstat -tlnp | grep :5003`
- 检查Python依赖：确保Flask已安装

#### 3. 动作执行异常
- 检查安全限制参数
- 检查动作转换逻辑

### 调试命令

```bash
# 检查服务器进程
ps aux | grep test_http_server

# 检查端口监听
netstat -tlnp | grep :5003

# 实时查看服务器日志
cd .test/cline_selfbuild_test
python test_http_server.py --host 127.0.0.1 --port 5003

# 测试完整请求流程
curl -X POST http://127.0.0.1:5003/predict_action \
  -H 'Content-Type: application/json' \
  -d '{"examples":[{"image":null,"lang":"test","state":null}]}' \
  | python -c "import json, sys; data=json.load(sys.stdin); actions=data['data']['unnormalized_actions'][0]; print(f'动作数量: {len(actions)}'); print(f'第一个动作: {actions[0]}')"
```

## 高级用法

### 1. 修改动作序列
编辑`.test/cline_selfbuild_test/test_http_server.py`中的`ACTION_SEQUENCE`和`BASE_ACTIONS`定义。

### 2. 自定义服务器端口（高级用法）
```bash
# 示例：启动在5004端口的服务器（非默认）
python test_http_server.py --host 127.0.0.1 --port 5004

# 客户端配置相应端口
rosparam set /inference_client/server_port 5004
```

**注意**：默认端口是5003。如果修改端口，需要确保客户端和服务器使用相同的端口。

### 3. 性能测试
使用多个并发请求测试服务器性能：
```bash
# 使用ab进行压力测试
ab -n 100 -c 10 -p test_payload.json -T 'application/json' http://127.0.0.1:5003/predict_action
```

## 注意事项

1. **测试环境**：服务器仅用于测试，不应用于生产环境
2. **安全限制**：动作已应用安全限制，但实际机器人操作仍需谨慎
3. **网络配置**：确保客户端和服务器在同一网络或localhost
4. **ROS环境**：客户端需要ROS环境支持

## 更新日志

### 2026-01-23
- 初始版本发布
- 支持固定动作轨迹
- 兼容真实VLA模型服务器接口
- 提供完整的测试和调试工具

---

**如有问题，请参考代码注释或联系开发团队。**