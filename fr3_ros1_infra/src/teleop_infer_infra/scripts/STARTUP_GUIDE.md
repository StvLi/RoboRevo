# H5验证HTTP服务器启动指南

## 概述

这是一个具身数据验证replay的HTTP服务器，具有以下特性：
1. 与真实VLA模型服务器接口完全兼容
2. 每次请求自动切换到不同的h5文件
3. 具身数据验证功能：每次响应后等待用户输入(y/n)
4. 验证结果自动记录到CSV文件

## 快速启动

### 方法1：直接启动（推荐）

```bash
# 进入项目目录
cd /home/alan/fr3_ros1_infra

# 启动验证服务器（默认配置）
python3 src/teleop_infer_infra/scripts/h5_validation_server.py
```

### 方法2：使用完整参数

```bash
cd /home/alan/fr3_ros1_infra

python3 src/teleop_infer_infra/scripts/h5_validation_server.py \
  --host 127.0.0.1 \
  --port 5003 \
  --h5-dir "/home/alan/桌面/UMI_replay_数据/正常" \
  --file-pattern "banana*.h5" \
  --mode sequential \
  --no-diff-postprocess
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--host` | `127.0.0.1` | 服务器监听地址 |
| `--port` | `5003` | 服务器监听端口 |
| `--h5-dir` | `/home/alan/桌面/UMI_replay_数据/正常` | H5文件目录路径 |
| `--file-pattern` | `banana*.h5` | H5文件匹配模式（glob格式） |
| `--mode` | `sequential` | 文件切换模式：`sequential`（顺序）、`round_robin`（循环）、`random`（随机） |
| `--no-diff-postprocess` | 不设置（默认启用） | 禁用差分后处理 |

## 验证流程

服务器启动后的工作流程：

1. **初始化**：扫描H5文件目录，加载所有匹配的h5文件
2. **等待请求**：服务器监听指定端口，等待客户端请求
3. **处理请求**：
   - 客户端发送 `POST /predict_action` 请求
   - 服务器返回当前h5文件中的动作序列
   - 服务器弹出提示：`[验证管理器] 请输入验证结果 [Yes(y)/No(n)]:`
4. **验证阶段**：
   - 用户在**服务器命令行**输入 `y`（合格）或 `n`（不合格）
   - 验证结果自动记录到 `output_data/validation_results.csv`
5. **阻滞机制**：
   - 在等待用户输入期间，新的 `/predict_action` 请求会被阻滞（返回HTTP 503）
   - 其他端点（如 `/health`、`/info`）仍然可用
   - 用户输入后，服务器恢复处理新请求

## 管理端点

服务器提供以下HTTP端点：

### 主要端点
- `POST /predict_action` - 接收推理请求，返回动作序列，然后等待验证
- `GET /health` - 健康检查（包含验证状态）
- `GET /info` - 服务器详细信息

### H5管理端点
- `GET /h5/status` - 获取H5文件管理器状态
- `POST /h5/reload` - 重新加载H5文件
- `POST /h5/switch_mode` - 切换文件切换模式
- `POST /h5/reset_counter` - 重置请求计数器

### 验证管理端点
- `GET /validation/status` - 获取验证状态
- `POST /validation/skip` - 跳过当前验证

## CSV文件格式

验证结果自动保存到 `output_data/validation_results.csv`：

```csv
filename,timestamp,validation_result,request_time
banana1.h5,20260126160420,YES,20260126160415
banana10.h5,20260126160425,NO,20260126160422
```

字段说明：
- `filename`：验证的h5文件名
- `timestamp`：验证时间（YYYYMMDDHHmmss格式）
- `validation_result`：验证结果（YES/NO/SKIPPED）
- `request_time`：请求时间

## 使用示例

### 1. 启动服务器
```bash
python3 h5_validation_server.py --port 5003
```

### 2. 检查服务器状态
```bash
curl http://127.0.0.1:5003/health
```

### 3. 发送测试请求
```bash
curl -X POST http://127.0.0.1:5003/predict_action \
  -H "Content-Type: application/json" \
  -d '{
    "examples": [{
      "image": ["base64_image_data"],
      "lang": "Pick up the banana",
      "state": null
    }]
  }'
```

### 4. 查看验证状态
```bash
curl http://127.0.0.1:5003/validation/status
```

### 5. 跳过当前验证
```bash
curl -X POST http://127.0.0.1:5003/validation/skip
```

## 故障排除

### 常见问题

1. **服务器启动失败**
   - 检查端口是否被占用：`netstat -tlnp | grep 5003`
   - 尝试使用其他端口：`--port 5004`

2. **找不到H5文件**
   - 检查H5目录路径：`ls /home/alan/桌面/UMI_replay_数据/正常`
   - 调整文件模式：`--file-pattern "*.h5"`

3. **验证提示不显示**
   - 确保在服务器命令行（不是客户端）输入
   - 检查用户输入线程是否启动

4. **CSV文件未创建**
   - 检查输出目录权限：`ls -la output_data/`
   - 手动创建目录：`mkdir -p output_data`

### 日志查看

服务器启动时会显示详细日志，包括：
- 加载的H5文件数量
- 服务器地址和端口
- 验证配置信息
- 用户输入提示

## 高级配置

### 修改默认配置

编辑 `h5_validation_server.py` 文件中的默认参数：

```python
# 修改默认H5目录
parser.add_argument('--h5-dir', default="/your/custom/h5/directory", help='H5文件目录路径')

# 修改默认端口
parser.add_argument('--port', type=int, default=8080, help='服务器端口')
```

### 集成到现有系统

服务器与真实VLA模型服务器接口完全兼容，可以直接替换现有测试服务器：

```python
# 客户端代码无需修改
import requests

response = requests.post(
    "http://127.0.0.1:5003/predict_action",
    json={
        "examples": [{
            "image": image_data,
            "lang": instruction,
            "state": None
        }]
    }
)
```

## 注意事项

1. **同步验证**：服务器采用同步验证模式，确保每段轨迹都被验证后才继续执行
2. **命令行输入**：验证输入必须在**服务器命令行**进行，不是在客户端
3. **数据记录**：验证结果自动保存，不会丢失
4. **队列支持**：支持多个请求排队，按顺序验证
5. **兼容性**：保持与现有VLA模型服务器接口完全一致