# 多次请求顺序调用多个H5文件方案

## 概述

基于现有 `test_http_server.py` 中的 `read_from_file` 函数，本方案描述如何扩展服务器以支持在多次HTTP请求中顺序调用多个不同的h5文件。当前实现只响应第一次请求并返回单个h5文件的动作序列，后续请求返回空动作序列。本方案将修改这一行为，使服务器能够根据请求计数顺序返回不同h5文件的动作序列。

## 现有功能分析

### 当前实现
1. **`read_from_file` 函数**：已实现从h5文件读取动作轨迹的功能
   - 支持h5py库读取HDF5格式
   - 支持差分后处理（绝对位姿→相对动作）
   - 自动检测动作数据集名称
   - 处理7D/8D动作数据

2. **请求处理限制**：
   - 只响应第一次请求（`has_responded_first_chunk` 标志）
   - 后续请求返回空动作序列
   - 无法处理多个连续请求

3. **文件选择**：
   - 默认使用固定文件路径：`/home/alan/桌面/UMI_replay_数据/正常/banana1.h5`
   - 可通过查询参数 `file_path` 指定其他文件

## 扩展方案设计

### 1. 设计目标

#### 1.1 核心功能
- 支持在多次请求中顺序调用多个h5文件
- 保持与现有VLA模型服务器的接口兼容性
- 提供灵活的序列选择策略

#### 1.2 配置选项
- 支持多种文件调度模式：顺序、循环、随机
- 可配置h5文件目录和文件匹配模式
- 支持请求计数重置和状态管理

#### 1.3 向后兼容
- 保持现有 `read_from_file` 函数不变
- 支持现有查询参数（`sequence`, `file_path`）
- 默认行为与现有代码兼容

### 2. 系统架构修改

#### 2.1 全局状态扩展
```python
# 扩展全局状态
request_count = 0
lock = threading.Lock()

# 新增：H5文件管理状态
h5_files_list = []  # 可用的h5文件列表
current_file_index = 0  # 当前文件索引
file_schedule_mode = "sequential"  # 调度模式：sequential, round_robin, random
h5_files_directory = "/home/alan/桌面/UMI_replay_数据/正常"  # 默认目录
```

#### 2.2 新增配置参数
```python
# 命令行参数扩展
parser.add_argument('--h5-dir', default="/home/alan/桌面/UMI_replay_数据/正常", 
                   help='H5文件目录路径')
parser.add_argument('--h5-pattern', default="*.h5", 
                   help='H5文件匹配模式（glob模式）')
parser.add_argument('--h5-mode', default="sequential", 
                   choices=["sequential", "round_robin", "random"],
                   help='H5文件调度模式')
parser.add_argument('--max-requests-per-file', type=int, default=1,
                   help='每个文件服务的最大请求数')
```

### 3. 核心功能实现

#### 3.1 H5文件扫描函数
```python
def scan_h5_files(directory, pattern="*.h5"):
    """
    扫描指定目录下的h5文件
    
    Args:
        directory: 目录路径
        pattern: 文件匹配模式（glob格式）
        
    Returns:
        排序后的h5文件路径列表
    """
    import glob
    import os
    
    if not os.path.exists(directory):
        print(f"[scan_h5_files] 警告：目录不存在: {directory}")
        return []
    
    # 使用glob模式匹配文件
    search_pattern = os.path.join(directory, pattern)
    h5_files = glob.glob(search_pattern)
    
    # 按文件名排序（确保顺序一致性）
    h5_files.sort()
    
    print(f"[scan_h5_files] 在 {directory} 中找到 {len(h5_files)} 个h5文件")
    for i, file_path in enumerate(h5_files[:5]):  # 显示前5个文件
        print(f"  {i+1}. {os.path.basename(file_path)}")
    if len(h5_files) > 5:
        print(f"  ... 和 {len(h5_files)-5} 个其他文件")
    
    return h5_files
```

#### 3.2 文件选择函数
```python
def select_h5_file(request_count, h5_files, mode="sequential"):
    """
    根据请求计数和调度模式选择h5文件
    
    Args:
        request_count: 当前请求计数
        h5_files: 可用的h5文件列表
        mode: 调度模式
        
    Returns:
        选中的h5文件路径，或None（如果没有可用文件）
    """
    if not h5_files:
        print(f"[select_h5_file] 警告：没有可用的h5文件")
        return None
    
    if mode == "sequential":
        # 顺序模式：每个请求使用不同的文件
        file_index = request_count % len(h5_files)
        selected_file = h5_files[file_index]
        print(f"[select_h5_file] 顺序模式：请求#{request_count} → 文件#{file_index}: {os.path.basename(selected_file)}")
        
    elif mode == "round_robin":
        # 循环模式：循环使用所有文件
        # 每个文件服务固定数量的请求后切换到下一个文件
        requests_per_file = app.config.get('MAX_REQUESTS_PER_FILE', 1)
        file_index = (request_count // requests_per_file) % len(h5_files)
        selected_file = h5_files[file_index]
        print(f"[select_h5_file] 循环模式：请求#{request_count} → 文件#{file_index}: {os.path.basename(selected_file)}")
        
    elif mode == "random":
        # 随机模式：随机选择文件
        import random
        file_index = random.randint(0, len(h5_files) - 1)
        selected_file = h5_files[file_index]
        print(f"[select_h5_file] 随机模式：请求#{request_count} → 文件#{file_index}: {os.path.basename(selected_file)}")
        
    else:
        # 默认使用第一个文件
        selected_file = h5_files[0]
        print(f"[select_h5_file] 默认模式：使用第一个文件: {os.path.basename(selected_file)}")
    
    return selected_file
```

#### 3.3 修改 `predict_action` 函数
```python
@app.route('/predict_action', methods=['POST'])
def predict_action():
    global request_count, h5_files_list, current_file_index
    
    # 更新请求计数
    with lock:
        current_count = request_count
        request_count += 1
    
    print(f"[HTTP测试服务器] 收到请求 #{current_count}")
    
    # 解析请求（与真实服务器相同）
    try:
        data = request.get_json()
        if data and 'examples' in data:
            examples = data['examples']
            print(f"[HTTP测试服务器] 收到 {len(examples)} 个示例")
            
            # 检查请求格式
            for i, example in enumerate(examples):
                if 'lang' in example:
                    print(f"[HTTP测试服务器] 示例 {i} 指令: {example['lang']}")
                if 'image' in example and example['image']:
                    print(f"[HTTP测试服务器] 示例 {i} 包含图像数据")
                if 'state' in example:
                    print(f"[HTTP测试服务器] 示例 {i} 状态: {example['state']}")
    except Exception as e:
        print(f"[HTTP测试服务器] 请求解析错误: {e}")
    
    # 检查是否有查询参数指定序列类型
    sequence_type = request.args.get('sequence', 'read_from_file')
    file_path_param = request.args.get('file_path', None)
    
    print(f"[HTTP测试服务器] 使用序列类型: {sequence_type}")
    
    # 根据序列类型生成动作序列
    if sequence_type == "read_from_file":
        # 确定要使用的h5文件
        if file_path_param:
            # 使用查询参数指定的文件
            selected_file = file_path_param
            print(f"[HTTP测试服务器] 使用查询参数指定文件: {selected_file}")
        else:
            # 根据调度模式选择文件
            selected_file = select_h5_file(
                current_count, 
                h5_files_list, 
                mode=app.config.get('H5_SCHEDULE_MODE', 'sequential')
            )
        
        if selected_file and os.path.exists(selected_file):
            action_sequence = generate_action_sequence("read_from_file", selected_file)
        else:
            print(f"[HTTP测试服务器] 警告：文件不存在或未指定，使用默认方形轨迹")
            action_sequence = generate_action_sequence("square")
    else:
        # 使用固定方形轨迹
        action_sequence = generate_action_sequence("square")
    
    # 转换为列表
    actions_list = action_sequence.tolist()
    
    # 构建响应（与真实服务器格式相同）
    response = {
        "data": {
            "unnormalized_actions": [actions_list]  # batch_size=1
        }
    }
    
    print(f"[HTTP测试服务器] 返回 {len(actions_list)} 个动作")
    print(f"[HTTP测试服务器] 第一个动作: {actions_list[0]}")
    print(f"[HTTP测试服务器] 最后一个动作: {actions_list[-1]}")
    
    # 调试信息：显示动作序列的统计
    if current_count % 10 == 0:  # 每10次请求显示一次统计
        print(f"[HTTP测试服务器] 请求 #{current_count} 统计:")
        print(f"  - 总动作数: {len(actions_list)}")
        print(f"  - 动作维度: {len(actions_list[0])}D")
        print(f"  - 使用文件: {os.path.basename(selected_file) if selected_file else '固定序列'}")
    
    # 添加微小延迟以模拟网络延迟
    time.sleep(0.01)
    
    return jsonify(response)
```

#### 3.4 新增管理端点
```python
@app.route('/h5/status', methods=['GET'])
def h5_status():
    """获取H5文件管理器状态"""
    with lock:
        status = {
            "request_count": request_count,
            "h5_files_count": len(h5_files_list),
            "current_file_index": current_file_index,
            "schedule_mode": app.config.get('H5_SCHEDULE_MODE', 'sequential'),
            "h5_directory": app.config.get('H5_DIRECTORY', ''),
            "available_files": [os.path.basename(f) for f in h5_files_list[:10]],  # 只显示前10个
            "total_files": len(h5_files_list)
        }
    
    return jsonify(status)

@app.route('/h5/reload', methods=['POST'])
def h5_reload():
    """重新扫描并加载H5文件"""
    global h5_files_list
    
    directory = app.config.get('H5_DIRECTORY', '')
    pattern = app.config.get('H5_PATTERN', '*.h5')
    
    with lock:
        h5_files_list = scan_h5_files(directory, pattern)
    
    return jsonify({
        "status": "success",
        "message": f"重新加载了 {len(h5_files_list)} 个h5文件",
        "file_count": len(h5_files_list)
    })

@app.route('/h5/switch_mode', methods=['POST'])
def h5_switch_mode():
    """切换H5文件调度模式"""
    data = request.get_json()
    if not data or 'mode' not in data:
        return jsonify({"error": "缺少mode参数"}), 400
    
    mode = data['mode']
    valid_modes = ["sequential", "round_robin", "random"]
    
    if mode not in valid_modes:
        return jsonify({"error": f"无效模式，有效值: {valid_modes}"}), 400
    
    app.config['H5_SCHEDULE_MODE'] = mode
    
    return jsonify({
        "status": "success",
        "message": f"调度模式已切换为: {mode}",
        "new_mode": mode
    })

@app.route('/h5/reset_counter', methods=['POST'])
def h5_reset_counter():
    """重置请求计数器"""
    global request_count, current_file_index
    
    with lock:
        old_count = request_count
        request_count = 0
        current_file_index = 0
    
    return jsonify({
        "status": "success",
        "message": f"请求计数器已重置（原值: {old_count}）",
        "new_count": request_count
    })
```

### 4. 服务器初始化修改

#### 4.1 修改 `run_server` 函数
```python
def run_server(host='127.0.0.1', port=5003, h5_dir=None, h5_pattern="*.h5", h5_mode="sequential"):
    """
    启动HTTP测试服务器
    
    Args:
        host: 监听地址
        port: 监听端口
        h5_dir: H5文件目录路径
        h5_pattern: H5文件匹配模式
        h5_mode: H5文件调度模式
    """
    global h5_files_list
    
    # 设置配置
    app.config['H5_DIRECTORY'] = h5_dir
    app.config['H5_PATTERN'] = h5_pattern
    app.config['H5_SCHEDULE_MODE']