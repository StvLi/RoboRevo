#!/usr/bin/env python3
"""
基于H5FileManager的具身数据验证HTTP服务器 - 在每次请求时自动切换不同的h5文件

这个服务器使用H5FileManager类来管理多个h5文件，每次收到/predict_action请求时，
会自动切换到下一个h5文件并返回其动作序列。

主要特性：
1. 与真实VLA模型服务器接口完全兼容
2. 每次请求自动切换到不同的h5文件（文件路径不在请求中）
3. 支持多种文件切换模式：顺序、循环、随机
4. 保持与现有test_http_server.py相同的IP和端口
5. 提供管理端点用于状态查询和模式切换
6. 具身数据验证功能：每次响应后等待用户输入(y/n)，记录验证结果到CSV

使用方式：
    python h5_validation_server.py --host 127.0.0.1 --port 5003 --mode sequential
"""

import json
import numpy as np
from flask import Flask, request, jsonify
import threading
import time
import sys
import os
import argparse
import select
import tty
import termios
import fcntl
import csv
from datetime import datetime

# 添加当前脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# 导入H5FileManager
try:
    from h5_file_manager import H5FileManager
except ImportError:
    # 尝试直接导入
    import h5_file_manager
    from h5_file_manager import H5FileManager

app = Flask(__name__)

# 全局变量
request_count = 0
lock = threading.Lock()
h5_manager = None
validation_lock = threading.Lock()
waiting_for_validation = False
current_validation_data = None
validation_queue = []

# CSV文件路径
output_dir = "output_data"
csv_file = os.path.join(output_dir, "validation_results.csv")

# 确保输出目录存在
os.makedirs(output_dir, exist_ok=True)

def init_h5_manager(h5_dir, file_pattern, mode, diff_postprocess):
    """
    初始化H5文件管理器
    
    Args:
        h5_dir: H5文件目录
        file_pattern: 文件匹配模式
        mode: 切换模式
        diff_postprocess: 是否应用差分后处理
    """
    global h5_manager
    
    print("=" * 70)
    print("初始化H5文件管理器")
    print("=" * 70)
    
    try:
        h5_manager = H5FileManager(
            directory=h5_dir,
            file_pattern=file_pattern,
            mode=mode,
            diff_postprocess=diff_postprocess,
            max_files=100
        )
        
        # 检查是否找到文件
        file_count = h5_manager.get_file_count()
        if file_count == 0:
            print("警告：未找到任何h5文件！")
            print(f"目录: {h5_dir}")
            print(f"文件模式: {file_pattern}")
        else:
            print(f"成功加载 {file_count} 个h5文件")
            
    except Exception as e:
        print(f"初始化H5文件管理器失败: {e}")
        import traceback
        traceback.print_exc()
        h5_manager = None

def _get_current_timestamp():
    """获取当前时间戳，格式为YYYYMMDDHHmmss"""
    return datetime.now().strftime("%Y%m%d%H%M%S")

def _init_csv_file():
    """初始化CSV文件，添加表头（如果文件不存在）"""
    if not os.path.exists(csv_file):
        with open(csv_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['filename', 'timestamp', 'validation_result', 'request_time'])
        print(f"[验证管理器] 创建新的CSV文件: {csv_file}")

def _record_to_csv(validation_data):
    """
    记录验证结果到CSV文件
    
    Args:
        validation_data: 验证数据字典
    """
    try:
        with open(csv_file, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                validation_data['filename'],
                validation_data['validation_time'],
                validation_data['validation_result'],
                validation_data['request_time']
            ])
        
        print(f"[验证管理器] 验证结果已记录到CSV文件: {csv_file}")
    except Exception as e:
        print(f"[验证管理器] 错误：记录到CSV文件失败: {e}")

def start_validation(filename, request_time=None):
    """
    开始验证过程
    
    Args:
        filename: 正在验证的文件名
        request_time: 请求时间（可选）
        
    Returns:
        bool: 是否成功开始验证
    """
    global waiting_for_validation, current_validation_data, validation_queue
    
    with validation_lock:
        if waiting_for_validation:
            print(f"[验证管理器] 警告：已有验证在进行中，将新验证加入队列")
            validation_queue.append({
                'filename': filename,
                'request_time': request_time or _get_current_timestamp()
            })
            return False
        
        # 设置验证数据
        current_validation_data = {
            'filename': filename,
            'request_time': request_time or _get_current_timestamp(),
            'validation_time': None,
            'validation_result': None
        }
        waiting_for_validation = True
        
        print(f"\n" + "=" * 70)
        print(f"[验证管理器] 开始验证")
        print(f"[验证管理器] 文件: {filename}")
        print(f"[验证管理器] 请求时间: {current_validation_data['request_time']}")
        print(f"[验证管理器] 请输入验证结果 [Yes(y)/No(n)]: ", end='', flush=True)
        
        return True

def process_user_input(user_input):
    """
    处理用户输入
    
    Args:
        user_input: 用户输入（y/n）
        
    Returns:
        bool: 是否成功处理输入
    """
    global waiting_for_validation, current_validation_data, validation_queue
    
    with validation_lock:
        if not waiting_for_validation or not current_validation_data:
            print(f"[验证管理器] 警告：没有等待验证的请求")
            return False
        
        # 清理用户输入
        user_input = user_input.strip().lower()
        
        # 验证输入
        if user_input not in ['y', 'n', 'yes', 'no']:
            print(f"[验证管理器] 错误：无效输入 '{user_input}'，请输入 y 或 n")
            return False
        
        # 确定验证结果
        validation_result = "YES" if user_input in ['y', 'yes'] else "NO"
        
        # 更新验证数据
        current_validation_data['validation_time'] = _get_current_timestamp()
        current_validation_data['validation_result'] = validation_result
        
        # 记录到CSV
        _record_to_csv(current_validation_data)
        
        print(f"[验证管理器] 验证结果: {validation_result}")
        print(f"[验证管理器] 验证时间: {current_validation_data['validation_time']}")
        print(f"[验证管理器] 验证完成")
        print("=" * 70 + "\n")
        
        # 重置状态
        waiting_for_validation = False
        completed_data = current_validation_data
        current_validation_data = None
        
        # 处理队列中的下一个验证
        if validation_queue:
            next_validation = validation_queue.pop(0)
            print(f"[验证管理器] 处理队列中的下一个验证: {next_validation['filename']}")
            start_validation(next_validation['filename'], next_validation['request_time'])
        
        return True

def is_waiting_for_validation():
    """检查是否正在等待验证输入"""
    with validation_lock:
        return waiting_for_validation

@app.before_request
def check_validation_status():
    """在每次请求前检查验证状态"""
    if request.path == '/predict_action' and is_waiting_for_validation():
        return jsonify({
            "error": "服务器正在等待验证输入",
            "message": "请先完成当前验证（输入y/n）",
            "current_file": current_validation_data['filename'] if current_validation_data else None
        }), 503  # 服务不可用

@app.route('/predict_action', methods=['POST'])
def predict_action():
    """
    处理推理请求 - 与真实VLA模型服务器接口完全兼容
    
    期望的请求格式（与真实服务器相同）：
    {
        "examples": [
            {
                "image": [base64_string1, base64_string2, base64_string3],
                "lang": "instruction string",
                "state": null  # 或状态数组
            }
        ]
    }
    
    返回格式（与真实服务器相同）：
    {
        "data": {
            "unnormalized_actions": [[[dx, dy, dz, rx, ry, rz, gripper], ...]]
        }
    }
    """
    global request_count, h5_manager
    
    # 更新请求计数
    with lock:
        current_count = request_count
        request_count += 1
    
    print(f"[H5 HTTP服务器] 收到请求 #{current_count}")
    
    # 解析请求（与真实服务器相同）
    try:
        data = request.get_json()
        if data and 'examples' in data:
            examples = data['examples']
            print(f"[H5 HTTP服务器] 收到 {len(examples)} 个示例")
            
            # 检查请求格式
            for i, example in enumerate(examples):
                if 'lang' in example:
                    print(f"[H5 HTTP服务器] 示例 {i} 指令: {example['lang']}")
                if 'image' in example and example['image']:
                    print(f"[H5 HTTP服务器] 示例 {i} 包含图像数据")
                if 'state' in example:
                    print(f"[H5 HTTP服务器] 示例 {i} 状态: {example['state']}")
    except Exception as e:
        print(f"[H5 HTTP服务器] 请求解析错误: {e}")
    
    # 获取当前文件信息（在获取动作序列之前）
    current_file_info = {}
    if h5_manager:
        current_file_info = h5_manager.get_current_file_info()
    
    # 生成动作序列
    if h5_manager and h5_manager.get_file_count() > 0:
        # 使用H5文件管理器获取下一个动作序列
        action_sequence = h5_manager.get_next_action_sequence()
        
        # 获取切换后的文件信息
        new_file_info = h5_manager.get_current_file_info()
        current_filename = new_file_info.get('file_name', '未知')
        print(f"[H5 HTTP服务器] 使用文件: {current_filename} "
              f"(索引: {new_file_info.get('file_index', -1)})")
    else:
        # 如果没有H5文件管理器或没有文件，返回零动作序列
        print(f"[H5 HTTP服务器] 警告：无可用h5文件，返回零动作序列")
        action_sequence = np.zeros((25, 7))
        current_filename = "unknown.h5"
    
    # 转换为列表
    actions_list = action_sequence.tolist()
    
    # 构建响应（与真实服务器格式相同）
    response = {
        "data": {
            "unnormalized_actions": [actions_list]  # batch_size=1
        }
    }
    
    print(f"[H5 HTTP服务器] 返回 {len(actions_list)} 个动作")
    if len(actions_list) > 0:
        print(f"[H5 HTTP服务器] 第一个动作: {actions_list[0]}")
        print(f"[H5 HTTP服务器] 最后一个动作: {actions_list[-1]}")
    
    # 每10次请求显示一次统计
    if current_count % 10 == 0:
        print(f"[H5 HTTP服务器] 请求 #{current_count} 统计:")
        print(f"  - 总动作数: {len(actions_list)}")
        print(f"  - 动作维度: {len(actions_list[0]) if actions_list else 0}D")
        if h5_manager:
            status = h5_manager.get_status()
            print(f"  - 已处理请求数: {status['request_count']}")
            print(f"  - 可用文件数: {status['total_files']}")
            print(f"  - 当前模式: {status['mode']}")
    
    # 启动验证过程
    request_time = _get_current_timestamp()
    start_validation(current_filename, request_time)
    
    # 添加微小延迟以模拟网络延迟
    time.sleep(0.01)
    
    return jsonify(response)

@app.route('/health', methods=['GET'])
def health():
    """健康检查端点 - 与真实服务器相同"""
    global request_count, h5_manager
    
    with lock:
        count = request_count
    
    h5_status = "未初始化"
    file_count = 0
    if h5_manager:
        status = h5_manager.get_status()
        h5_status = "运行中" if status['total_files'] > 0 else "无文件"
        file_count = status['total_files']
    
    validation_status = "等待输入" if is_waiting_for_validation() else "空闲"
    current_file = current_validation_data['filename'] if current_validation_data else None
    
    return jsonify({
        "status": "healthy", 
        "request_count": count,
        "server_type": "h5_validation_server",
        "h5_manager_status": h5_status,
        "h5_file_count": file_count,
        "validation_status": validation_status,
        "current_validation_file": current_file,
        "validation_queue_length": len(validation_queue),
        "description": "H5验证HTTP服务器，每次请求自动切换不同h5文件并等待验证输入"
    })

@app.route('/info', methods=['GET'])
def info():
    """服务器信息端点"""
    global h5_manager
    
    endpoints = {
        "POST /predict_action": "接收推理请求，返回h5文件中的动作序列，然后等待验证输入",
        "GET /health": "健康检查（包含验证状态）",
        "GET /info": "服务器信息",
        "GET /h5/status": "H5文件管理器状态",
        "POST /h5/reload": "重新加载H5文件",
        "POST /h5/switch_mode": "切换文件切换模式",
        "POST /h5/reset_counter": "重置请求计数器",
        "POST /validation/skip": "跳过当前验证",
        "GET /validation/status": "获取验证状态"
    }
    
    h5_info = {}
    if h5_manager:
        status = h5_manager.get_status()
        h5_info = {
            "directory": status['directory'],
            "file_pattern": status['file_pattern'],
            "mode": status['mode'],
            "total_files": status['total_files'],
            "request_count": status['request_count'],
            "diff_postprocess": status['diff_postprocess']
        }
    
    validation_info = {
        "waiting_for_input": is_waiting_for_validation(),
        "current_filename": current_validation_data['filename'] if current_validation_data else None,
        "queue_length": len(validation_queue),
        "csv_file": csv_file,
        "output_dir": output_dir
    }
    
    return jsonify({
        "name": "H5验证HTTP服务器",
        "version": "1.0.0",
        "description": "基于H5FileManager的具身数据验证HTTP服务器，每次请求自动切换不同h5文件并等待验证输入",
        "endpoints": endpoints,
        "h5_manager": h5_info,
        "validation_manager": validation_info
    })

@app.route('/h5/status', methods=['GET'])
def h5_status():
    """获取H5文件管理器状态"""
    global h5_manager
    
    if not h5_manager:
        return jsonify({
            "status": "error",
            "message": "H5文件管理器未初始化"
        }), 500
    
    status = h5_manager.get_status()
    return jsonify(status)

@app.route('/h5/reload', methods=['POST'])
def h5_reload():
    """重新扫描并加载H5文件"""
    global h5_manager
    
    if not h5_manager:
        return jsonify({
            "status": "error",
            "message": "H5文件管理器未初始化"
        }), 500
    
    result = h5_manager.reload_files()
    return jsonify(result)

@app.route('/h5/switch_mode', methods=['POST'])
def h5_switch_mode():
    """切换H5文件切换模式"""
    global h5_manager
    
    if not h5_manager:
        return jsonify({
            "status": "error",
            "message": "H5文件管理器未初始化"
        }), 500
    
    data = request.get_json()
    if not data or 'mode' not in data:
        return jsonify({
            "status": "error",
            "message": "缺少mode参数"
        }), 400
    
    result = h5_manager.switch_mode(data['mode'])
    return jsonify(result)

@app.route('/h5/reset_counter', methods=['POST'])
def h5_reset_counter():
    """重置请求计数器"""
    global h5_manager, request_count
    
    if not h5_manager:
        return jsonify({
            "status": "error",
            "message": "H5文件管理器未初始化"
        }), 500
    
    # 重置H5管理器的计数器
    h5_result = h5_manager.reset_counter()
    
    # 重置服务器的请求计数器
    with lock:
        old_count = request_count
        request_count = 0
    
    return jsonify({
        "status": "success",
        "message": f"计数器已重置",
        "server_request_count_old": old_count,
        "server_request_count_new": request_count,
        "h5_manager_result": h5_result
    })

@app.route('/validation/status', methods=['GET'])
def validation_status():
    """获取验证状态"""
    return jsonify({
        "waiting_for_input": is_waiting_for_validation(),
        "current_filename": current_validation_data['filename'] if current_validation_data else None,
        "queue_length": len(validation_queue),
        "csv_file": csv_file,
        "output_dir": output_dir
    })

@app.route('/validation/skip', methods=['POST'])
def validation_skip():
    """跳过当前验证"""
    global waiting_for_validation, current_validation_data, validation_queue
    
    with validation_lock:
        if not waiting_for_validation or not current_validation_data:
            return jsonify({
                "status": "error",
                "message": "没有等待验证的请求"
            }), 400
        
        # 更新验证数据
        current_validation_data['validation_time'] = _get_current_timestamp()
        current_validation_data['validation_result'] = "SKIPPED"
        
        # 记录到CSV
        _record_to_csv(current_validation_data)
        
        print(f"[验证管理器] 验证已跳过")
        
        # 重置状态
        waiting_for_validation = False
        skipped_data = current_validation_data
        current_validation_data = None
        
        # 处理队列中的下一个验证
        if validation_queue:
            next_validation = validation_queue.pop(0)
            print(f"[验证管理器] 处理队列中的下一个验证: {next_validation['filename']}")
            start_validation(next_validation['filename'], next_validation['request_time'])
        
        return jsonify({
            "status": "success",
            "message": "验证已跳过",
            "skipped_file": skipped_data['filename']
        })

def user_input_thread():
    """用户输入处理线程"""
    print("[用户输入线程] 启动，等待用户输入...")
    
    # 设置非阻塞输入
    import sys
    import select
    
    while True:
        try:
            # 检查是否有输入可用
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                user_input = sys.stdin.readline().strip()
                if user_input:
                    process_user_input(user_input)
            else:
                # 短暂休眠以避免CPU占用过高
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[用户输入线程] 收到中断信号，退出")
            break
        except Exception as e:
            print(f"[用户输入线程] 错误: {e}")
            time.sleep(1)

def run_server(host='127.0.0.1', port=5003, h5_dir=None, file_pattern="banana*.h5", 
               mode="sequential", diff_postprocess=True):
    """
    启动H5验证HTTP服务器
    
    Args:
        host: 监听地址
        port: 监听端口
        h5_dir: H5文件目录路径
        file_pattern: H5文件匹配模式
        mode: H5文件切换模式
        diff_postprocess: 是否应用差分后处理
    """
    global h5_manager
    
    print("=" * 70)
    print("H5验证HTTP服务器启动")
    print("=" * 70)
    print(f"服务器地址: http://{host}:{port}")
    print(f"主要端点: POST /predict_action")
    print(f"健康检查: GET /health")
    print(f"服务器信息: GET /info")
    print(f"H5管理端点: GET /h5/status, POST /h5/reload, POST /h5/switch_mode, POST /h5/reset_counter")
    print(f"验证管理端点: GET /validation/status, POST /validation/skip")
    print()
    print("服务器特性:")
    print("- 与真实VLA模型服务器接口完全兼容")
    print("- 每次请求自动切换到不同的h5文件")
    print("- 支持多种切换模式: sequential, round_robin, random")
    print("- 具身数据验证功能：每次响应后等待用户输入(y/n)")
    print("- 验证结果自动记录到CSV文件")
    print("- 部署在本机，便于测试")
    print()
    print("H5文件配置:")
    print(f"  - 目录: {h5_dir}")
    print(f"  - 文件模式: {file_pattern}")
    print(f"  - 切换模式: {mode}")
    print(f"  - 差分后处理: {diff_postprocess}")
    print()
    print("验证配置:")
    print(f"  - 输出目录: {output_dir}")
    print(f"  - CSV文件: {csv_file}")
    print("=" * 70)
    
    # 初始化CSV文件
    _init_csv_file()
    
    # 初始化H5文件管理器
    init_h5_manager(h5_dir, file_pattern, mode, diff_postprocess)
    
    # 检查初始化结果
    if h5_manager:
        file_count = h5_manager.get_file_count()
        if file_count == 0:
            print("警告：未找到任何h5文件，服务器将继续运行但将返回零动作序列")
        else:
            print(f"已加载 {file_count} 个h5文件，准备接收请求")
    else:
        print("错误：H5文件管理器初始化失败，服务器将继续运行但将返回零动作序列")
    
    # 启动用户输入线程
    input_thread = threading.Thread(target=user_input_thread, daemon=True)
    input_thread.start()
    print("[主线程] 用户输入线程已启动")
    
    # 启动Flask服务器
    print("[主线程] 启动Flask服务器...")
    app.run(host=host, port=port, debug=False, threaded=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='H5验证HTTP服务器')
    parser.add_argument('--host', default='127.0.0.1', help='服务器地址')
    parser.add_argument('--port', type=int, default=5003, help='服务器端口')
    parser.add_argument('--h5-dir', default="/home/alan/桌面/UMI_replay_数据/正常", 
                       help='H5文件目录路径')
    parser.add_argument('--file-pattern', default="banana*.h5", 
                       help='H5文件匹配模式（glob模式）')
    parser.add_argument('--mode', default="sequential", 
                       choices=["sequential", "round_robin", "random"],
                       help='H5文件切换模式')
    parser.add_argument('--no-diff-postprocess', action='store_false', dest='diff_postprocess',
                       help='禁用差分后处理')
    
    args = parser.parse_args()
    
    # 设置diff_postprocess参数
    diff_postprocess = args.diff_postprocess
    
    # 直接运行服务器
    run_server(
        host=args.host,
        port=args.port,
        h5_dir=args.h5_dir,
        file_pattern=args.file_pattern,
        mode=args.mode,
        diff_postprocess=diff_postprocess
    )
