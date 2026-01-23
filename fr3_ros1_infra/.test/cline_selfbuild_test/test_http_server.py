#!/usr/bin/env python3
"""
HTTP服务器测试样例 - 模拟VLA模型服务器

这个服务器模拟云端部署的VLA模型服务器，但不使用图像进行推理，
而是返回固定的动作轨迹。用于测试InferenceClient的HTTP通信功能。

设计原则：
1. 与真实VLA模型服务器接口完全兼容
2. 返回固定的、可预测的动作序列用于测试
3. 部署在本机，便于测试HTTP通信
4. 不依赖图像输入，专注于测试通信协议
"""

import json
import numpy as np
from flask import Flask, request, jsonify
import threading
import time
import sys
import os

# 添加包路径以导入配置
sys.path.append(os.path.join(os.path.dirname(__file__), 'src/teleop_infer_infra/src'))

app = Flask(__name__)

# 全局变量跟踪请求计数
request_count = 0
lock = threading.Lock()

# 固定的动作轨迹定义
# 7D动作格式: [dx, dy, dz, rx, ry, rz, gripper]
# 我们定义一个简单的方形轨迹，每个方向5个动作

# 基础动作定义（单位：米/弧度）
BASE_ACTIONS = {
    'x_plus':  [0.005, 0.000, 0.000, 0.0, 0.0, 0.0, 0.0],   # X+方向
    'y_plus':  [0.000, 0.005, 0.000, 0.0, 0.0, 0.0, 0.0],   # Y+方向  
    'x_minus': [-0.005, 0.000, 0.000, 0.0, 0.0, 0.0, 0.0],  # X-方向
    'y_minus': [0.000, -0.005, 0.000, 0.0, 0.0, 0.0, 0.0],  # Y-方向
    'z_plus':  [0.000, 0.000, 0.005, 0.0, 0.0, 0.0, 0.0],   # Z+方向
    'z_minus': [0.000, 0.000, -0.005, 0.0, 0.0, 0.0, 0.0],  # Z-方向
    'zero':    [0.000, 0.000, 0.000, 0.0, 0.0, 0.0, 0.0]    # 零动作
}

# 动作序列：创建一个方形轨迹
ACTION_SEQUENCE = [
    'x_plus', 'x_plus', 'x_plus', 'x_plus', 'x_plus',      # 向右移动
    'y_plus', 'y_plus', 'y_plus', 'y_plus', 'y_plus',      # 向前移动
    'x_minus', 'x_minus', 'x_minus', 'x_minus', 'x_minus', # 向左移动
    'y_minus', 'y_minus', 'y_minus', 'y_minus', 'y_minus', # 向后移动
    'zero', 'zero', 'zero', 'zero', 'zero'                 # 回到原点
]

def generate_action_sequence(sequence_name="square"):
    """
    生成动作序列
    
    Args:
        sequence_name: 序列名称，目前只支持"square"
        
    Returns:
        numpy数组形状 (25, 7) 包含25个动作
    """
    if sequence_name == "square":
        # 生成方形轨迹
        actions = []
        for action_name in ACTION_SEQUENCE:
            actions.append(BASE_ACTIONS[action_name])
        return np.array(actions)
    else:
        # 默认返回零动作
        return np.zeros((25, 7))

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
    global request_count
    
    # 记录请求
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
    
    # 生成固定的动作序列（不依赖图像推理）
    # 每次请求返回完整的25个动作序列
    action_sequence = generate_action_sequence("square")
    
    # 转换为列表
    actions_list = action_sequence.tolist()
    
    # 注意：真实VLA服务器可能根据请求中的exec_steps参数返回不同数量的动作
    # 但为了测试兼容性，我们总是返回完整的25个动作序列
    
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
    if current_count == 0:  # 只在第一次请求时显示
        print(f"[HTTP测试服务器] 动作序列统计:")
        print(f"  - 总动作数: {len(actions_list)}")
        print(f"  - 动作维度: {len(actions_list[0])}D")
        print(f"  - 轨迹模式: 方形轨迹 (25个动作)")
    
    # 添加微小延迟以模拟网络延迟
    time.sleep(0.01)
    
    return jsonify(response)

@app.route('/health', methods=['GET'])
def health():
    """健康检查端点 - 与真实服务器相同"""
    with lock:
        count = request_count
    
    return jsonify({
        "status": "healthy", 
        "request_count": count,
        "server_type": "http_test_server",
        "description": "HTTP测试服务器，返回固定动作轨迹"
    })

@app.route('/info', methods=['GET'])
def info():
    """服务器信息端点"""
    return jsonify({
        "name": "HTTP测试服务器",
        "version": "1.0.0",
        "description": "模拟VLA模型服务器，返回固定动作轨迹",
        "endpoints": {
            "POST /predict_action": "接收推理请求，返回动作序列",
            "GET /health": "健康检查",
            "GET /info": "服务器信息"
        },
        "action_sequence": {
            "name": "square",
            "length": len(ACTION_SEQUENCE),
            "description": "方形轨迹：右→前→左→后→原点"
        }
    })

def run_server(host='127.0.0.1', port=5003):
    """
    启动HTTP测试服务器
    
    Args:
        host: 监听地址
        port: 监听端口
    """
    print("=" * 70)
    print("HTTP测试服务器启动")
    print("=" * 70)
    print(f"服务器地址: http://{host}:{port}")
    print(f"主要端点: POST /predict_action")
    print(f"健康检查: GET /health")
    print(f"服务器信息: GET /info")
    print()
    print("服务器特性:")
    print("- 与真实VLA模型服务器接口完全兼容")
    print("- 返回固定的方形动作轨迹（25个动作）")
    print("- 不依赖图像输入，专注于测试HTTP通信")
    print("- 部署在本机，便于测试")
    print()
    print("动作序列说明:")
    print("- 总共25个动作，分为5个阶段")
    print("- 阶段1: 5个X+动作（向右移动）")
    print("- 阶段2: 5个Y+动作（向前移动）")
    print("- 阶段3: 5个X-动作（向左移动）")
    print("- 阶段4: 5个Y-动作（向后移动）")
    print("- 阶段5: 5个零动作（回到原点）")
    print("=" * 70)
    
    app.run(host=host, port=port, debug=False, threaded=True)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='HTTP测试服务器')
    parser.add_argument('--host', default='127.0.0.1', help='服务器地址')
    parser.add_argument('--port', type=int, default=5003, help='服务器端口')
    
    args = parser.parse_args()
    
    # 直接运行服务器
    run_server(host=args.host, port=args.port)