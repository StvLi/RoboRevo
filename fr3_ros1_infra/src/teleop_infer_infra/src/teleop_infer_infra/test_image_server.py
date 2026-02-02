#!/usr/bin/env python3
"""
HTTP服务器测试样例 V2 - 验证图像上传与夹爪控制

功能升级：
1. 检查 Client 是否上传了 3 张图片（中、左、右）。
2. 返回的动作序列包含夹爪控制信号（第7维）。
   - 前半段：移动并张开夹爪
   - 后半段：移动并闭合夹爪
"""

import json
import numpy as np
from flask import Flask, request, jsonify
import threading
import time

app = Flask(__name__)

# 全局计数
request_count = 0
lock = threading.Lock()

# === 动作定义 ===
# 格式: [dx, dy, dz, rx, ry, rz, gripper]
# gripper > 0.5 表示张开，<= 0.5 表示闭合

# 动作 A: 向右移动 + 张开夹爪
ACTION_OPEN_RIGHT = [0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] 

# 动作 B: 向左移动 + 闭合夹爪
ACTION_CLOSE_LEFT = [-0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 动作 C: 停止 + 保持闭合
ACTION_STOP = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def generate_gripper_test_sequence():
    """
    生成一个测试序列：
    1. 前 10 步：向右 + 张开
    2. 后 10 步：向左 + 闭合
    3. 最后 5 步：停止
    """
    actions = []
    # 阶段 1: 张开
    for _ in range(10):
        actions.append(ACTION_OPEN_RIGHT)
    # 阶段 2: 闭合
    for _ in range(10):
        actions.append(ACTION_CLOSE_LEFT)
    # 阶段 3: 停止
    for _ in range(5):
        actions.append(ACTION_STOP)
        
    return np.array(actions)

@app.route('/predict_action', methods=['POST'])
def predict_action():
    global request_count
    with lock:
        request_count += 1
        current_count = request_count

    print(f"\n[Server] 收到请求 #{current_count} " + "="*30)
    
    # === 1. 验证图像输入 ===
    try:
        data = request.get_json()
        if data and 'examples' in data:
            example = data['examples'][0]
            
            # 检查图像字段
            images = example.get('image', [])
            
            if images is None:
                print("❌ 错误: 收到 image 字段为 None (Client可能没发图)")
            elif len(images) == 3:
                # 打印每张图的数据长度，验证是否是 Base64
                sizes = [len(img) for img in images]
                print(f"✅ 成功接收 3 张图像!")
                print(f"   - 中相机大小: {sizes[0]/1024:.1f} KB")
                print(f"   - 左相机大小: {sizes[1]/1024:.1f} KB")
                print(f"   - 右相机大小: {sizes[2]/1024:.1f} KB")
            else:
                print(f"⚠️ 警告: 收到 {len(images)} 张图像 (期望 3 张)")
                
            # 检查指令
            lang = example.get('lang', "")
            print(f"   - 指令文本: '{lang}'")
            
    except Exception as e:
        print(f"❌ 解析请求失败: {e}")

    # === 2. 生成动作序列 ===
    action_sequence = generate_gripper_test_sequence()
    actions_list = action_sequence.tolist()

    response = {
        "data": {
            "unnormalized_actions": [actions_list]
        }
    }
    
    # 只有第一次请求时打印动作数据，避免刷屏
    if current_count == 1:
        print(f"[Server] 发送测试轨迹: 10步张开 -> 10步闭合 -> 5步停止")
        print(f"   - 第1步(张开): {actions_list[0]}")
        print(f"   - 第15步(闭合): {actions_list[15]}")

    # 模拟网络延迟
    time.sleep(0.05)
    return jsonify(response)

@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "healthy", "mode": "gripper_test"})

def run_server(port=5003):
    print(f"启动 V2 测试服务器 - 端口 {port}")
    print("功能: 验证 3 相机上传 + 测试夹爪开关")
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5003) # 默认改为 5003
    args = parser.parse_args()
    run_server(port=args.port)