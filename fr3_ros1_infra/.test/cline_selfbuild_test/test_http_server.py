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
import h5py
import ipdb
from scipy.spatial.transform import Rotation as R
# 添加包路径以导入配置
sys.path.append(os.path.join(os.path.dirname(__file__), 'src/teleop_infer_infra/src'))

app = Flask(__name__)

# 全局变量跟踪请求计数
request_count = 0
lock = threading.Lock()
# 标志：是否已经响应过第一次请求
has_responded_first_chunk = False

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

def read_from_file(dir_to_file, diff_postprocess = True):
    """
    从h5文件中读取动作轨迹
    
    Args:
        dir_to_file: h5文件路径
        
    Returns:
        numpy数组形状 (n, 7) 包含n个动作，每个动作7个维度
        [dx, dy, dz, rx, ry, rz, gripper]
    """
    try:
        print(f"[read_from_file] 读取文件: {dir_to_file}")
        
        with h5py.File(dir_to_file, 'r') as f:
            # 检查文件中的数据集
            if 'action' in f:
                actions = f['action'][:]
                # ipdb.set_trace()
                print(f"[read_from_file] 找到action数据集，形状: {actions.shape}")
                
                # 检查动作维度
                if len(actions.shape) == 2:
                    # 如果动作有8个维度，取前7个
                    if actions.shape[1] == 8:
                        print(f"[read_from_file] 动作有8个维度，取前7个")
                        actions = actions[:, :7]
                    elif actions.shape[1] == 7:
                        print(f"[read_from_file] 动作有7个维度，直接使用")
                    else:
                        print(f"[read_from_file] 警告: 动作维度为{actions.shape[1]}，期望7或8")
                        # 如果维度不匹配，尝试取前7列
                        if actions.shape[1] > 7:
                            actions = actions[:, :7]
                        else:
                            # 如果维度不足，补零
                            print(f"[read_from_file] 警告: 维度不足，补零到7维")
                            new_actions = np.zeros((actions.shape[0], 7))
                            new_actions[:, :actions.shape[1]] = actions
                            actions = new_actions

                    # 如果设置差分后处理
                    if diff_postprocess == True:
                        print(f"[read_from_file] 应用差分后处理（4x4齐次变换矩阵逻辑）")
                        # 使用4x4齐次变换矩阵计算差分
                        # 第0步保持原样，第1步及以后计算相对变换
                        new_actions = np.zeros([actions.shape[0],6])
                        # new_actions[0, 0:3] = actions[0, 0:3]  # 第0步保持原样
                        # new_actions[0, 3:6] = [R.from_quat(actions[0, 3:7]).as_rotvec()]
                        # print('<<<FIRST FRAME>>>')
                        # print(actions[0,0:3])
                        # print(R.from_quat(actions[0,3:7]).as_matrix())
                        # print(R.from_quat(
                        #     [actions[0,4], actions[0,5], actions[0,6], actions[0,3]]
                        #                   ).as_matrix())

                        for i in range(1, actions.shape[0]):
                            # 提取前一个动作的平移和旋转
                            t_prev = actions[i-1, 0:3]  # 平移: [dx, dy, dz]
                            q_prev = actions[i-1, 3:7]  # 四元数: [qx, qy, qz, qw]
                            # q_prev = [actions[i-1, 4],actions[i-1, 5],actions[i-1, 6],actions[i-1, 3]]
                            
                            # 提取当前动作的平移和旋转
                            t_curr = actions[i, 0:3]    # 平移: [dx, dy, dz]
                            q_curr = actions[i, 3:7]    # 四元数: [qx, qy, qz, qw]
                            # q_curr = [actions[i, 4],actions[i, 5],actions[i, 6],actions[i, 3]]
                            
                            # 创建旋转对象并获取3x3旋转矩阵
                            R_prev_obj = R.from_quat(q_prev)
                            R_curr_obj = R.from_quat(q_curr)
                            R_prev = R_prev_obj.as_matrix()  # 3x3旋转矩阵
                            R_curr = R_curr_obj.as_matrix()  # 3x3旋转矩阵
                            
                            # 构建4x4齐次变换矩阵 T_prev
                            T_prev = np.eye(4)
                            T_prev[0:3, 0:3] = R_prev
                            T_prev[0:3, 3] = t_prev
                            
                            # 构建4x4齐次变换矩阵 T_curr
                            T_curr = np.eye(4)
                            T_curr[0:3, 0:3] = R_curr
                            T_curr[0:3, 3] = t_curr
                            
                            # 计算相对变换：ΔT = T_prev^{-1} * T_curr
                            T_prev_inv = np.linalg.inv(T_prev)
                            T_delta = T_prev_inv @ T_curr
                            
                            # 从ΔT中提取相对平移和旋转
                            t_rel = T_delta[0:3, 3]  # 相对平移
                            R_delta = T_delta[0:3, 0:3]  # 相对旋转矩阵
                            
                            # 将旋转矩阵转换为轴角表示
                            R_delta_obj = R.from_matrix(R_delta)
                            rotvec = R_delta_obj.as_rotvec()
                            
                            # 将结果存储到新动作中
                            new_actions[i, 0:3] = t_rel  # 相对平移
                            new_actions[i, 3:6] = rotvec  # 相对旋转（轴角表示）
                        #     new_actions[i, 0:3] = [0,0,0] # 调试不平移
                        #     new_actions[i, 3:6] = [0,0,0] # 调试不旋转
                        # new_actions[0, 0:3] = [0,0,0.1] # 调试不平移
                        # new_actions[0, 3:6] = [0,0,0] # 调试不旋转

                        # 将差分后的动作赋值回actions变量
                        # ipdb.set_trace()
                        actions = new_actions
                        print(f"[read_from_file] 差分后动作形状: {actions.shape}")
                        print(f"[read_from_file] 差分后第一个动作: {actions[0]}")
                        print(f"[read_from_file] 差分后第二个动作: {actions[1]} (相对变换)")

                    
                    # 限制动作数量，避免返回太多动作
                    max_actions = 1000  # 限制最大动作数
                    if actions.shape[0] > max_actions:
                        print(f"[read_from_file] 动作数量{actions.shape[0]}超过限制{max_actions}，取前{max_actions}个")
                        actions = actions[:max_actions, :]
                    
                    print(f"[read_from_file] 返回动作形状: {actions.shape}")
                    return actions
                else:
                    print(f"[read_from_file] 错误: action数据集形状不是2D: {actions.shape}")
                    return np.zeros((25, 7))
            else:
                # 尝试其他常见名称
                action_keys = ['actions', 'traj', 'trajectory', 'act', 'motion', 'states']
                for key in action_keys:
                    if key in f:
                        actions = f[key][:]
                        print(f"[read_from_file] 找到{key}数据集，形状: {actions.shape}")
                        
                        # 处理动作数据
                        if len(actions.shape) == 2 and actions.shape[1] >= 7:
                            actions = actions[:, :7]
                            # 限制动作数量
                            max_actions = 1000
                            if actions.shape[0] > max_actions:
                                actions = actions[:max_actions, :]
                            print(f"[read_from_file] 返回动作形状: {actions.shape}")
                            return actions
                
                print(f"[read_from_file] 错误: 文件中未找到动作数据集")
                print(f"[read_from_file] 可用键: {list(f.keys())}")
                return np.zeros((25, 7))
                
    except Exception as e:
        print(f"[read_from_file] 读取文件错误: {e}")
        import traceback
        traceback.print_exc()
        return np.zeros((25, 7))


def generate_action_sequence(sequence_name="read_from_file", file_path=None):
    """
    生成动作序列
    
    Args:
        sequence_name: 序列名称，支持"square"或"read_from_file"
        file_path: 当sequence_name为"read_from_file"时，指定h5文件路径
        
    Returns:
        numpy数组形状 (n, 7) 包含n个动作
    """
    if sequence_name == "square":
        # 生成方形轨迹
        actions = []
        for action_name in ACTION_SEQUENCE:
            actions.append(BASE_ACTIONS[action_name])
        return np.array(actions)
    elif sequence_name == "read_from_file":
        if file_path is None:
            # 使用默认文件路径
            default_path = "/home/alan/桌面/UMI_replay_数据/正常/banana1.h5"
            print(f"[generate_action_sequence] 使用默认文件路径: {default_path}")
            actions = read_from_file(default_path)
        else:
            actions = read_from_file(file_path)
        return actions
        # return np.array([[0.1,0,0,0,0,0]])
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
    global request_count, has_responded_first_chunk
    
    # 记录请求
    with lock:
        current_count = request_count
        request_count += 1
    
    print(f"[HTTP测试服务器] 收到请求 #{current_count}")
    
    # 检查是否已经响应过第一次请求
    if has_responded_first_chunk and current_count > 0:
        print(f"[HTTP测试服务器] 警告：已响应过第一次action chunk，本次请求 #{current_count} 将返回空动作序列")
        # 返回空动作序列
        empty_actions = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 5  # 返回5个零动作
        response = {
            "data": {
                "unnormalized_actions": [empty_actions]
            }
        }
        return jsonify(response)
    
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
    # 检查是否有查询参数指定序列类型
    sequence_type = request.args.get('sequence', 'read_from_file')
    file_path_param = request.args.get('file_path', None)
    
    print(f"[HTTP测试服务器] 使用序列类型: {sequence_type}")
    if file_path_param:
        print(f"[HTTP测试服务器] 使用文件路径: {file_path_param}")
    
    # 根据序列类型生成动作序列
    if sequence_type == "read_from_file":
        action_sequence = generate_action_sequence("read_from_file", file_path_param)
    else:
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
        print(f"  - 轨迹模式: {sequence_type}轨迹 ({len(actions_list)}个动作)")
    
    # 标记已经响应过第一次请求
    with lock:
        has_responded_first_chunk = True
    
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