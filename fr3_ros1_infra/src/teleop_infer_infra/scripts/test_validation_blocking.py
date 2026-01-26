#!/usr/bin/env python3
"""
测试验证阻滞功能

这个脚本测试验证服务器在等待用户输入时是否会正确阻滞新请求。
"""

import requests
import json
import time
import sys
import os
import subprocess
import threading

def test_validation_blocking():
    """测试验证阻滞功能"""
    print("=" * 70)
    print("测试验证阻滞功能")
    print("=" * 70)
    
    # 服务器配置
    host = "127.0.0.1"
    port = 5008  # 使用不同的端口避免冲突
    base_url = f"http://{host}:{port}"
    
    # 启动服务器（在后台）
    print("启动验证服务器...")
    server_process = subprocess.Popen(
        [
            sys.executable, 
            "src/teleop_infer_infra/scripts/h5_validation_server.py",
            "--host", host,
            "--port", str(port),
            "--h5-dir", "/home/alan/桌面/UMI_replay_数据/正常",
            "--mode", "sequential",
            "--no-diff-postprocess"
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        universal_newlines=True
    )
    
    # 等待服务器启动
    print("等待服务器启动...")
    time.sleep(5)
    
    # 检查服务器是否运行
    try:
        health_response = requests.get(f"{base_url}/health", timeout=5)
        if health_response.status_code == 200:
            print("✓ 服务器已启动")
        else:
            print(f"✗ 服务器健康检查失败: HTTP {health_response.status_code}")
            server_process.terminate()
            return False
    except requests.exceptions.RequestException as e:
        print(f"✗ 无法连接到服务器: {e}")
        server_process.terminate()
        return False
    
    # 测试1: 发送第一个请求
    print("\n测试1: 发送第一个请求...")
    try:
        response1 = requests.post(
            f"{base_url}/predict_action",
            json={
                "examples": [{
                    "image": ["dummy_image_data"],
                    "lang": "Test instruction 1",
                    "state": None
                }]
            },
            timeout=10
        )
        
        if response1.status_code == 200:
            print("✓ 第一个请求成功 (HTTP 200)")
            data1 = response1.json()
            actions = data1.get('data', {}).get('unnormalized_actions', [])
            if actions:
                print(f"  返回动作数: {len(actions[0])}")
        else:
            print(f"✗ 第一个请求失败: HTTP {response1.status_code}")
            print(f"  响应: {response1.text}")
    except requests.exceptions.RequestException as e:
        print(f"✗ 第一个请求异常: {e}")
    
    # 等待服务器进入验证状态
    print("\n等待服务器进入验证状态...")
    time.sleep(2)
    
    # 检查验证状态
    print("检查验证状态...")
    try:
        status_response = requests.get(f"{base_url}/validation/status", timeout=5)
        if status_response.status_code == 200:
            status_data = status_response.json()
            waiting = status_data.get('waiting_for_input', False)
            print(f"  等待输入: {waiting}")
            if waiting:
                print("  ✓ 服务器正在等待验证输入")
            else:
                print("  ✗ 服务器没有等待验证输入")
        else:
            print(f"✗ 无法获取验证状态: HTTP {status_response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"✗ 验证状态检查异常: {e}")
    
    # 测试2: 发送第二个请求（应该被阻滞）
    print("\n测试2: 发送第二个请求（应该被阻滞）...")
    try:
        response2 = requests.post(
            f"{base_url}/predict_action",
            json={
                "examples": [{
                    "image": ["dummy_image_data"],
                    "lang": "Test instruction 2",
                    "state": None
                }]
            },
            timeout=5
        )
        
        if response2.status_code == 503:
            print("✓ 第二个请求被正确阻滞 (HTTP 503)")
            error_data = response2.json()
            print(f"  错误信息: {error_data.get('message', '未知错误')}")
            print(f"  当前文件: {error_data.get('current_file', '未知')}")
        elif response2.status_code == 200:
            print("✗ 第二个请求成功 (HTTP 200) - 验证阻滞功能未生效!")
            print("  响应: ", response2.text[:100])
        else:
            print(f"✗ 第二个请求返回意外状态码: HTTP {response2.status_code}")
            print(f"  响应: {response2.text}")
    except requests.exceptions.Timeout:
        print("✓ 第二个请求超时（预期行为，服务器正在等待验证）")
    except requests.exceptions.RequestException as e:
        print(f"✗ 第二个请求异常: {e}")
    
    # 测试3: 检查健康端点（应该仍然可用）
    print("\n测试3: 检查健康端点（应该仍然可用）...")
    try:
        health_response2 = requests.get(f"{base_url}/health", timeout=5)
        if health_response2.status_code == 200:
            print("✓ 健康端点可用 (HTTP 200)")
            health_data = health_response2.json()
            print(f"  验证状态: {health_data.get('validation_status', '未知')}")
            print(f"  当前文件: {health_data.get('current_validation_file', '无')}")
        else:
            print(f"✗ 健康端点不可用: HTTP {health_response2.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"✗ 健康端点检查异常: {e}")
    
    # 清理
    print("\n清理: 停止服务器...")
    server_process.terminate()
    try:
        server_process.wait(timeout=5)
        print("✓ 服务器已停止")
    except subprocess.TimeoutExpired:
        print("⚠ 服务器终止超时，强制终止...")
        server_process.kill()
    
    print("\n" + "=" * 70)
    print("测试总结")
    print("=" * 70)
    print("验证阻滞功能测试完成。")
    print("预期行为:")
    print("1. 第一个请求成功 (HTTP 200)")
    print("2. 服务器进入验证状态，等待用户输入")
    print("3. 第二个请求被阻滞 (HTTP 503 或超时)")
    print("4. 健康端点仍然可用")
    print("\n如果测试结果符合预期，则验证阻滞功能正常工作。")

if __name__ == "__main__":
    test_validation_blocking()