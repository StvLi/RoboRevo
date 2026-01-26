#!/usr/bin/env python3
"""
简单测试H5FileManager类

这个脚本用于测试H5FileManager类的基本功能。
"""

import os
import sys

# 添加当前脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

print("=" * 70)
print("测试H5FileManager类")
print("=" * 70)

# 尝试导入H5FileManager
try:
    from h5_file_manager import H5FileManager
    print("✓ 成功导入H5FileManager")
except ImportError as e:
    print(f"✗ 导入H5FileManager失败: {e}")
    # 尝试另一种导入方式
    try:
        import h5_file_manager
        from h5_file_manager import H5FileManager
        print("✓ 使用直接导入成功")
    except ImportError as e2:
        print(f"✗ 直接导入也失败: {e2}")
        sys.exit(1)

# 测试目录
test_dir = "/home/alan/桌面/UMI_replay_数据/正常"

# 检查目录是否存在
if not os.path.exists(test_dir):
    print(f"✗ 测试目录不存在: {test_dir}")
    sys.exit(1)

print(f"✓ 测试目录存在: {test_dir}")

# 创建H5FileManager实例
try:
    manager = H5FileManager(
        directory=test_dir,
        file_pattern="banana*.h5",
        mode="sequential",
        diff_postprocess=True,
        max_files=5  # 限制为5个文件用于测试
    )
    print("✓ H5FileManager初始化成功")
except Exception as e:
    print(f"✗ H5FileManager初始化失败: {e}")
    print("这可能是因为缺少依赖（h5py, numpy, scipy）")
    sys.exit(1)

# 获取状态
status = manager.get_status()
print(f"\nH5FileManager状态:")
print(f"  - 找到文件数: {status['total_files']}")
print(f"  - 当前模式: {status['mode']}")
print(f"  - 目录: {status['directory']}")
print(f"  - 文件模式: {status['file_pattern']}")
print(f"  - 差分后处理: {status['diff_postprocess']}")
print(f"  - 可用文件（前10个）: {status['available_files']}")

# 测试获取动作序列
print("\n测试获取动作序列:")
for i in range(3):
    try:
        action_sequence = manager.get_next_action_sequence()
        file_info = manager.get_current_file_info()
        print(f"  请求 #{i+1}:")
        print(f"    - 使用文件: {file_info['file_name']} (索引: {file_info['file_index']})")
        print(f"    - 动作序列形状: {action_sequence.shape}")
        print(f"    - 动作数量: {action_sequence.shape[0]}")
        print(f"    - 动作维度: {action_sequence.shape[1]}")
    except Exception as e:
        print(f"  请求 #{i+1} 失败: {e}")

# 测试模式切换
print("\n测试模式切换:")
try:
    result = manager.switch_mode("random")
    print(f"  切换到随机模式: {result['message']}")
    
    # 再获取一个动作序列
    action_sequence = manager.get_next_action_sequence()
    file_info = manager.get_current_file_info()
    print(f"  随机模式请求:")
    print(f"    - 使用文件: {file_info['file_name']} (索引: {file_info['file_index']})")
except Exception as e:
    print(f"  模式切换测试失败: {e}")

# 测试重置计数器
print("\n测试重置计数器:")
try:
    result = manager.reset_counter()
    print(f"  重置计数器: {result['message']}")
    
    status = manager.get_status()
    print(f"  重置后请求计数: {status['request_count']}")
except Exception as e:
    print(f"  重置计数器测试失败: {e}")

print("\n" + "=" * 70)
print("H5FileManager测试完成")
print("=" * 70)