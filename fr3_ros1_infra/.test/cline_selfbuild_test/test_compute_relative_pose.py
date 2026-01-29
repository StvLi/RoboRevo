# 文件路径: fr3_ros1_infra/.test/cline_selfbuild_test/test_compute_relative_pose.py

import sys
import os
import numpy as np

# 添加项目根目录到Python路径，以便导入模块
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))

from util.compute_relative_pose import Pose, compute_relative_pose, relative_actions_from_init, differential_actions_between_step

def test_compute_relative_pose():
    """
    测试 compute_relative_pose 函数的功能。
    """
    print("Testing compute_relative_pose...")
    
    # 定义参考姿态和目标姿态
    refer_pose = Pose(translation=[1, 2, 3], rotation=[0, 0, 0, 1], rot_form='quat')
    target_pose = Pose(translation=[4, 5, 6], rotation=[0, 0, 0, 1], rot_form='quat')
    
    # 计算相对姿态
    relative_pose = compute_relative_pose(target_pose, refer_pose)
    
    # 验证结果
    expected_translation = np.array([3, 3, 3])  # 目标平移减去参考平移
    expected_rotation = np.array([0, 0, 0])     # 轴角未发生变化
    
    print(relative_pose.rotation.as_quat())
    assert np.allclose(relative_pose.translation, expected_translation), "Translation mismatch!"
    assert np.allclose(relative_pose.rotation.as_rotvec(), expected_rotation), "Rotation mismatch!"
    
    print("compute_relative_pose test passed!")

def test_relative_actions_from_init():
    """
    测试 relative_actions_from_init 函数的功能。
    """
    print("Testing relative_actions_from_init...")
    
    # 定义初始状态和目标状态列表
    init_states = [0, 0, 0, 0, 0, 0, 1]  # 平移 + 四元数
    target_states = [
        [1, 0, 0, 0, 0, 0, 1],
        [0, 1, 0, 0, 0, 0, 1],
        [0, 0, 1, 0, 0, 0, 1]
    ]
    
    # 计算相对动作序列
    actions = relative_actions_from_init(target_states, init_states)
    
    # 验证结果
    expected_actions = [
        np.array([1, 0, 0, 0, 0, 0]),  # 第一个目标状态的相对动作
        np.array([0, 1, 0, 0, 0, 0]),  # 第二个目标状态的相对动作
        np.array([0, 0, 1, 0, 0, 0])   # 第三个目标状态的相对动作
    ]
    
    for i, action in enumerate(actions):
        assert np.allclose(action, expected_actions[i]), f"Action {i} mismatch!"
    
    print("relative_actions_from_init test passed!")

def test_differential_actions_between_step():
    """
    测试 differential_actions_between_step 函数的功能。
    """
    print("Testing differential_actions_between_step...")
    
    # 定义状态序列
    states = [
        [0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 1, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 0, 0, 1]
    ]
    
    # 计算差分动作序列
    actions = differential_actions_between_step(states)
    
    # 验证结果
    expected_actions = [
        np.array([1, 0, 0, 0, 0, 0]),  # 第一步差分动作
        np.array([0, 1, 0, 0, 0, 0]),  # 第二步差分动作
        np.array([0, 0, 1, 0, 0, 0])   # 第三步差分动作
    ]
    
    for i, action in enumerate(actions):
        assert np.allclose(action, expected_actions[i]), f"Differential action {i} mismatch!"
    
    print("differential_actions_between_step test passed!")

if __name__ == "__main__":
    # 运行所有测试
    test_compute_relative_pose()
    test_relative_actions_from_init()
    test_differential_actions_between_step()
    print("All tests passed successfully!")