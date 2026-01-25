import h5py
import numpy as np
def read_from_file(dir_to_file):
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
                    
                    # 限制动作数量，避免返回太多动作
                    max_actions = 100  # 限制最大动作数
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
                action_keys = ['actions', 'traj', 'trajectory', 'act', 'motion']
                for key in action_keys:
                    if key in f:
                        actions = f[key][:]
                        print(f"[read_from_file] 找到{key}数据集，形状: {actions.shape}")
                        
                        # 处理动作数据
                        if len(actions.shape) == 2 and actions.shape[1] >= 7:
                            actions = actions[:, :7]
                            # 限制动作数量
                            max_actions = 100
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
default_path = "/home/alan/桌面/UMI_replay_数据/正常/banana1.h5"
actions=read_from_file(default_path)