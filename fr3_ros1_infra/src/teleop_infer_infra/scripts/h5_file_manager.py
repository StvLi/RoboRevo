#!/usr/bin/env python3
"""
H5文件管理器 - 用于在多次请求中顺序读取不同的h5文件

这个模块提供了一个H5FileManager类，用于管理位于指定目录下的多个h5文件。
每次调用get_next_action_sequence()方法时，会自动切换到下一个文件并返回其动作序列。

主要功能：
1. 自动扫描指定目录下的h5文件（支持banana*.h5模式）
2. 支持多种文件切换模式：顺序、循环、随机
3. 封装read_from_file功能，支持差分后处理
4. 线程安全，支持多线程环境
5. 提供状态查询和重置功能

使用示例：
    manager = H5FileManager(
        directory="/home/alan/桌面/UMI_replay_数据/正常",
        file_pattern="banana*.h5",
        mode="sequential"
    )
    
    # 每次调用返回不同文件的动作序列
    actions1 = manager.get_next_action_sequence()
    actions2 = manager.get_next_action_sequence()
"""

import os
import glob
import threading
import numpy as np
import h5py
from scipy.spatial.transform import Rotation as R
import random
from typing import List, Optional, Dict, Any


class H5FileManager:
    """
    H5文件管理器类
    
    管理多个h5文件，在每次请求时自动切换到下一个文件。
    """
    
    def __init__(self, 
                 directory: str = "/home/alan/桌面/UMI_replay_数据/正常",
                 file_pattern: str = "banana*.h5",
                 mode: str = "sequential",
                 diff_postprocess: bool = True,
                 max_files: int = 100):
        """
        初始化H5文件管理器
        
        Args:
            directory: h5文件所在目录
            file_pattern: 文件匹配模式（glob格式）
            mode: 文件切换模式，可选值：
                - "sequential": 顺序模式（默认），按文件名顺序切换
                - "round_robin": 循环模式，循环使用所有文件
                - "random": 随机模式，随机选择文件
            diff_postprocess: 是否应用差分后处理
            max_files: 最大文件数量限制
        """
        self.directory = directory
        self.file_pattern = file_pattern
        self.mode = mode
        self.diff_postprocess = diff_postprocess
        self.max_files = max_files
        
        # 线程锁，确保线程安全
        self.lock = threading.Lock()
        
        # 状态变量
        self.request_count = 0
        self.current_file_index = 0
        self.h5_files = []
        
        # 初始化：扫描h5文件
        self._scan_h5_files()
        
        print(f"[H5FileManager] 初始化完成")
        print(f"  目录: {directory}")
        print(f"  文件模式: {file_pattern}")
        print(f"  找到文件数: {len(self.h5_files)}")
        print(f"  切换模式: {mode}")
        print(f"  差分后处理: {diff_postprocess}")
    
    def _scan_h5_files(self) -> None:
        """
        扫描指定目录下的h5文件
        
        使用glob模式匹配文件，并按文件名排序。
        """
        if not os.path.exists(self.directory):
            print(f"[H5FileManager] 警告：目录不存在: {self.directory}")
            self.h5_files = []
            return
        
        # 使用glob模式匹配文件
        search_pattern = os.path.join(self.directory, self.file_pattern)
        files = glob.glob(search_pattern)
        
        # 按文件名排序（确保顺序一致性）
        files.sort()
        
        # 限制最大文件数量
        if len(files) > self.max_files:
            print(f"[H5FileManager] 警告：找到{len(files)}个文件，限制为前{self.max_files}个")
            files = files[:self.max_files]
        
        self.h5_files = files
        
        if self.h5_files:
            print(f"[H5FileManager] 在 {self.directory} 中找到 {len(self.h5_files)} 个h5文件")
            for i, file_path in enumerate(self.h5_files[:5]):  # 显示前5个文件
                print(f"  {i+1}. {os.path.basename(file_path)}")
            if len(self.h5_files) > 5:
                print(f"  ... 和 {len(self.h5_files)-5} 个其他文件")
        else:
            print(f"[H5FileManager] 警告：未找到匹配的h5文件")
    
    def _read_from_file(self, file_path: str) -> np.ndarray:
        """
        从h5文件中读取动作轨迹
        
        基于test_http_server.py中的read_from_file函数实现。
        
        Args:
            file_path: h5文件路径
            
        Returns:
            numpy数组形状 (n, 7) 包含n个动作，每个动作7个维度
            [dx, dy, dz, rx, ry, rz, gripper]
        """
        try:
            print(f"[H5FileManager] 读取文件: {file_path}")
            
            with h5py.File(file_path, 'r') as f:
                # 检查文件中的数据集
                if 'action' in f:
                    actions = f['action'][:]
                    print(f"[H5FileManager] 找到action数据集，形状: {actions.shape}")
                    
                    # 检查动作维度
                    if len(actions.shape) == 2:
                        # 如果动作有8个维度，取前7个
                        if actions.shape[1] == 8:
                            print(f"[H5FileManager] 动作有8个维度，取前7个")
                            actions = actions[:, :7]
                        elif actions.shape[1] == 7:
                            print(f"[H5FileManager] 动作有7个维度，直接使用")
                        else:
                            print(f"[H5FileManager] 警告: 动作维度为{actions.shape[1]}，期望7或8")
                            # 如果维度不匹配，尝试取前7列
                            if actions.shape[1] > 7:
                                actions = actions[:, :7]
                            else:
                                # 如果维度不足，补零
                                print(f"[H5FileManager] 警告: 维度不足，补零到7维")
                                new_actions = np.zeros((actions.shape[0], 7))
                                new_actions[:, :actions.shape[1]] = actions
                                actions = new_actions

                        # 如果设置差分后处理
                        if self.diff_postprocess:
                            print(f"[H5FileManager] 应用差分后处理（4x4齐次变换矩阵逻辑）")
                            # 使用4x4齐次变换矩阵计算差分
                            # 第0步保持原样，第1步及以后计算相对变换
                            new_actions = np.zeros([actions.shape[0], 6])
                            
                            for i in range(1, actions.shape[0]):
                                # 提取前一个动作的平移和旋转
                                t_prev = actions[i-1, 0:3]  # 平移: [dx, dy, dz]
                                q_prev = actions[i-1, 3:7]  # 四元数: [qx, qy, qz, qw]
                                
                                # 提取当前动作的平移和旋转
                                t_curr = actions[i, 0:3]    # 平移: [dx, dy, dz]
                                q_curr = actions[i, 3:7]    # 四元数: [qx, qy, qz, qw]
                                
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
                            
                            # 将差分后的动作赋值回actions变量
                            actions = new_actions
                            print(f"[H5FileManager] 差分后动作形状: {actions.shape}")
                        
                        # 限制动作数量，避免返回太多动作
                        max_actions = 1000  # 限制最大动作数
                        if actions.shape[0] > max_actions:
                            print(f"[H5FileManager] 动作数量{actions.shape[0]}超过限制{max_actions}，取前{max_actions}个")
                            actions = actions[:max_actions, :]
                        
                        # 确保返回7维动作（如果差分后处理返回6维，补零第7维）
                        if actions.shape[1] == 6:
                            print(f"[H5FileManager] 差分后动作为6维，补零第7维（夹爪）")
                            new_actions = np.zeros((actions.shape[0], 7))
                            new_actions[:, :6] = actions
                            actions = new_actions
                        
                        print(f"[H5FileManager] 返回动作形状: {actions.shape}")
                        return actions
                    else:
                        print(f"[H5FileManager] 错误: action数据集形状不是2D: {actions.shape}")
                        return np.zeros((25, 7))
                else:
                    # 尝试其他常见名称
                    action_keys = ['actions', 'traj', 'trajectory', 'act', 'motion', 'states']
                    for key in action_keys:
                        if key in f:
                            actions = f[key][:]
                            print(f"[H5FileManager] 找到{key}数据集，形状: {actions.shape}")
                            
                            # 处理动作数据
                            if len(actions.shape) == 2 and actions.shape[1] >= 7:
                                actions = actions[:, :7]
                                # 限制动作数量
                                max_actions = 1000
                                if actions.shape[0] > max_actions:
                                    actions = actions[:max_actions, :]
                                print(f"[H5FileManager] 返回动作形状: {actions.shape}")
                                return actions
                    
                    print(f"[H5FileManager] 错误: 文件中未找到动作数据集")
                    print(f"[H5FileManager] 可用键: {list(f.keys())}")
                    return np.zeros((25, 7))
                    
        except Exception as e:
            print(f"[H5FileManager] 读取文件错误: {e}")
            import traceback
            traceback.print_exc()
            return np.zeros((25, 7))
    
    def _select_next_file(self) -> Optional[str]:
        """
        根据当前模式和请求计数选择下一个文件
        
        Returns:
            选中的h5文件路径，或None（如果没有可用文件）
        """
        with self.lock:
            if not self.h5_files:
                print(f"[H5FileManager] 警告：没有可用的h5文件")
                return None
            
            current_count = self.request_count
            self.request_count += 1
            
            if self.mode == "sequential":
                # 顺序模式：每个请求使用不同的文件
                file_index = current_count % len(self.h5_files)
                selected_file = self.h5_files[file_index]
                self.current_file_index = file_index
                print(f"[H5FileManager] 顺序模式：请求#{current_count} → 文件#{file_index}: {os.path.basename(selected_file)}")
                
            elif self.mode == "round_robin":
                # 循环模式：循环使用所有文件
                # 每个文件服务固定数量的请求后切换到下一个文件
                requests_per_file = 1  # 每个文件服务1次请求
                file_index = (current_count // requests_per_file) % len(self.h5_files)
                selected_file = self.h5_files[file_index]
                self.current_file_index = file_index
                print(f"[H5FileManager] 循环模式：请求#{current_count} → 文件#{file_index}: {os.path.basename(selected_file)}")
                
            elif self.mode == "random":
                # 随机模式：随机选择文件
                file_index = random.randint(0, len(self.h5_files) - 1)
                selected_file = self.h5_files[file_index]
                self.current_file_index = file_index
                print(f"[H5FileManager] 随机模式：请求#{current_count} → 文件#{file_index}: {os.path.basename(selected_file)}")
                
            else:
                # 默认使用第一个文件
                selected_file = self.h5_files[0]
                self.current_file_index = 0
                print(f"[H5FileManager] 默认模式：使用第一个文件: {os.path.basename(selected_file)}")
            
            return selected_file
    
    def get_next_action_sequence(self) -> np.ndarray:
        """
        获取下一个动作序列
        
        自动选择下一个h5文件，读取并返回其动作序列。
        
        Returns:
            numpy数组形状 (n, 7) 包含n个动作
        """
        # 选择下一个文件
        selected_file = self._select_next_file()
        
        if selected_file and os.path.exists(selected_file):
            # 读取文件中的动作序列
            action_sequence = self._read_from_file(selected_file)
            return action_sequence
        else:
            print(f"[H5FileManager] 警告：文件不存在或未指定，返回零动作序列")
            return np.zeros((25, 7))
    
    def get_current_file_info(self) -> Dict[str, Any]:
        """
        获取当前文件信息
        
        Returns:
            包含当前文件信息的字典
        """
        with self.lock:
            if not self.h5_files or self.current_file_index >= len(self.h5_files):
                return {
                    "file_path": None,
                    "file_name": None,
                    "file_index": -1,
                    "total_files": len(self.h5_files)
                }
            
            current_file = self.h5_files[self.current_file_index]
            return {
                "file_path": current_file,
                "file_name": os.path.basename(current_file),
                "file_index": self.current_file_index,
                "total_files": len(self.h5_files)
            }
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取管理器状态
        
        Returns:
            包含管理器状态的字典
        """
        with self.lock:
            status = {
                "request_count": self.request_count,
                "current_file_index": self.current_file_index,
                "total_files": len(self.h5_files),
                "mode": self.mode,
                "directory": self.directory,
                "file_pattern": self.file_pattern,
                "diff_postprocess": self.diff_postprocess,
                "available_files": [os.path.basename(f) for f in self.h5_files[:10]],  # 只显示前10个
            }
        
        return status
    
    def reload_files(self) -> Dict[str, Any]:
        """
        重新扫描并加载h5文件
        
        Returns:
            包含重新加载结果的字典
        """
        with self.lock:
            old_count = len(self.h5_files)
            self._scan_h5_files()
            new_count = len(self.h5_files)
            
            # 重置索引，避免越界
            if self.current_file_index >= new_count and new_count > 0:
                self.current_file_index = 0
        
        return {
            "status": "success",
            "message": f"重新加载了 {new_count} 个h5文件（之前: {old_count}）",
            "old_file_count": old_count,
            "new_file_count": new_count
        }
    
    def switch_mode(self, new_mode: str) -> Dict[str, Any]:
        """
        切换文件切换模式
        
        Args:
            new_mode: 新的切换模式
            
        Returns:
            包含切换结果的字典
        """
        valid_modes = ["sequential", "round_robin", "random"]
        
        if new_mode not in valid_modes:
            return {
                "status": "error",
                "message": f"无效模式，有效值: {valid_modes}",
                "current_mode": self.mode
            }
        
        with self.lock:
            old_mode = self.mode
            self.mode = new_mode
        
        return {
            "status": "success",
            "message": f"切换模式成功: {old_mode} -> {new_mode}",
            "old_mode": old_mode,
            "new_mode": new_mode
        }
    
    def reset_counter(self) -> Dict[str, Any]:
        """
        重置请求计数器
        
        Returns:
            包含重置结果的字典
        """
        with self.lock:
            old_count = self.request_count
            self.request_count = 0
            self.current_file_index = 0
        
        return {
            "status": "success",
            "message": f"请求计数器已重置（原值: {old_count}）",
            "old_count": old_count,
            "new_count": self.request_count
        }
    
    def get_file_list(self) -> List[str]:
        """
        获取所有h5文件列表
        
        Returns:
            h5文件路径列表
        """
        with self.lock:
            return self.h5_files.copy()
    
    def get_file_count(self) -> int:
        """
        获取h5文件数量
        
        Returns:
            h5文件数量
        """
        with self.lock:
            return len(self.h5_files)
