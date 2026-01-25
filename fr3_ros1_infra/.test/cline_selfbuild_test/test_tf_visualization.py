#!/usr/bin/env python3
"""
坐标可视化脚本 - 从h5文件读取位姿并通过tf可视化
要求：
1. 30帧/s
2. tf关系父系为fr3_link0，子系为umi_target
3. 直接获得h5文件中actions的位姿不需要diff处理，只需要将四元数转为tf
4. 循环播放
5. 不需要mock等功能，最小实现
"""

import rospy
import tf
import numpy as np
import h5py
import time
import sys
import os

def read_from_file(dir_to_file):
    """
    从h5文件中读取动作轨迹
    
    Args:
        dir_to_file: h5文件路径
        
    Returns:
        numpy数组形状 (n, 8) 包含n个位姿，每个位姿8个维度
        [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w, gripper]
    """
    try:
        rospy.loginfo(f"[read_from_file] 读取文件: {dir_to_file}")
        
        with h5py.File(dir_to_file, 'r') as f:
            # 检查文件中的数据集
            if 'action' in f:
                actions = f['action'][:]
                rospy.loginfo(f"[read_from_file] 找到action数据集，形状: {actions.shape}")
                
                # 检查动作维度
                if len(actions.shape) == 2:
                    # 如果动作有8个维度，直接使用
                    if actions.shape[1] == 8:
                        rospy.loginfo(f"[read_from_file] 动作有8个维度，直接使用")
                        return actions
                    elif actions.shape[1] == 7:
                        rospy.loginfo(f"[read_from_file] 动作有7个维度，补零到8维")
                        # 补零到8维 [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w, gripper]
                        new_actions = np.zeros((actions.shape[0], 8))
                        new_actions[:, :7] = actions
                        return new_actions
                    else:
                        rospy.logwarn(f"[read_from_file] 警告: 动作维度为{actions.shape[1]}，期望7或8")
                        # 尝试处理
                        if actions.shape[1] > 8:
                            actions = actions[:, :8]
                        elif actions.shape[1] > 7:
                            # 补零到8维
                            new_actions = np.zeros((actions.shape[0], 8))
                            new_actions[:, :actions.shape[1]] = actions
                            actions = new_actions
                        else:
                            # 补零到8维
                            new_actions = np.zeros((actions.shape[0], 8))
                            new_actions[:, :actions.shape[1]] = actions
                            actions = new_actions
                        return actions
                else:
                    rospy.logerr(f"[read_from_file] 错误: action数据集形状不是2D: {actions.shape}")
                    return np.zeros((25, 8))
            else:
                # 尝试其他常见名称
                action_keys = ['actions', 'traj', 'trajectory', 'act', 'motion']
                for key in action_keys:
                    if key in f:
                        actions = f[key][:]
                        rospy.loginfo(f"[read_from_file] 找到{key}数据集，形状: {actions.shape}")
                        
                        # 处理动作数据
                        if len(actions.shape) == 2 and actions.shape[1] >= 7:
                            if actions.shape[1] >= 8:
                                actions = actions[:, :8]
                            else:
                                # 补零到8维
                                new_actions = np.zeros((actions.shape[0], 8))
                                new_actions[:, :actions.shape[1]] = actions
                                actions = new_actions
                            rospy.loginfo(f"[read_from_file] 返回动作形状: {actions.shape}")
                            return actions
                
                rospy.logerr(f"[read_from_file] 错误: 文件中未找到动作数据集")
                rospy.logerr(f"[read_from_file] 可用键: {list(f.keys())}")
                return np.zeros((25, 8))
                
    except Exception as e:
        rospy.logerr(f"[read_from_file] 读取文件错误: {e}")
        import traceback
        traceback.print_exc()
        return np.zeros((25, 8))

class TFVisualizationNode:
    def __init__(self, h5_file_path, loop=True, rate_hz=30):
        """
        初始化TF可视化节点
        
        Args:
            h5_file_path: h5文件路径
            loop: 是否循环播放
            rate_hz: 发布频率 (Hz)
        """
        rospy.init_node('tf_visualization_node')
        
        # 读取h5文件中的位姿数据
        self.poses = read_from_file(h5_file_path)
        rospy.loginfo(f"读取到 {len(self.poses)} 个位姿")
        
        if len(self.poses) == 0:
            rospy.logerr("未读取到位姿数据，节点将退出")
            sys.exit(1)
        
        # 检查数据维度
        if self.poses.shape[1] < 7:
            rospy.logerr(f"位姿数据维度不足: {self.poses.shape[1]}，至少需要7维")
            sys.exit(1)
        
        # 创建TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 设置参数
        self.loop = loop
        self.rate = rospy.Rate(rate_hz)
        self.current_index = 0
        
        # 父坐标系和子坐标系
        self.parent_frame = "fr3_link0"
        self.child_frame = "umi_target"
        
        rospy.loginfo(f"TF可视化节点已初始化")
        rospy.loginfo(f"  文件: {h5_file_path}")
        rospy.loginfo(f"  位姿数量: {len(self.poses)}")
        rospy.loginfo(f"  频率: {rate_hz} Hz")
        rospy.loginfo(f"  循环播放: {loop}")
        rospy.loginfo(f"  TF关系: {self.parent_frame} -> {self.child_frame}")
    
    def publish_tf(self):
        """发布当前位姿的TF变换"""
        try:
            # 获取当前位姿
            pose = self.poses[self.current_index]
            
            # 提取位置 (前3个元素)
            position = pose[:3]
            
            # 提取四元数 (第4-7个元素)
            # 注意：h5文件中的四元数顺序可能是 [x, y, z, w]
            if len(pose) >= 7:
                # 如果有7个或更多元素，假设第4-7个是四元数
                quaternion = pose[3:7]
                
                # 确保四元数归一化
                quaternion_norm = np.linalg.norm(quaternion)
                if quaternion_norm > 0:
                    quaternion = quaternion / quaternion_norm
                else:
                    # 如果四元数为零，使用单位四元数
                    quaternion = np.array([0.0, 0.0, 0.0, 1.0])
            else:
                # 如果维度不足，使用单位四元数
                quaternion = np.array([0.0, 0.0, 0.0, 1.0])
            
            # 获取当前时间
            current_time = rospy.Time.now()
            
            # 发布TF变换
            self.tf_broadcaster.sendTransform(
                translation=(position[0], position[1], position[2]),
                rotation=(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                time=current_time,
                child=self.child_frame,
                parent=self.parent_frame
            )
            
            # 记录调试信息（每100帧记录一次）
            if self.current_index % 100 == 0:
                rospy.logdebug(f"发布TF变换 [{self.current_index}/{len(self.poses)}]: "
                              f"位置={position}, 四元数={quaternion}")
            
        except Exception as e:
            rospy.logerr(f"发布TF变换时出错: {e}")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("开始发布TF变换...")
        
        try:
            while not rospy.is_shutdown():
                # 发布当前位姿的TF变换
                self.publish_tf()
                
                # 移动到下一个位姿
                self.current_index += 1
                
                # 检查是否到达末尾
                if self.current_index >= len(self.poses):
                    if self.loop:
                        rospy.loginfo("到达轨迹末尾，循环播放...")
                        self.current_index = 0
                    else:
                        rospy.loginfo("到达轨迹末尾，节点将退出")
                        break
                
                # 保持发布频率
                self.rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS中断，节点停止")
        except Exception as e:
            rospy.logerr(f"运行循环出错: {e}")
        
        rospy.loginfo("TF可视化节点已停止")

def main():
    """主函数"""
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='从h5文件读取位姿并通过TF可视化')
    parser.add_argument('--file', type=str, 
                       default='/home/alan/桌面/UMI_replay_数据/正常/banana1.h5',
                       help='h5文件路径 (默认: /home/alan/桌面/UMI_replay_数据/正常/banana1.h5)')
    parser.add_argument('--rate', type=float, default=30.0,
                       help='发布频率 (Hz) (默认: 30)')
    parser.add_argument('--no-loop', action='store_true',
                       help='不循环播放 (默认: 循环播放)')
    parser.add_argument('--parent', type=str, default='fr3_link0',
                       help='父坐标系 (默认: fr3_link0)')
    parser.add_argument('--child', type=str, default='umi_target',
                       help='子坐标系 (默认: umi_target)')
    
    args = parser.parse_args()
    
    # 创建节点
    node = TFVisualizationNode(
        h5_file_path=args.file,
        loop=not args.no_loop,
        rate_hz=args.rate
    )
    
    # 设置父坐标系和子坐标系
    node.parent_frame = args.parent
    node.child_frame = args.child
    
    # 运行节点
    node.run()

if __name__ == '__main__':
    main()