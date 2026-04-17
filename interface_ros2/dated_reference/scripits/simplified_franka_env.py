#!/usr/bin/env python3
"""
中国某机器人公司 G组 端侧遥操作软件开发
李佩泽 LI Peize lipeize@agibot.com
环境：简易Franka接口 v0.0.1
"""


import rospy
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

class SimplifiedFrankaEnv:
    """
    简化的Franka环境类
    提供Gym风格的接口，用于机器人控制
    作为ExpEnvInteract节点的成员类使用
    """
    
    def __init__(self, debug=True):
        """
        初始化SimplifiedFrankaEnv
        
        Args:
            debug: 是否启用调试输出
        """
        self.debug = debug
        
        # 初始化成员属性
        self.current_pose = None  # 字典: {'position': np.array, 'orientation': R}
        self.current_force = np.zeros(6)  # [Fx, Fy, Fz, Tx, Ty, Tz]
        self.target_pose = None  # 字典: {'position': np.array, 'orientation': R}
        
        # 发布器 - 用于发布目标位姿到/cartesian_impedance_example_controller/equilibrium_pose
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose', 
            PoseStamped, 
            queue_size=10
        )
        
        # 订阅器 - 用于接收机器人状态
        self.state_sub = rospy.Subscriber(
            'franka_state_controller/franka_states',
            FrankaState,
            self.state_callback,
            queue_size=10
        )
        
        # 等待第一个状态消息以确保数据可用
        rospy.loginfo("等待接收机器人状态...")
        rospy.wait_for_message('franka_state_controller/franka_states', FrankaState)
        rospy.loginfo("SimplifiedFrankaEnv已初始化")
    
    def state_callback(self, msg):
        """
        Franka状态回调函数
        用O_T_EE更新self.current_pose，用K_F_ext_hat_K更新self.current_force
        
        Args:
            msg: FrankaState消息
        """
        try:
            # 从O_T_EE变换矩阵提取位置和姿态
            transform_matrix = np.reshape(msg.O_T_EE, (4, 4))
            
            # 提取位置 (3维向量)
            position = transform_matrix[3, :3]
            
            # 提取旋转矩阵并创建scipy Rotation对象
            inv_rotation_matrix = transform_matrix[:3, :3]
            orientation = R.from_matrix(inv_rotation_matrix).inv()

            # 更新current_pose
            self.current_pose = {
                'position': position,
                'orientation': orientation
            }
            # print(self.current_pose)
            
            # 从K_F_ext_hat_K更新current_force (6维向量)
            self.current_force = np.array([
                msg.K_F_ext_hat_K[0],  # Fx
                msg.K_F_ext_hat_K[1],  # Fy
                msg.K_F_ext_hat_K[2],  # Fz
                msg.K_F_ext_hat_K[3],  # Tx
                msg.K_F_ext_hat_K[4],  # Ty
                msg.K_F_ext_hat_K[5]   # Tz
            ])
            
            if self.debug and hasattr(self, 'state_count'):
                self.state_count += 1
                if self.state_count % 100 == 0:
                    rospy.loginfo(f"已接收 {self.state_count} 条状态消息")
            else:
                self.state_count = 1
            
        except Exception as e:
            rospy.logerr(f"处理状态消息时出错: {e}")
    
    def step(self, action=None, buttons=None):
        """
        Gym标准格式的step方法
        
        Args:
            action: 可选的动作输入，用于retarget算法
            buttons: 可选的按钮状态，用于retarget算法
            
        Returns:
            tuple: (observation, reward, done, info) - Gym标准格式
        """
        try:
            # 检查数据是否可用
            if self.current_pose is None:
                rospy.logwarn("当前位姿数据不可用，跳过step")
                return self._get_observation(), 0.0, False, {}
            
            # 调用retarget函数更新target_pose
            self.retarget(action, buttons)
            
            # 发布更新后的target_pose到equilibrium_pose
            self._publish_target_pose()
            
            # 返回Gym标准格式
            observation = self._get_observation()
            reward = 0.0  # 您可以根据需要实现奖励函数
            done = False  # 您可以根据需要实现终止条件
            info = {}     # 额外的信息字典
            
            if self.debug:
                rospy.logdebug(f"Step完成 - 观测: {observation.shape}, 奖励: {reward}")
            
            return observation, reward, done, info
            
        except Exception as e:
            rospy.logerr(f"Step方法出错: {e}")
            return self._get_observation(), 0.0, False, {'error': str(e)}
    
    def retarget(self, action=None, buttons=None):
        """
        Retarget函数 - 实现复杂的retarget算法
        
        Args:
            action: 6维动作向量 [dx, dy, dz, drx, dry, drz]
            buttons: 按钮状态列表 [button1_state]
        """
        try:
            # 检查数据是否可用
            if self.current_pose is None:
                rospy.logwarn("当前位姿数据不可用，无法进行retarget")
                return
            
            # 如果没有动作输入，保持当前位置
            if action is None:
                self.target_pose = copy.deepcopy(self.current_pose)
                return
            
            # 初始化目标位姿
            if self.target_pose is None:
                self.target_pose = copy.deepcopy(self.current_pose)
            
            # 保存上一次的目标位姿
            last_target_pose = copy.deepcopy(self.target_pose)
            
            # 更新位置部分
            # 使用动作的前3维更新位置
            new_position = last_target_pose['position'] + action[:3]
            
            # 更新姿态部分
            # 获取上一次目标姿态的旋转对象
            last_target_rotation = last_target_pose['orientation']
            
            # 将动作的旋转向量转换到目标坐标系
            action_rotvec_in_target_frame = last_target_rotation.as_matrix().T @ action[3:6]
            
            # 创建动作对应的旋转
            action_rotation = R.from_rotvec(action_rotvec_in_target_frame)
            
            # 计算新的目标姿态
            new_target_rotation = last_target_rotation * action_rotation
            
            # 更新目标位姿
            self.target_pose = {
                'position': new_position,
                'orientation': new_target_rotation
            }
            
            # # 按钮重置逻辑 (可选)
            # if buttons is not None and len(buttons) > 1 and not buttons[1]:
            #     # 如果按钮2释放，重置目标位姿到当前位置
            #     self.target_pose = copy.deepcopy(self.current_pose)
            #     if self.debug:
            #         rospy.loginfo("目标位姿已重置到当前位置")
            
            if self.debug:
                rospy.logdebug(f"Retarget完成:")
                rospy.logdebug(f"  - 动作: {action}")
                rospy.logdebug(f"  - 目标位置: {self.target_pose['position']}")
                rospy.logdebug(f"  - 目标姿态(四元数): {self.target_pose['orientation'].as_quat()}")
                
        except Exception as e:
            rospy.logerr(f"Retarget函数出错: {e}")
            # 出错时保持当前位置
            if self.current_pose is not None:
                self.target_pose = copy.deepcopy(self.current_pose)
    
    def reset(self):
        """
        Gym标准格式的reset方法
        
        Returns:
            observation: 初始观测值
        """
        try:
            # 检查数据是否可用
            if self.current_pose is None:
                rospy.logwarn("当前位姿数据不可用，无法重置")
                return self._get_observation()
            
            # 将current_pose深拷贝给target_pose
            self.target_pose = copy.deepcopy(self.current_pose)
            
            # 将current_pose发布到equilibrium_pose
            self._publish_pose(self.current_pose)
            
            rospy.loginfo("环境已重置 - 目标位姿设置为当前位置")
            
            if self.debug:
                rospy.logdebug(f"Reset完成 - 当前位置: {self.current_pose['position']}")
            
            return self._get_observation()
            
        except Exception as e:
            rospy.logerr(f"Reset方法出错: {e}")
            return self._get_observation()
    
    def _publish_target_pose(self):
        """发布target_pose到equilibrium_pose话题"""
        if self.target_pose is not None:
            self._publish_pose(self.target_pose)
    
    def _publish_pose(self, pose_dict):
        """
        发布位姿到equilibrium_pose话题
        
        Args:
            pose_dict: 包含'position'和'orientation'的字典
        """
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "panda_link0"  # 基坐标系
            
            # 设置位置
            pose_msg.pose.position.x = pose_dict['position'][0]
            pose_msg.pose.position.y = pose_dict['position'][1]
            pose_msg.pose.position.z = pose_dict['position'][2]
            
            # 设置姿态 (从scipy Rotation转换为四元数)
            quaternion = pose_dict['orientation'].as_quat()  # [x, y, z, w]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            
            self.pose_pub.publish(pose_msg)
            
            if self.debug:
                rospy.logdebug(f"已发布位姿到equilibrium_pose - 位置: {pose_dict['position']}")
                
        except Exception as e:
            rospy.logerr(f"发布位姿时出错: {e}")
    
    def _get_observation(self):
        """
        获取当前观测值
        
        Returns:
            np.array: 观测向量，包含位置、姿态和力信息
        """
        try:
            if self.current_pose is None:
                # 如果数据不可用，返回零向量
                return np.zeros(13)  # 3位置 + 4四元数 + 6力
            
            # 从scipy Rotation获取四元数
            quaternion = self.current_pose['orientation'].as_quat()  # [x, y, z, w]
            
            # 构建观测向量: [位置(3), 四元数(4), 力(6)]
            observation = np.concatenate([
                self.current_pose['position'],  # 3维
                quaternion,                     # 4维
                self.current_force              # 6维
            ])
            
            return observation
            
        except Exception as e:
            rospy.logerr(f"获取观测值时出错: {e}")
            return np.zeros(13)
    
    def get_current_pose(self):
        """
        获取当前位姿
        
        Returns:
            dict: 当前位姿字典，包含'position'和'orientation'
        """
        return copy.deepcopy(self.current_pose) if self.current_pose is not None else None
    
    def get_current_force(self):
        """
        获取当前力信息
        
        Returns:
            np.array: 当前6维力向量
        """
        return self.current_force.copy()
    
    def get_target_pose(self):
        """
        获取目标位姿
        
        Returns:
            dict: 目标位姿字典，包含'position'和'orientation'
        """
        return copy.deepcopy(self.target_pose) if self.target_pose is not None else None
    
    def cleanup(self):
        """清理资源"""
        self.state_sub.unregister()
        rospy.loginfo("SimplifiedFrankaEnv资源已清理")


# 使用示例
if __name__ == "__main__":
    # 测试代码
    rospy.init_node('simplified_franka_env_test')
    
    env = SimplifiedFrankaEnv(debug=True)
    
    try:
        # 测试reset
        observation = env.reset()
        rospy.loginfo(f"Reset后观测: {observation}")
        
        # 测试step
        rate = rospy.Rate(1)  # 1Hz
        for i in range(5):
            observation, reward, done, info = env.step()
            rospy.loginfo(f"Step {i}: 观测长度={len(observation)}, 奖励={reward}, 完成={done}")
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("测试程序被中断")
    finally:
        env.cleanup()
