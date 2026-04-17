#!/usr/bin/env python3

"""
中国某机器人公司 G组 端侧遥操作软件开发
李佩泽 LI Peize lipeize@agibot.com
专家输入：haption6d残差映射拖拽 v0.0.1
"""

import rospy
import numpy as np
import copy
import threading
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import WrenchStamped

# 导入Virtuose消息类型
try:
    from exp_env_interact.msg import out_virtuose_pose, out_virtuose_status
    rospy.loginfo("成功导入Virtuose消息类型")
except ImportError as e:
    rospy.logerr(f"无法导入Virtuose消息类型: {e}")
    rospy.logerr("请确保已编译消息文件")
    import sys
    sys.exit(1)

class Haption6DExpert:
    """
    Haption 6D专家控制类
    直接订阅Virtuose ROS话题，实现专家控制逻辑
    作为ExpEnvInteract节点的成员类使用
    """
    
    def __init__(self, debug=True):
        """
        初始化Haption6DExpert
        
        Args:
            debug: 是否启用调试输出
        """
        self.debug = debug
        
        # 状态机
        self.fsm = "idle"
        
        # 当前和上一次的位姿和按钮数据
        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])  # [x, y, z, qx, qy, qz, qw]
        self.current_buttons = np.array([False, False, False])  # [button1, button2, button3]
        self.last_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.last_buttons = np.array([False, False, False])
        self.tmp_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.tmp_buttons = np.array([False, False, False])
        
        # 动作输出
        self.action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [dx, dy, dz, drx, dry, drz]
        
        # 数据锁
        self.data_lock = threading.Lock()
        
        # 订阅ROS话题
        self.pose_sub = rospy.Subscriber(
            '/out_virtuose_pose', 
            out_virtuose_pose, 
            self.pose_callback,
            queue_size=10
        )
        
        self.status_sub = rospy.Subscriber(
            '/out_virtuose_status', 
            out_virtuose_status, 
            self.status_callback,
            queue_size=10
        )
        
        # 统计信息
        self.pose_count = 0
        self.status_count = 0
        
        rospy.loginfo("Haption6DExpert已初始化")
    
    def pose_callback(self, msg):
        """
        Virtuose位姿回调函数
        
        Args:
            msg: out_virtuose_physical_pose消息
        """
        try:
            # 从ROS消息中提取pose数据
            x = msg.virtuose_pose.translation.x
            y = msg.virtuose_pose.translation.y
            z = msg.virtuose_pose.translation.z
            qx = msg.virtuose_pose.rotation.x
            qy = msg.virtuose_pose.rotation.y
            qz = msg.virtuose_pose.rotation.z
            qw = msg.virtuose_pose.rotation.w
            
            # 更新当前位姿
            with self.data_lock:
                self.tmp_pose = np.array([x, y, z, qx, qy, qz, qw])
            
            # 更新统计信息
            self.pose_count += 1
            
            if self.debug and self.pose_count % 5000 == 0:
                rospy.loginfo(f"已接收 {self.pose_count} 条pose消息")
            
        except Exception as e:
            rospy.logerr(f"处理pose消息时出错: {e}")
    
    def status_callback(self, msg):
        """
        Virtuose状态回调函数
        
        Args:
            msg: out_virtuose_status消息
        """
        try:
            # 从ROS消息中提取buttons数据
            buttons = msg.buttons
            
            # 解析按钮状态
            button1 = (buttons & 0x01) != 0
            button2 = (buttons & 0x02) != 0
            button3 = (buttons & 0x04) != 0
            

            # 更新当前按钮状态
            with self.data_lock:
                self.tmp_buttons = np.array([button1, button2, button3])
            
            # 更新统计信息
            self.status_count += 1
            
            if self.debug and self.status_count % 5000 == 0:
                rospy.loginfo(f"已接收 {self.status_count} 条status消息")
            
        except Exception as e:
            rospy.logerr(f"处理status消息时出错: {e}")
    
    def get_action(self, obs=None):
        """
        获取当前动作
        
        Args:
            obs: 观测值，仅作为输入参数，不进行处理
            
        Returns:
            tuple: (action_vector, button_state)
                   action_vector: 6维动作向量 [dx, dy, dz, drx, dry, drz]
                   button_state: 按钮状态列表 [button1_state]
        """
        self.update_buttons()
        self.update_fsm()
        self.update_pose()
        self.update_action()
        
        return [self.action, [self.current_buttons[0]]]
    
    def update_buttons(self):
        """更新按钮状态"""
        self.current_buttons = self.tmp_buttons
        if self.debug:
            rospy.logdebug(f'FSM状态: {self.fsm}')
            rospy.logdebug(f'当前按钮: {self.current_buttons}')
        
    
    def update_fsm(self):
        """更新状态机"""
        if self.fsm == "idle":
            if self.current_buttons[1]:  # 按钮2按下
                self.fsm = "project_diff"
        elif self.fsm == "project_diff": 
            if not self.current_buttons[1]:  # 按钮2释放
                self.fsm = "idle"
        else:
            self.fsm = "idle"
    
    def update_pose(self):
        """更新位姿数据"""
        if self.fsm == "idle":
            with self.data_lock:
                self.current_pose = copy.deepcopy(self.tmp_pose)
                self.last_pose = copy.deepcopy(self.current_pose)
            if self.debug:
                rospy.logdebug(f'FSM状态: {self.fsm}')
                rospy.logdebug(f'当前位姿: {self.current_pose}')
                rospy.logdebug(f'上次位姿: {self.last_pose}')
        elif self.fsm == "project_diff":
            with self.data_lock:
                self.last_pose = copy.deepcopy(self.current_pose)
                self.current_pose = copy.deepcopy(self.tmp_pose)
            if self.debug:
                rospy.logdebug(f'FSM状态: {self.fsm}')
                rospy.logdebug(f'上次位姿: {self.last_pose}')
                rospy.logdebug(f'当前位姿: {self.current_pose}')
    
    def normalize_quaternion(self, quat):
        """
        归一化四元数
        
        Args:
            quat (np.array): 四元数 [qx, qy, qz, qw]
            
        Returns:
            np.array: 归一化后的四元数
        """
        norm = np.linalg.norm(quat)
        if norm > 1e-8:  # 避免除零
            return quat / norm
        else:
            return np.array([0.0, 0.0, 0.0, 1.0])  # 默认单位四元数
    
    def update_action(self):
        """更新动作向量"""
        if self.fsm == "project_diff":
            current_translation = self.current_pose[0:3]
            last_translation = self.last_pose[0:3]
            
            # 归一化四元数
            current_quat_normalized = self.normalize_quaternion(self.current_pose[3:7])
            last_quat_normalized = self.normalize_quaternion(self.last_pose[3:7])
            
            # 创建旋转对象
            current_orientation = R.from_quat(current_quat_normalized)
            last_orientation = R.from_quat(last_quat_normalized)
            
            # 计算平移变化
            delta_translation_space = current_translation - last_translation
            
            # 计算旋转变化
            delta_orientation_body = R.from_matrix(
                last_orientation.as_matrix().T 
                @ current_orientation.as_matrix()
            ).as_rotvec()
            delta_orientation_space = last_orientation.as_matrix() @ delta_orientation_body
            
            # 更新动作向量
            self.action = np.hstack([delta_translation_space, delta_orientation_space])
            
            if self.debug:
                rospy.logdebug(f'动作向量: {self.action}')
        elif self.fsm == "idle":
            self.action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    def get_current_pose(self):
        """
        获取当前位姿
        
        Returns:
            np.array: 当前位姿 [x, y, z, qx, qy, qz, qw]
        """
        with self.data_lock:
            return self.current_pose.copy()
    
    def get_current_buttons(self):
        """
        获取当前按钮状态
        
        Returns:
            np.array: 当前按钮状态 [button1, button2, button3]
        """
        with self.data_lock:
            return self.current_buttons.copy()
    
    def cleanup(self):
        """清理资源"""
        self.pose_sub.unregister()
        self.status_sub.unregister()
        rospy.loginfo(f"Haption6DExpert资源已清理 - 总共接收: Pose({self.pose_count}), Status({self.status_count})")


# 使用示例
if __name__ == "__main__":
    # 测试代码
    rospy.init_node('haption6d_expert_test')
    
    expert = Haption6DExpert(debug=True)
    
    try:
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            action, buttons = expert.get_action()
            rospy.loginfo(f"动作: {action}, 按钮: {buttons}")
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("测试程序被中断")
    finally:
        expert.cleanup()