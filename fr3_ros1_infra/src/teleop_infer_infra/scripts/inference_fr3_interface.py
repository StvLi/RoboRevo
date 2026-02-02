#!/usr/bin/env python3
"""
BAAI 具身大模型组
基于InferenceClient的FR3机器人控制接口
"""

import rospy
import numpy as np
import sys
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import copy
# 获取工作空间根目录路径
workspace_root = os.path.join(os.path.dirname(__file__), '../../..')
import message_filters
# 添加包路径
# 1. 添加teleop_infer_infra包路径
sys.path.append(os.path.join(workspace_root, 'src/teleop_infer_infra/src'))
# 2. 添加simplified_franka_env路径（在tmp/reference_scripts/dated_teleop目录中）
# sys.path.append(os.path.join(workspace_root, 'tmp/reference_scripts/dated_teleop'))

# 导入InferenceClient和SimplifiedFrankaEnv
from teleop_infer_infra.inference import InferenceClient
from teleop_infer_infra.simplified_franka_env import SimplifiedFrankaEnv

class InferenceFR3Interface:
    def __init__(self, use_mock: bool = False, debug: bool = True):
        """
        初始化InferenceFR3Interface节点
        
        Args:
            use_mock: 使用mock模式（生成随机动作）进行测试
            debug: 启用调试日志
        """
        rospy.init_node('inference_fr3_interface_node')
        
        # 创建agent和env实例
        # 使用InferenceClient作为agent，替代Haption6DExpert
        self.agent = InferenceClient(debug=debug, use_mock=use_mock)
        self.env = SimplifiedFrankaEnv(ref_frame='ee', debug=debug)
        
        # 重置环境
        self.obs = self.env.reset()
        
        # 交互状态跟踪
        self.step_count = 0
        self.is_first_step = True
        
        # 设置时钟回调频率 (10Hz)
        self.rate = rospy.Rate(30)  # 10Hz
        self.bridge = CvBridge()
        self.latest_images = None
        self.img_lock = threading.Lock()
        mid_topic = '/camera/color/image_raw'
        left_topic = '/camera_1/color/image_raw'
        right_topic = '/camera_2/color/image_raw'
        rospy.loginfo(f"订阅相机: 中={mid_topic}, 左={left_topic}, 右={right_topic}")
        sub_mid = message_filters.Subscriber(mid_topic, Image)
        sub_left = message_filters.Subscriber(left_topic, Image)
        sub_right = message_filters.Subscriber(right_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_mid, sub_left, sub_right], 10, 0.1
        )
        self.ts.registerCallback(self.image_callback)
        # 设置图像源
        self._setup_image_source()
        
        # 设置初始机器人状态
        self._setup_robot_state()
        
        rospy.loginfo("InferenceFR3Interface节点已启动")
        rospy.loginfo(f"Agent: InferenceClient (mock={use_mock}), Env: SimplifiedFrankaEnv")
        rospy.loginfo("环境已重置，准备开始交互")

    def image_callback(self, msg_mid, msg_left, msg_right):
        try:
            # 1. 转换图像
            cv_mid = self.bridge.imgmsg_to_cv2(msg_mid, "rgb8")
            cv_left = self.bridge.imgmsg_to_cv2(msg_left, "rgb8")
            cv_right = self.bridge.imgmsg_to_cv2(msg_right, "rgb8")
            
            # 2. (可选) 可以在这里做 resize，例如缩放到 640x480
            # cv_mid = cv2.resize(cv_mid, (640, 480)) ...
            
            with self.img_lock:
                # 3. 按顺序存入列表: [中, 左, 右] 
                self.latest_images = [cv_mid, cv_left, cv_right]
                
        except Exception as e:
            rospy.logerr_throttle(1, f"图像处理出错: {e}")
            self.latest_images = None

    def _setup_image_source(self):
        def real_image_source():
            with self.img_lock:
                # 如果还没有收到图像，返回 3 张黑图占位
                if not self.latest_images:
                    black_img = np.zeros((480, 640, 3), dtype=np.uint8)
                    rospy.loginfo("self.latest_images未更新 等待相机图像...")
                    return [black_img, black_img, black_img]
                
                # 返回深拷贝的图像列表
                return copy.deepcopy(self.latest_images)
        
        # 注入给 agent
        self.agent.set_image_sources(real_image_source)
        rospy.loginfo("多相机图像源已设置 (中-左-右)")

    def _setup_robot_state(self):
        """设置初始机器人状态"""
        # 初始机器人状态：[x, y, z, qx, qy, qz, qw, gripper_width]
        initial_robot_state = np.array([0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.08])
        self.agent.set_robot_state(initial_robot_state)
        rospy.logdebug(f"初始机器人状态已设置: {initial_robot_state}")

    def timer_callback(self):
        """时钟回调函数 - 实现agent和env的交互"""
        try:
            # 使用当前观测值获取agent的动作
            # InferenceClient的get_action()返回(action, buttons)，其中buttons[1]在execution状态时为True
            # TODO: 
            action, buttons = self.agent.get_action(self.obs)
            
            # 记录FSM状态和按钮状态
            if self.step_count % 100 == 0:
                stats = self.agent.get_statistics()
                rospy.logdebug(f"Step {self.step_count}: FSM={self.agent.fsm}, "
                              f"Buttons={buttons}, Buffer={stats['buffer_index']}/{stats['buffer_size']}")
            
            # 使用动作和按钮状态执行env的step，获取新的观测值
            self.obs, reward, done, info = self.env.step(action, buttons)

            # <<<临时加入的shit>>>
            if buttons[2]:
                self.env.reset()
            
            # 更新统计信息
            self.step_count += 1
            
            # 调试输出 - 单行更新显示最新一步
            if self.step_count % 10 == 0:  # 每5000步更新一次显示，避免刷屏
                rospy.loginfo(f"Step {self.step_count}: "
                             f"动作范数={np.linalg.norm(action):.4f}, "
                             f"按钮={buttons}, 奖励={reward:.4f}, 完成={done}")
            
            # 只在第一步或完成时显示详细观测信息
            if self.is_first_step or done:
                rospy.loginfo(f"  详细观测:")
                # 从字典中提取位置和力信息
                ee_pos = self.obs['ee']['position']
                ee_force = self.obs['ee']['wrench'][:3]  # 只取力的部分，不包括力矩
                rospy.loginfo(f"    - 位置: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
                rospy.loginfo(f"    - 力: [{ee_force[0]:.3f}, {ee_force[1]:.3f}, {ee_force[2]:.3f}]")
                self.is_first_step = False
            
            # 检查是否完成，如果完成则重置环境
            if done:
                rospy.loginfo("任务完成，重置环境")
                self.obs = self.env.reset()
                self.step_count = 0
                self.is_first_step = True
                # 重置InferenceClient
                self.agent.reset()
            
        except Exception as e:
            rospy.logerr(f"时钟回调函数出错: {e}")

    def run(self):
        """主运行循环"""
        rospy.loginfo("InferenceFR3Interface节点开始运行...")
        rospy.loginfo("开始InferenceClient和SimplifiedFrankaEnv交互循环")
        rospy.loginfo("按钮说明: [button0, button1, button2] - button1是SimplifiedFrankaEnv使能标记")
        rospy.loginfo("FSM状态: idle(等待), inference(发送请求), execution(执行动作)")
        
        while not rospy.is_shutdown():
            # 调用时钟回调函数
            self.timer_callback()
            # 保持循环频率
            self.rate.sleep()

    def cleanup(self):
        """清理资源"""
        self.agent.cleanup()
        self.env.cleanup()
        
        # 打印最终统计信息
        stats = self.agent.get_statistics()
        rospy.loginfo(f"InferenceFR3Interface节点资源已清理")
        rospy.loginfo(f"总共执行 {self.step_count} 步")
        rospy.loginfo(f"推理次数: {stats['inference_count']}")
        rospy.loginfo(f"动作生成次数: {stats['action_count']}")
        rospy.loginfo(f"错误次数: {stats['error_count']}")
        rospy.loginfo(f"最终FSM状态: {stats['fsm_state']}")

    def print_status(self):
        """打印当前状态"""
        stats = self.agent.get_statistics()
        rospy.loginfo("=" * 60)
        rospy.loginfo("InferenceFR3Interface 状态报告")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"总步数: {self.step_count}")
        rospy.loginfo(f"当前FSM状态: {self.agent.fsm}")
        rospy.loginfo(f"推理次数: {stats['inference_count']}")
        rospy.loginfo(f"动作生成次数: {stats['action_count']}")
        rospy.loginfo(f"错误次数: {stats['error_count']}")
        rospy.loginfo(f"缓冲区: {stats['buffer_index']}/{stats['buffer_size']}")
        rospy.loginfo("=" * 60)


if __name__ == '__main__':
    node = None
    try:
        # 创建节点实例
        # use_mock=True: 使用mock模式（无需真实推理服务器）
        # use_mock=False: 连接真实推理服务器
        node = InferenceFR3Interface(use_mock=False, debug=True)
        
        # 运行节点
        node.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断，节点停止")
    except Exception as e:
        rospy.logerr(f"节点运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.cleanup()
        rospy.loginfo("InferenceFR3Interface节点已停止")