#!/usr/bin/env python3
"""
中国某机器人公司 G组 端侧遥操作软件开发
李佩泽 LI Peize lipeize@agibot.com
上下互动统一接口 v0.0.1
"""

import rospy
import numpy as np
from haption6d_expert import Haption6DExpert
from simplified_franka_env import SimplifiedFrankaEnv

class ExpEnvInteract:
    def __init__(self):
        """初始化EXP_ENV_INTERACT节点"""
        rospy.init_node('exp_env_interact_node')
        
        # 创建agent和env实例
        self.agent = Haption6DExpert(debug=True)
        self.env = SimplifiedFrankaEnv(debug=True)
        
        # 重置环境
        self.obs = self.env.reset()
        
        # 交互状态跟踪
        self.step_count = 0
        self.is_first_step = True
        
        # 设置时钟回调频率 (10Hz)
        self.rate = rospy.Rate(1000)  # 500Hz
        
        rospy.loginfo("EXP_ENV_INTERACT节点已启动")
        rospy.loginfo("Agent和Env已初始化，环境已重置")

    def timer_callback(self):
        """时钟回调函数 - 实现agent和env的交互"""
        try:
            # 使用当前观测值获取agent的动作
            action, buttons = self.agent.get_action(self.obs)
            
            # 使用动作和按钮状态执行env的step，获取新的观测值
            self.obs, reward, done, info = self.env.step(action, buttons)
            
            # 更新统计信息
            self.step_count += 1
            
            # 调试输出 - 单行更新显示最新一步
            if self.step_count % 5000 == 0:  # 每50步更新一次显示，避免刷屏
                rospy.loginfo(f"Step {self.step_count}: 动作={action}, 按钮={buttons}, 奖励={reward:.4f}, 完成={done}")
            
            # 只在第一步或完成时显示详细观测信息
            if self.is_first_step or done:
                rospy.loginfo(f"  详细观测:")
                rospy.loginfo(f"    - 观测长度: {len(self.obs)}")
                rospy.loginfo(f"    - 位置: [{self.obs[0]:.3f}, {self.obs[1]:.3f}, {self.obs[2]:.3f}]")
                rospy.loginfo(f"    - 力: [{self.obs[10]:.3f}, {self.obs[11]:.3f}, {self.obs[12]:.3f}]")
                self.is_first_step = False
            
            # 检查是否完成，如果完成则重置环境
            if done:
                rospy.loginfo("任务完成，重置环境")
                self.obs = self.env.reset()
                self.step_count = 0
                self.is_first_step = True
            
        except Exception as e:
            rospy.logerr(f"时钟回调函数出错: {e}")

    def run(self):
        """主运行循环"""
        rospy.loginfo("EXP_ENV_INTERACT节点开始运行...")
        rospy.loginfo("开始Agent和Env交互循环")
        
        while not rospy.is_shutdown():
            # 调用时钟回调函数
            self.timer_callback()
            
            # 保持循环频率
            self.rate.sleep()

    def cleanup(self):
        """清理资源"""
        self.agent.cleanup()
        self.env.cleanup()
        rospy.loginfo(f"EXP_ENV_INTERACT节点资源已清理 - 总共执行 {self.step_count} 步")

if __name__ == '__main__':
    node = None
    try:
        node = ExpEnvInteract()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行出错: {e}")
    finally:
        if node is not None:
            node.cleanup()