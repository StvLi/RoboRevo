#!/usr/bin/env python3
"""ROS2：Haption / Virtuose 6D 专家输入（差分映射）。"""

from __future__ import annotations

import copy
import threading
from typing import Any, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool

from exp_env_interact._scipy_compat import rotation_3x3, rotation_from_3x3
from virtuose_ros2.msg import OutVirtuosePose


def _default_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


class Haption6DExpert:
    """
    Haption 6D 专家控制类：订阅 Virtuose 话题，输出 6D 增量与按钮状态。
    需挂载在已初始化的 rclpy.Node 上（由上层节点传入）。
    """

    def __init__(self, node: Node, debug: bool = True) -> None:
        self._node = node
        self.debug = False

        self.fsm = 'idle'

        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.current_bools = np.array([False, False, False, False])
        self.last_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.last_bools = np.array([False, False, False, False])
        self.tmp_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.tmp_bools = np.array([False, False, False, False])

        self.action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        qos = _default_qos()
        self.pose_sub = node.create_subscription(
            OutVirtuosePose,
            '/out_virtuose_pose',
            self.pose_callback,
            qos,
        )
        self.bool_sub_a = node.create_subscription(
            Bool,
            '/out_fsm_trigger/button_a',
            self.bool_a_callback,
            qos,
        )
        self.bool_sub_b = node.create_subscription(
            Bool,
            '/out_fsm_trigger/button_b',
            self.bool_b_callback,
            qos,
        )
        self.trigger_sub_a = node.create_subscription(
            Bool,
            '/out_fsm_trigger/trigger_a',
            self.trigger_a_callback,
            qos,
        )
        self.trigger_sub_b = node.create_subscription(
            Bool,
            '/out_fsm_trigger/trigger_b',
            self.trigger_b_callback,
            qos,
        )

        self.pose_count = 0
        self.status_count = 0

        self._node.get_logger().info('Haption6DExpert 已初始化（ROS2）')

    def pose_callback(self, msg: OutVirtuosePose) -> None:
        try:
            p = msg.virtuose_pose.translation
            o = msg.virtuose_pose.rotation
            tmp = np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w])
            self.tmp_pose = tmp
            self.pose_count += 1
            if self.debug and self.pose_count % 5000 == 0:
                self._node.get_logger().info(f'已接收 {self.pose_count} 条 pose 消息')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'处理 pose 消息时出错: {exc}')

    def _update_tmp_bool(self, index: int, value: bool) -> None:
        self.tmp_bools[index] = bool(value)
        self.status_count += 1
        if self.debug and self.status_count % 5000 == 0:
            self._node.get_logger().info(f'已接收 {self.status_count} 条 bool 消息')

    def bool_a_callback(self, msg: Bool) -> None:
        try:
            self._update_tmp_bool(0, msg.data)
            # if msg.data:
            #     self._node.get_logger().info(f'button_a 消息: {msg.data}')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'处理 button_a 消息时出错: {exc}')

    def bool_b_callback(self, msg: Bool) -> None:
        try:
            self._update_tmp_bool(1, msg.data)
            # if msg.data:
            #     self._node.get_logger().info(f'button_b 消息: {msg.data}')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'处理 button_b 消息时出错: {exc}')

    def trigger_a_callback(self, msg: Bool) -> None:
        try:
            self._update_tmp_bool(2, msg.data)
            # if msg.data:
            #     self._node.get_logger().info(f'trigger_a 消息: {msg.data}')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'处理 trigger_a 消息时出错: {exc}')

    def trigger_b_callback(self, msg: Bool) -> None:
        try:
            self._update_tmp_bool(3, msg.data)
            # if msg.data:
            #     self._node.get_logger().info(f'trigger_b 消息: {msg.data}')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'处理 trigger_b 消息时出错: {exc}')

    def get_action(self, obs: Optional[Any] = None) -> Tuple[np.ndarray, List[bool]]:
        del obs
        self.update_buttons()
        self.update_fsm()
        self.update_pose()
        self.update_action()
        return self.action, [bool(self.current_bools[0])]

    def update_buttons(self) -> None:
        self.current_bools = self.tmp_bools
        if self.debug:
            self._node.get_logger().debug(f'FSM 状态: {self.fsm}')
            self._node.get_logger().debug(f'当前按钮: {self.current_bools}')

    def update_fsm(self) -> None:
        if self.fsm == 'idle':
            if self.current_bools[3]:
                # 按下拨杆A，进入ready_to_project状态
                self.fsm = 'ready_to_project'
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
            if self.current_bools[1]:
                # 按下按钮B，进入project_diff状态
                self.fsm = 'button_project_diff'
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
        elif self.fsm == 'ready_to_project':
            if not self.current_bools[3]:
                # 按下拨杆A，进入project_diff状态
                self.fsm = 'trigger_project_diff'
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
        elif self.fsm == 'trigger_project_diff':
            if self.current_bools[3]:
                # 按下拨杆A，进入ready_to_idle状态
                self.fsm = 'ready_to_idle'
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
        elif self.fsm == 'button_project_diff':
            if not self.current_bools[1]:
                # 释放按钮B，进入idle状态
                self.fsm = 'idle'
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
        elif self.fsm == 'ready_to_idle':
            if not self.current_bools[3]:
                # 按下拨杆A，进入idle状态
                self.fsm = 'idle'
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
        else:
            self.fsm = 'idle'

    def update_pose(self) -> None:
        if self.fsm == 'idle':
            self.current_pose = copy.deepcopy(self.tmp_pose)
            self.last_pose = copy.deepcopy(self.current_pose)
            if self.debug:
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
                self._node.get_logger().info(f'当前位姿: {self.current_pose}')
                self._node.get_logger().info(f'上次位姿: {self.last_pose}')
        elif self.fsm == 'button_project_diff' or self.fsm == 'trigger_project_diff':
            self.last_pose = copy.deepcopy(self.current_pose)
            self.current_pose = copy.deepcopy(self.tmp_pose)
            if self.debug:
                self._node.get_logger().info(f'FSM 状态: {self.fsm}')
                self._node.get_logger().info(f'上次位姿: {self.last_pose}')
                self._node.get_logger().info(f'当前位姿: {self.current_pose}')

    @staticmethod
    def normalize_quaternion(quat: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(quat)
        if norm > 1e-8:
            return quat / norm
        return np.array([0.0, 0.0, 0.0, 1.0])

    def update_action(self) -> None:
        if self.fsm == 'button_project_diff' or self.fsm == 'trigger_project_diff':
            print('--- project_diff: difference between current and last pose ---')
            current_translation = self.current_pose[0:3]
            last_translation = self.last_pose[0:3]
            current_quat = self.normalize_quaternion(self.current_pose[3:7])
            last_quat = self.normalize_quaternion(self.last_pose[3:7])
            current_orientation = R.from_quat(current_quat)
            last_orientation = R.from_quat(last_quat)
            delta_translation_space = current_translation - last_translation
            lm = rotation_3x3(last_orientation)
            cm = rotation_3x3(current_orientation)
            delta_orientation_body = rotation_from_3x3(lm.T @ cm).as_rotvec()
            delta_orientation_space = lm @ delta_orientation_body
            self.action = np.hstack([delta_translation_space, delta_orientation_space])
            # if self.debug:
            if True:
                self._node.get_logger().info(f'动作向量: {self.action}')
        elif self.fsm == 'idle':
            self.action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def get_current_pose(self) -> np.ndarray:
        return self.current_pose.copy()

    def get_current_buttons(self) -> np.ndarray:
        return self.current_bools.copy()

    def cleanup(self) -> None:
        self.pose_sub.destroy()
        self.bool_sub_a.destroy()
        self.bool_sub_b.destroy()
        self.trigger_sub_a.destroy()
        self.trigger_sub_b.destroy()
        self._node.get_logger().info(
            f'Haption6DExpert 资源已清理 — Pose({self.pose_count}), Status({self.status_count})'
        )


def _standalone_main() -> None:
    rclpy.init()
    node = Node('haption6d_expert_test')
    expert = Haption6DExpert(node, debug=True)
    rate = node.create_rate(10)
    try:
        while rclpy.ok():
            action, buttons = expert.get_action()
            node.get_logger().info(f'动作: {action}, 按钮: {buttons}')
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        expert.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    _standalone_main()
