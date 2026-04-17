#!/usr/bin/env python3
"""ROS2：简化 Franka 环境（笛卡尔阻抗平衡位姿 + FrankaState 观测）。"""

from __future__ import annotations

import copy
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R

from exp_env_interact._scipy_compat import rotation_3x3, rotation_from_3x3


def _default_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def _spin_until(node: Node, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False


class SimplifiedFrankaEnv:
    """订阅 Franka 状态，发布 equilibrium_pose；接口与 ROS1 版一致。"""

    def __init__(self, node: Node, debug: bool = True) -> None:
        self._node = node
        self.debug = debug

        self.current_pose: Optional[Dict[str, Any]] = None
        self.current_force = np.zeros(6)
        self.target_pose: Optional[Dict[str, Any]] = None

        qos = _default_qos()
        self.pose_pub = node.create_publisher(
            PoseStamped,
            '/cartesian_impedance_example_controller/equilibrium_pose',
            qos,
        )
        self.state_sub = node.create_subscription(
            FrankaState,
            'franka_state_controller/franka_states',
            self.state_callback,
            qos,
        )

        self._node.get_logger().info('等待接收机器人状态（FrankaState）...')
        ok = _spin_until(node, lambda: self.current_pose is not None, timeout_sec=60.0)
        if not ok:
            self._node.get_logger().error('超时未收到 FrankaState，后续 step 可能不可用。')
        else:
            self._node.get_logger().info('SimplifiedFrankaEnv 已初始化（ROS2）')

    def state_callback(self, msg: FrankaState) -> None:
        try:
            transform_matrix = np.reshape(np.array(msg.o_t_ee, dtype=float), (4, 4))
            position = transform_matrix[3, :3]
            inv_rotation_matrix = transform_matrix[:3, :3]
            orientation = rotation_from_3x3(inv_rotation_matrix).inv()
            self.current_pose = {'position': position, 'orientation': orientation}
            kf = msg.k_f_ext_hat_k
            self.current_force = np.array(
                [kf[0], kf[1], kf[2], kf[3], kf[4], kf[5]],
                dtype=float,
            )
            if self.debug:
                if not hasattr(self, 'state_count'):
                    self.state_count = 0
                self.state_count += 1
                if self.state_count % 100 == 0:
                    self._node.get_logger().info(f'已接收 {self.state_count} 条状态消息')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'处理状态消息时出错: {exc}')

    def step(
        self,
        action: Optional[np.ndarray] = None,
        buttons: Optional[List[bool]] = None,
    ) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        del buttons
        try:
            if self.current_pose is None:
                self._node.get_logger().warn('当前位姿数据不可用，跳过 step')
                return self._get_observation(), 0.0, False, {}
            self.retarget(action)
            self._publish_target_pose()
            observation = self._get_observation()
            reward = 0.0
            done = False
            info: Dict[str, Any] = {}
            if self.debug:
                self._node.get_logger().debug(f'Step 完成 — 观测: {observation.shape}, 奖励: {reward}')
            return observation, reward, done, info
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'Step 方法出错: {exc}')
            return self._get_observation(), 0.0, False, {'error': str(exc)}

    def retarget(self, action: Optional[np.ndarray] = None, buttons: Optional[List[bool]] = None) -> None:
        del buttons
        try:
            if self.current_pose is None:
                self._node.get_logger().warn('当前位姿数据不可用，无法进行 retarget')
                return
            if action is None:
                self.target_pose = copy.deepcopy(self.current_pose)
                return
            action = np.asarray(action, dtype=float).reshape(6,)
            if self.target_pose is None:
                self.target_pose = copy.deepcopy(self.current_pose)
            last_target_pose = copy.deepcopy(self.target_pose)
            new_position = last_target_pose['position'] + action[:3]
            last_target_rotation = last_target_pose['orientation']
            m = rotation_3x3(last_target_rotation)
            action_rotvec_in_target_frame = m.T @ action[3:6]
            action_rotation = R.from_rotvec(action_rotvec_in_target_frame)
            new_target_rotation = last_target_rotation * action_rotation
            self.target_pose = {'position': new_position, 'orientation': new_target_rotation}
            if self.debug:
                self._node.get_logger().debug(f'Retarget 完成: 动作={action}')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'Retarget 函数出错: {exc}')
            if self.current_pose is not None:
                self.target_pose = copy.deepcopy(self.current_pose)

    def reset(self) -> np.ndarray:
        try:
            if self.current_pose is None:
                self._node.get_logger().warn('当前位姿数据不可用，无法重置')
                return self._get_observation()
            self.target_pose = copy.deepcopy(self.current_pose)
            self._publish_pose(self.current_pose)
            self._node.get_logger().info('环境已重置 — 目标位姿设置为当前位置')
            if self.debug:
                self._node.get_logger().debug(f'Reset 完成 — 当前位置: {self.current_pose["position"]}')
            return self._get_observation()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'Reset 方法出错: {exc}')
            return self._get_observation()

    def _publish_target_pose(self) -> None:
        if self.target_pose is not None:
            self._publish_pose(self.target_pose)

    def _publish_pose(self, pose_dict: Dict[str, Any]) -> None:
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self._node.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'panda_link0'
            pose_msg.pose.position.x = float(pose_dict['position'][0])
            pose_msg.pose.position.y = float(pose_dict['position'][1])
            pose_msg.pose.position.z = float(pose_dict['position'][2])
            quat = pose_dict['orientation'].as_quat()
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            self.pose_pub.publish(pose_msg)
            if self.debug:
                self._node.get_logger().debug(f'已发布 equilibrium_pose — 位置: {pose_dict["position"]}')
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'发布位姿时出错: {exc}')

    def _get_observation(self) -> np.ndarray:
        try:
            if self.current_pose is None:
                return np.zeros(13)
            quaternion = self.current_pose['orientation'].as_quat()
            return np.concatenate(
                [self.current_pose['position'], quaternion, self.current_force]
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'获取观测值时出错: {exc}')
            return np.zeros(13)

    def get_current_pose(self) -> Optional[Dict[str, Any]]:
        return copy.deepcopy(self.current_pose) if self.current_pose is not None else None

    def get_current_force(self) -> np.ndarray:
        return self.current_force.copy()

    def get_target_pose(self) -> Optional[Dict[str, Any]]:
        return copy.deepcopy(self.target_pose) if self.target_pose is not None else None

    def cleanup(self) -> None:
        self.state_sub.destroy()
        self._node.get_logger().info('SimplifiedFrankaEnv 资源已清理')


def _standalone_main() -> None:
    rclpy.init()
    node = Node('simplified_franka_env_test')
    env = SimplifiedFrankaEnv(node, debug=True)
    try:
        obs = env.reset()
        node.get_logger().info(f'Reset 后观测: {obs}')
        rate = node.create_rate(1)
        for i in range(5):
            obs, reward, done, info = env.step()
            node.get_logger().info(
                f'Step {i}: len={len(obs)}, reward={reward}, done={done}, info={info}'
            )
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        env.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    _standalone_main()
