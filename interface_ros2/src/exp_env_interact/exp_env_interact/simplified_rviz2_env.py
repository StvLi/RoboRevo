#!/usr/bin/env python3
"""ROS2：无 Franka 的 RViz2 调试环境 — 发布 TF，retarget 逻辑与 SimplifiedFrankaEnv 一致。"""

from __future__ import annotations

import copy
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R

from exp_env_interact._scipy_compat import rotation_3x3
from tf2_ros import TransformBroadcaster


def _default_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


class SimplifiedRviz2Env:
    """
    不连接真实机械臂：内部维护 current/target 位姿。
    reset 后位于父坐标系原点、无转动；step 使用与 simplified_franka_env 相同的差分积分，
    将目标位姿通过 tf2 广播（理想跟踪：每步后 current_pose = target_pose）。
    """

    def __init__(
        self,
        node: Node,
        debug: bool = True,
        parent_frame: str = 'world',
        child_frame: str = 'ee_debug',
    ) -> None:
        self._node = node
        self.debug = debug
        self.parent_frame = parent_frame
        self.child_frame = child_frame

        self.current_pose: Optional[Dict[str, Any]] = None
        self.current_force = np.zeros(6)
        self.target_pose: Optional[Dict[str, Any]] = None

        self._tf_broadcaster = TransformBroadcaster(node)

        self._identity_pose = {
            'position': np.zeros(3, dtype=float),
            'orientation': R.from_quat([0.0, 0.0, 0.0, 1.0]),
        }

        self._node.get_logger().info(
            f'SimplifiedRviz2Env 已初始化 — TF {self.parent_frame} -> {self.child_frame}'
        )

    def _publish_tf(self, pose_dict: Dict[str, Any]) -> None:
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        pos = pose_dict['position']
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        q = pose_dict['orientation'].as_quat()
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self._tf_broadcaster.sendTransform(t)

    def reset(self) -> np.ndarray:
        try:
            self.current_pose = copy.deepcopy(self._identity_pose)
            self.target_pose = copy.deepcopy(self._identity_pose)
            self.current_force = np.zeros(6)
            self._publish_tf(self.target_pose)
            self._node.get_logger().info('RViz2 环境已重置 — 原点、无转动')
            if self.debug:
                self._node.get_logger().debug('Reset 完成 — 位于 world 原点')
            return self._get_observation()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f'Reset 方法出错: {exc}')
            return self._get_observation()

    def step(
        self,
        action: Optional[np.ndarray] = None,
        buttons: Optional[List[bool]] = None,
    ) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        del buttons
        try:
            if self.current_pose is None:
                self._node.get_logger().warn('当前位姿不可用，跳过 step')
                return self._get_observation(), 0.0, False, {}
            self.retarget(action)
            if self.target_pose is None:
                return self._get_observation(), 0.0, False, {}
            # 理想跟踪：观测与 TF 一致
            self.current_pose = copy.deepcopy(self.target_pose)
            self._publish_tf(self.target_pose)
            observation = self._get_observation()
            reward = 0.0
            done = False
            info: Dict[str, Any] = {}
            if self.debug:
                self._node.get_logger().debug(f'Step 完成 — 观测: {observation.shape}')
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
        self._node.get_logger().info('SimplifiedRviz2Env 资源已清理')


def _standalone_main() -> None:
    rclpy.init()
    node = Node('simplified_rviz2_env_test')
    env = SimplifiedRviz2Env(node, debug=True)
    try:
        obs = env.reset()
        node.get_logger().info(f'Reset 后观测: {obs}')
        rate = node.create_rate(2)
        for _ in range(20):
            a = np.array([0.001, 0.0, 0.0, 0.0, 0.0, 0.0])
            obs, _, _, _ = env.step(a)
            rate.sleep()
        node.get_logger().info(f'末观测位置: {obs[:3]}')
    except KeyboardInterrupt:
        pass
    finally:
        env.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    _standalone_main()
