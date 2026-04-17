#!/usr/bin/env python3
"""ROS2：上下层统一接口节点（专家 + 环境）。"""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node

from exp_env_interact.haption6d_expert import Haption6DExpert


class ExpEnvInteractNode(Node):
    """组合 Haption6DExpert 与环境（Franka 或 RViz2 调试）。"""

    def __init__(self) -> None:
        super().__init__('exp_env_interact_node')

        self.declare_parameter('debug', True)
        self.declare_parameter('backend', 'franka')
        self.declare_parameter('control_rate_hz', 1000.0)
        self.declare_parameter('rviz2_parent_frame', 'world')
        self.declare_parameter('rviz2_child_frame', 'ee_debug')

        debug = self.get_parameter('debug').get_parameter_value().bool_value
        backend = (
            self.get_parameter('backend').get_parameter_value().string_value or 'franka'
        ).lower().strip()

        rate_pv = self.get_parameter('control_rate_hz').get_parameter_value()
        if rate_pv.type == ParameterType.PARAMETER_DOUBLE:
            rate_hz = float(rate_pv.double_value)
        elif rate_pv.type == ParameterType.PARAMETER_INTEGER:
            rate_hz = float(rate_pv.integer_value)
        else:
            rate_hz = 1000.0
        if rate_hz <= 0.0:
            rate_hz = 1000.0
            self.get_logger().warn('control_rate_hz 无效，已回退为 1000 Hz')

        self.agent = Haption6DExpert(self, debug=debug)
        if backend == 'rviz2':
            from exp_env_interact.simplified_rviz2_env import SimplifiedRviz2Env

            parent = self.get_parameter('rviz2_parent_frame').get_parameter_value().string_value
            child = self.get_parameter('rviz2_child_frame').get_parameter_value().string_value
            parent = parent or 'world'
            child = child or 'ee_debug'
            self.env = SimplifiedRviz2Env(
                self, debug=debug, parent_frame=parent, child_frame=child
            )
            self.get_logger().info(f'后端: RViz2 调试 TF ({parent} -> {child})')
        else:
            from exp_env_interact.simplified_franka_env import SimplifiedFrankaEnv

            self.env = SimplifiedFrankaEnv(self, debug=debug)
            self.get_logger().info('后端: Franka（equilibrium_pose + FrankaState）')

        self.obs = self.env.reset()
        self.step_count = 0
        self.is_first_step = True

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info('EXP_ENV_INTERACT 节点已启动（ROS2）')
        self.get_logger().info('Agent 与 Env 已初始化，环境已 reset')

    def _timer_callback(self) -> None:
        try:
            action, buttons = self.agent.get_action(self.obs)
            self.obs, reward, done, info = self.env.step(action, buttons)
            self.step_count += 1

            if self.step_count % 50 == 0:
                self.get_logger().info(
                    f'Step {self.step_count}: 动作={action}, 按钮={buttons}, '
                    f'奖励={reward:.4f}, 完成={done}, info={info}'
                )

            if self.is_first_step or done:
                self.get_logger().info('  详细观测:')
                self.get_logger().info(f'    - 观测长度: {len(self.obs)}')
                self.get_logger().info(
                    f'    - 位置: [{float(self.obs[0]):.3f}, {float(self.obs[1]):.3f}, {float(self.obs[2]):.3f}]'
                )
                self.get_logger().info(
                    f'    - 力: [{float(self.obs[10]):.3f}, {float(self.obs[11]):.3f}, {float(self.obs[12]):.3f}]'
                )
                self.is_first_step = False

            if done:
                self.get_logger().info('任务完成，重置环境')
                self.obs = self.env.reset()
                self.step_count = 0
                self.is_first_step = True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'定时器回调出错: {exc}')

    def shutdown_cleanup(self) -> None:
        self.agent.cleanup()
        self.env.cleanup()
        self.get_logger().info(f'EXP_ENV_INTERACT 资源已清理 — 共执行 {self.step_count} 步')


def main() -> None:
    rclpy.init()
    node = ExpEnvInteractNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
