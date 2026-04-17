#!/usr/bin/env python3
"""
HV6T to DAS Control Node
订阅 /out_handle_analog (std_msgs/Float32) 话题
将0.2~1.0的模拟量线性映射到0.1~0.0
发布到 /target_distance (std_msgs/Float32) 话题

线性映射公式：
  input: 0.2 ~ 1.0
  output: 0.1 ~ 0.0
  计算公式: output = 0.125 * (1 - input)
  或等效: output = -0.125 * input + 0.125

输入值超出0.2~1.0范围时进行钳位处理
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class AnalogToDistanceMapper(Node):
    """模拟量到距离映射节点"""

    def __init__(self):
        super().__init__('hv6t_2_das_node')
        
        # 参数设置
        self.input_min = 0.2
        self.input_max = 1.0
        self.output_min = 0.0
        self.output_max = 0.1
        
        # 计算线性映射参数
        # output = slope * input + intercept
        # 当 input = input_min 时，output = output_max
        # 当 input = input_max 时，output = output_min
        self.slope = (self.output_min - self.output_max) / (self.input_max - self.input_min)
        self.intercept = self.output_max - self.slope * self.input_min
        
        # 创建订阅者：订阅 /out_handle_analog 话题
        self.subscription = self.create_subscription(
            Float32,
            '/out_handle_analog',
            self.analog_callback,
            10  # 队列大小
        )
        self.subscription  # 防止未使用变量警告
        
        # 创建发布者：发布到 /target_distance 话题
        self.publisher = self.create_publisher(
            Float32,
            '/target_distance',
            10  # 队列大小
        )
        
        self.get_logger().info(f'HV6T to DAS 映射节点已启动')
        self.get_logger().info(f'输入范围: {self.input_min} ~ {self.input_max}')
        self.get_logger().info(f'输出范围: {self.output_max} ~ {self.output_min}')
        self.get_logger().info(f'斜率: {self.slope:.4f}, 截距: {self.intercept:.4f}')

    def clamp_input(self, value):
        """钳位输入值到指定范围"""
        if value < self.input_min:
            self.get_logger().debug(f'输入值 {value:.4f} 小于下限 {self.input_min}, 钳位到 {self.input_min}')
            return self.input_min
        elif value > self.input_max:
            self.get_logger().debug(f'输入值 {value:.4f} 大于上限 {self.input_max}, 钳位到 {self.input_max}')
            return self.input_max
        return value

    def analog_callback(self, msg):
        """处理接收到的模拟量数据"""
        # 获取输入值
        input_value = msg.data
        
        # 钳位处理
        clamped_value = self.clamp_input(input_value)
        
        # 线性映射计算
        output_value = self.slope * clamped_value + self.intercept
        
        # 创建并发布输出消息
        output_msg = Float32()
        output_msg.data = output_value
        self.publisher.publish(output_msg)
        
        # 调试日志（频率降低以避免过多日志）
        self.get_logger().debug(
            f'映射: {input_value:.4f} -> {clamped_value:.4f} -> {output_value:.4f}'
        )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建节点
    node = AnalogToDistanceMapper()
    
    try:
        # 运行节点
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()