#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration

class Publisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
    
        # 创建自定义QoS配置以保存所有历史数据
        qos_profile = rclpy.qos.QoSProfile(
            depth=0,  # 设置为0表示无限制深度
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
    
        self.publisher_ = self.create_publisher(String, 'topic999', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: \"%s\"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    pub_node = Publisher()
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

