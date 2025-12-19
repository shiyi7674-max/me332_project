#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration
from time import sleep

class Subscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
    
        # 创建匹配的QoS配置
        qos_profile = rclpy.qos.QoSProfile(
            depth=0,  # 无限制深度
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
    
        self.subscription = self.create_subscription(
            String,
            'topic999',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.received_messages = []  # 新增：用于存储所有接收到的消息

    def listener_callback(self, msg):
        self.get_logger().info('I heard: \"%s\"' % msg.data)
        self.received_messages.append(msg.data)  # 保存所有消息
        self.get_logger().info('Total messages received: %d' % len(self.received_messages))


def main(args=None):
    rclpy.init(args=args)
    sub_node = Subscriber()
    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

