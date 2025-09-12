#!/usr/bin/env python3
"""
简单的无人机启用器脚本
持续发布启用信号到 /X3/enable 话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class DroneEnabler(Node):
    def __init__(self):
        super().__init__('drone_enabler')
        
        # 创建发布者
        self.publisher = self.create_publisher(Bool, '/X3/enable', 10)
        
        # 创建定时器，每秒发布一次启用信号
        self.timer = self.create_timer(1.0, self.publish_enable)
        
        self.get_logger().info("Drone enabler started - sending enable signals to /X3/enable")
    
    def publish_enable(self):
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info("Sent enable signal")

def main(args=None):
    rclpy.init(args=args)
    
    node = DroneEnabler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()