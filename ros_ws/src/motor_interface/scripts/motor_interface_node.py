#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorInterfaceNode(Node):
    def __init__(self):
        super().__init__('motor_interface_node')
        self.get_logger().info('Motor Interface Node Started')
        
        # Create publishers for wheel velocities
        self.left_wheel_pub = self.create_publisher(
            Float64,
            '/left_wheel_joint/velocity',
            10)
        self.right_wheel_pub = self.create_publisher(
            Float64,
            '/right_wheel_joint/velocity',
            10)
            
        # Create a timer to publish velocities periodically
        self.timer = self.create_timer(0.1, self.publish_velocities)
        
    def publish_velocities(self):
        # Example: Set both wheels to 1.0 rad/s
        left_msg = Float64()
        left_msg.data = 1.0
        right_msg = Float64()
        right_msg.data = 1.0
        
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 