#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from collections import deque
import numpy as np

class IMUDriftCalculator(Node):
    def __init__(self):
        super().__init__('imu_drift_calculator')
        
        # Parameters
        self.window_size = 100  # Size of moving average window
        
        # Initialize deques for x, y, z acceleration values
        self.accel_x = deque(maxlen=self.window_size)
        self.accel_y = deque(maxlen=self.window_size)
        self.accel_z = deque(maxlen=self.window_size)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10)
        
        # Create publisher
        self.publisher = self.create_publisher(
            Vector3,
            '/imu_calib/linear_accel_drift',
            10)
        
        self.get_logger().info('IMU Drift Calculator Node Started')

    def imu_callback(self, msg):
        # Add new acceleration values to deques
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        
        # Calculate moving averages
        avg_x = np.mean(self.accel_x) if len(self.accel_x) > 0 else 0.0
        avg_y = np.mean(self.accel_y) if len(self.accel_y) > 0 else 0.0
        avg_z = np.mean(self.accel_z) if len(self.accel_z) > 0 else 0.0
        
        # Create and publish drift message
        drift_msg = Vector3()
        drift_msg.x = float(avg_x)
        drift_msg.y = float(avg_y)
        drift_msg.z = float(avg_z)
        
        self.publisher.publish(drift_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriftCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 