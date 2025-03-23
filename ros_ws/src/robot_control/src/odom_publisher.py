#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.imu_sub = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Linear acceleration from IMU (assuming in m/s^2)
        linear_acceleration_x = msg.linear_acceleration.x
        linear_acceleration_y = msg.linear_acceleration.y
        linear_acceleration_z = msg.linear_acceleration.z # Not used for 2D odom

        # Angular velocity from IMU (assuming in rad/s)
        angular_velocity_z = msg.angular_velocity.z

        # Simple integration (for now, ignoring noise and drift)
        self.theta += angular_velocity_z * dt
        
        # Integrate linear acceleration to get velocity (very basic, needs proper filtering and calibration)
        velocity_x = linear_acceleration_x * dt
        velocity_y = linear_acceleration_y * dt

        # Integrate velocity to get position
        self.x += velocity_x * dt * math.cos(self.theta) - velocity_y * dt * math.sin(self.theta) 
        self.y += velocity_x * dt * math.sin(self.theta) + velocity_y * dt * math.cos(self.theta)


        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'  # Assuming 'base_link' is your robot base frame

        # Set pose (position and orientation)
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=math.sin(self.theta/2), w=math.cos(self.theta/2)) # Assuming yaw only rotation
        )

        # Set twist (velocity) -  using linear acceleration as velocity for now, needs proper integration and filtering
        odom_msg.twist.twist = Twist(
            linear=Vector3(x=velocity_x, y=velocity_y, z=0.0), # Using integrated linear acceleration as velocity
            angular=Vector3(x=0.0, y=0.0, z=angular_velocity_z)
        )

        # Publish odometry message
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()