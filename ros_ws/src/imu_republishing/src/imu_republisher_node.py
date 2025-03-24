#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters

class IMURepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')
        
        # Create a parameter client to query the calibration server node
        calibration_node_name = 'imu_calibration_server'
        service_name = f'/{calibration_node_name}/get_parameters'
        self.param_client = self.create_client(GetParameters, service_name)
        
        # Wait for the parameter service to become available
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Parameter service not available on {service_name}. Using default drift values.")
            self.drift_x = 0.0
            self.drift_y = 0.0
            self.drift_z = 0.0
        else:
            request = GetParameters.Request()
            request.names = ['imu_drift_x', 'imu_drift_y', 'imu_drift_z']
            future = self.param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response = future.result()
                # Ensure the parameter types are as expected, otherwise default to 0.0
                self.drift_x = float(response.values[0].double_value) if response.values[0].type == ParameterType.PARAMETER_DOUBLE else 0.0
                self.drift_y = float(response.values[1].double_value) if response.values[1].type == ParameterType.PARAMETER_DOUBLE else 0.0
                self.drift_z = float(response.values[2].double_value) if response.values[2].type == ParameterType.PARAMETER_DOUBLE else 0.0
            else:
                self.get_logger().warn("Failed to get calibration parameters. Using default drift values.")
                self.drift_x = 0.0
                self.drift_y = 0.0
                self.drift_z = 0.0
        
        self.get_logger().info(f'Using drift corrections - X: {self.drift_x}, Y: {self.drift_y}, Z: {self.drift_z}')
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Imu,
            '/imu/drift_corrected',
            10
        )
        
        self.get_logger().info('IMU Republisher node is running')
    
    def imu_callback(self, msg):
        """Process incoming IMU messages and apply drift correction"""
        corrected_msg = Imu()
        corrected_msg.header = msg.header
        
        # Apply drift corrections to linear acceleration
        corrected_msg.linear_acceleration.x = msg.linear_acceleration.x - self.drift_x
        corrected_msg.linear_acceleration.y = msg.linear_acceleration.y - self.drift_y
        corrected_msg.linear_acceleration.z = msg.linear_acceleration.z - self.drift_z
        
        # Copy other fields unchanged
        corrected_msg.angular_velocity = msg.angular_velocity
        corrected_msg.orientation = msg.orientation
        corrected_msg.orientation_covariance = msg.orientation_covariance
        corrected_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # Publish corrected message
        self.publisher.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_republisher = IMURepublisher()
    
    try:
        rclpy.spin(imu_republisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_republisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 