#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from imu_calibration.action import CalibrateIMU
from collections import deque
import numpy as np
import time

class IMUCalibrationServer(Node):
    def __init__(self):
        super().__init__('imu_calibration_server')
        
        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize calibration variables
        self.is_calibrating = False
        self.accel_x = deque()
        self.accel_y = deque()
        self.accel_z = deque()
        self.start_time = None
        self.calibration_duration = 0
        self.samples_collected = 0
        
        # Create the action server
        self._action_server = ActionServer(
            self,
            CalibrateIMU,
            'calibrate_imu',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        # Create IMU subscriber
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Declare parameters
        self.declare_parameter('imu_drift_x', 0.0)
        self.declare_parameter('imu_drift_y', 0.0)
        self.declare_parameter('imu_drift_z', 0.0)
        
        self.get_logger().info('IMU Calibration Action Server is ready')
    
    def imu_callback(self, msg):
        """
        Process incoming IMU messages during calibration
        """
        if not self.is_calibrating:
            return
            
        # Add acceleration data to queues
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        self.samples_collected += 1
    
    def calculate_drift(self):
        """
        Calculate the average drift from collected data
        """
        if len(self.accel_x) == 0:
            return None, None, None
            
        avg_x = float(np.mean(self.accel_x))
        avg_y = float(np.mean(self.accel_y))
        avg_z = float(np.mean(self.accel_z))
        
        return avg_x, avg_y, avg_z
    
    async def execute_callback(self, goal_handle):
        """
        Execute calibration action
        """
        self.get_logger().info(f'Starting calibration for {goal_handle.request.duration} seconds with window size {goal_handle.request.window_size}')
        
        # Initialize calibration
        self.is_calibrating = True
        self.start_time = time.time()
        self.calibration_duration = goal_handle.request.duration
        self.samples_collected = 0
        
        # Clear and resize queues
        self.accel_x = deque(maxlen=goal_handle.request.window_size)
        self.accel_y = deque(maxlen=goal_handle.request.window_size)
        self.accel_z = deque(maxlen=goal_handle.request.window_size)
        
        feedback_msg = CalibrateIMU.Feedback()
        
        # Monitor calibration progress
        while self.is_calibrating:
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            
            if elapsed_time >= self.calibration_duration:
                self.is_calibrating = False
                break
                
            # Publish feedback
            feedback_msg.time_elapsed = elapsed_time
            feedback_msg.samples_collected = self.samples_collected
            goal_handle.publish_feedback(feedback_msg)
            
            # Check if the goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.is_calibrating = False
                return CalibrateIMU.Result()
            
            # Sleep for 0.1 seconds
            time.sleep(0.1)
        
        # Calculate final drift values
        drift_x, drift_y, drift_z = self.calculate_drift()
        
        # Create result message
        result = CalibrateIMU.Result()
        
        if drift_x is None:
            result.success = False
            result.message = 'No IMU data received during calibration'
            goal_handle.abort()
            return result
            
        # Store results as parameters
        self.set_parameters([
            Parameter('imu_drift_x', Parameter.Type.DOUBLE, drift_x),
            Parameter('imu_drift_y', Parameter.Type.DOUBLE, drift_y),
            Parameter('imu_drift_z', Parameter.Type.DOUBLE, drift_z)
        ])
        
        # Set result values
        result.success = True
        result.message = 'Calibration completed successfully'
        result.drift_x = drift_x
        result.drift_y = drift_y
        result.drift_z = drift_z
        
        self.get_logger().info(
            f'Calibration completed. Drift values - '
            f'X: {drift_x:.6f}, Y: {drift_y:.6f}, Z: {drift_z:.6f}'
        )
        
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    
    calibration_server = IMUCalibrationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(calibration_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        calibration_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 