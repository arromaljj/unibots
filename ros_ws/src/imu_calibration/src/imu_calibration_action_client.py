#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from imu_calibration.action import CalibrateIMU
import sys

class IMUCalibrationClient(Node):
    def __init__(self):
        super().__init__('imu_calibration_client')
        self._action_client = ActionClient(
            self,
            CalibrateIMU,
            'calibrate_imu'
        )

    def send_goal(self, duration, window_size):
        goal_msg = CalibrateIMU.Goal()
        goal_msg.duration = duration
        goal_msg.window_size = window_size

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        if result.success:
            self.get_logger().info(
                f'Calibration successful!\n'
                f'Drift values:\n'
                f'X: {result.drift_x:.6f}\n'
                f'Y: {result.drift_y:.6f}\n'
                f'Z: {result.drift_z:.6f}'
            )
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Time elapsed: {feedback.time_elapsed:.2f} seconds, '
            f'Samples collected: {feedback.samples_collected}'
        )

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: python3 imu_calibration_action_client.py <duration_seconds> <window_size>")
        return

    try:
        duration = float(sys.argv[1])
        window_size = int(sys.argv[2])
    except ValueError:
        print("Error: duration must be a float and window_size must be an integer")
        return

    action_client = IMUCalibrationClient()
    action_client.send_goal(duration, window_size)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main() 