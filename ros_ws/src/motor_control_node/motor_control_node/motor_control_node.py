#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from .motor_controller import MotorController
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from .motor import Motor


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Declare parameters matching URDF configuration
        self.declare_parameter('wheel_separation', 0.24)  # meters (from URDF)
        self.declare_parameter('wheel_radius', 0.05)      # meters (from URDF)
        self.declare_parameter('standby_pin', 25)
        
        # Left motor pins
        self.declare_parameter('left_motor.in1_pin', 8)
        self.declare_parameter('left_motor.in2_pin', 7)
        self.declare_parameter('left_motor.pwm_pin', 12)
        self.declare_parameter('left_motor.reversed', True)
        
        # Right motor pins
        self.declare_parameter('right_motor.in1_pin', 23)
        self.declare_parameter('right_motor.in2_pin', 24)
        self.declare_parameter('right_motor.pwm_pin', 13)
        self.declare_parameter('right_motor.reversed', False)
        
        # Get parameter values
        wheel_separation = self.get_parameter('wheel_separation').value
        wheel_radius = self.get_parameter('wheel_radius').value
        standby_pin = self.get_parameter('standby_pin').value
        
        # Get left motor parameters
        left_motor_pins = {
            'in1': self.get_parameter('left_motor.in1_pin').value,
            'in2': self.get_parameter('left_motor.in2_pin').value,
            'pwm': self.get_parameter('left_motor.pwm_pin').value
        }
        self.left_motor_pins = left_motor_pins
        left_reversed = self.get_parameter('left_motor.reversed').value
        
        # Get right motor parameters
        right_motor_pins = {
            'in1': self.get_parameter('right_motor.in1_pin').value,
            'in2': self.get_parameter('right_motor.in2_pin').value,
            'pwm': self.get_parameter('right_motor.pwm_pin').value
        }
        self.right_motor_pins = right_motor_pins
        right_reversed = self.get_parameter('right_motor.reversed').value
        
        # Print configuration
        self.get_logger().info("\nMotor Control Node Configuration:")
        self.get_logger().info(f"Wheel separation: {wheel_separation}m (from URDF)")
        self.get_logger().info(f"Wheel radius: {wheel_radius}m (from URDF)")
        self.get_logger().info(f"Standby pin: {standby_pin}")
        self.get_logger().info("\nLeft motor configuration:")
        self.get_logger().info(f"IN1: {left_motor_pins['in1']}")
        self.get_logger().info(f"IN2: {left_motor_pins['in2']}")
        self.get_logger().info(f"PWM: {left_motor_pins['pwm']}")
        self.get_logger().info(f"Reversed: {left_reversed}")
        self.get_logger().info("\nRight motor configuration:")
        self.get_logger().info(f"IN1: {right_motor_pins['in1']}")
        self.get_logger().info(f"IN2: {right_motor_pins['in2']}")
        self.get_logger().info(f"PWM: {right_motor_pins['pwm']}")
        self.get_logger().info(f"Reversed: {right_reversed}")
        
        # Initialize motors
        print("\nInitializing left motor...")
        print(f"Left motor pins - IN1: {self.left_motor_pins['in1']}, IN2: {self.left_motor_pins['in2']}, PWM: {self.left_motor_pins['pwm']}")  
        self.left_motor = Motor(
            in1=left_motor_pins['in1'],
            in2=left_motor_pins['in2'],
            pwm=left_motor_pins['pwm'],
            standbyPin=standby_pin,
            reverse=left_reversed
        )
        
        # Create right motor instance
        print("\nInitializing right motor...")
        print(f"Right motor pins - IN1: {self.right_motor_pins['in1']}, IN2: {self.right_motor_pins['in2']}, PWM: {self.right_motor_pins['pwm']}")
        self.right_motor = Motor(
            in1=right_motor_pins['in1'],
            in2=right_motor_pins['in2'],
            pwm=right_motor_pins['pwm'],
            standbyPin=standby_pin,
            reverse=right_reversed,
            shared_standby_line=self.left_motor.standby_line
        )
        self.handle_diff_drive()




        



        



    def handle_diff_drive(self):
        # Setup QoS profile for command topics
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to the diff_drive_controller's command topics
        self.left_command_sub = self.create_subscription(
            Float64,
            '/diff_drive_controller/left_wheel_velocity',
            self.left_wheel_callback,
            qos
        )
        
        self.right_command_sub = self.create_subscription(
            Float64,
            '/diff_drive_controller/right_wheel_velocity',
            self.right_wheel_callback,
            qos
        )
        

    def left_wheel_callback(self, msg):
        """
        Process left wheel velocity commands
        """
        velocity = msg.data  # rad/s
        self.left_motor.drive(velocity)

    def right_wheel_callback(self, msg):
        """
        Process right wheel velocity commands
        """
        velocity = msg.data  # rad/s
        self.right_motor.drive(velocity)


    def cleanup(self):
        self.left_motor.brake()
        self.right_motor.brake()


    def __del__(self):
        """Cleanup when the node is destroyed."""
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down motor control node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 