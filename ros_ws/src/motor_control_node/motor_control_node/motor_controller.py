#!/usr/bin/env python3
import sys
import os
import time

# Add the motor_interface source directory to the Python path
motor_interface_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'motor_interface/src')
if motor_interface_path not in sys.path:
    sys.path.insert(0, motor_interface_path)

try:
    from .motor import Motor
except ImportError as e:
    print(f"Error importing motor module. Make sure motor.py exists in: {motor_interface_path}")
    raise e

class MotorController:
    def __init__(self, wheel_separation, wheel_radius, 
                 left_motor_pins, right_motor_pins, standby_pin,
                 left_reversed=False, right_reversed=False,
                 min_pwm=50, max_pwm=254):
        """
        Initialize the motor controller with the given parameters.
        
        Args:
            wheel_separation (float): Distance between the wheels in meters
            wheel_radius (float): Radius of the wheels in meters
            left_motor_pins (dict): Dictionary containing left motor pins (in1, in2, pwm)
            right_motor_pins (dict): Dictionary containing right motor pins (in1, in2, pwm)
            standby_pin (int): GPIO pin for the standby control
            left_reversed (bool): Whether the left motor direction is reversed
            right_reversed (bool): Whether the right motor direction is reversed
            min_pwm (int): Minimum PWM value (0-100)
            max_pwm (int): Maximum PWM value (0-100)
        """
        print("\nInitializing Motor Controller...")
        print(f"Wheel separation: {wheel_separation}m")
        print(f"Wheel radius: {wheel_radius}m")
        print(f"Left motor reversed: {left_reversed}")
        print(f"Right motor reversed: {right_reversed}")
        print(f"PWM range: {min_pwm}% to {max_pwm}%")
        
        self.wheel_separation = wheel_separation
        self.wheel_radius = wheel_radius
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        
        # Create left motor instance
        print("\nInitializing left motor...")
        print(f"Left motor pins - IN1: {left_motor_pins['in1']}, IN2: {left_motor_pins['in2']}, PWM: {left_motor_pins['pwm']}")
        self.left_motor = Motor(
            in1=left_motor_pins['in1'],
            in2=left_motor_pins['in2'],
            pwm=left_motor_pins['pwm'],
            standbyPin=standby_pin,
            reverse=left_reversed
        )
        
        # Create right motor instance
        print("\nInitializing right motor...")
        print(f"Right motor pins - IN1: {right_motor_pins['in1']}, IN2: {right_motor_pins['in2']}, PWM: {right_motor_pins['pwm']}")
        self.right_motor = Motor(
            in1=right_motor_pins['in1'],
            in2=right_motor_pins['in2'],
            pwm=right_motor_pins['pwm'],
            standbyPin=standby_pin,
            reverse=right_reversed,
            shared_standby_line=self.left_motor.standby_line
        )
        
        # Set motors to standby mode initially
        print("\nSetting motors to standby mode...")
        self.standby(True)
        print("Motor Controller initialization complete!\n")
    

    
    def standby(self, enable):
        """Enable or disable the motor driver."""
        print(f"Setting standby mode to: {'enabled' if enable else 'disabled'}")
        self.left_motor.standby(enable)
    
    def stop(self):
        """Stop both motors."""
        print("Stopping both motors...")
        self.left_motor.brake()
        self.right_motor.brake()
        print("Motors stopped")
    

    

    
    def __del__(self):
        """Cleanup when the controller is destroyed."""
        print("\nCleaning up Motor Controller...")
        self.stop()
        self.standby(False)
        print("Cleanup complete") 