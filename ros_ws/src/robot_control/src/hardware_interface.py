#!/usr/bin/env python3

import gpiod
import rclpy
from rclpy.node import Node
from hardware_interface.hardware_info import HardwareInfo
from hardware_interface.system_interface import SystemInterface
from hardware_interface.hardware_interface_return_values import HardwareInterfaceReturnValues

from .motor import Motor

class UnibotsMotorHardwareInterface(SystemInterface):
    """
    Basic hardware interface for TB6612FNG motor driver.
    """
    def __init__(self):
        super().__init__()
        # Commands
        self.left_velocity_command = 0.0
        self.right_velocity_command = 0.0
        
        # Hardware components
        self.left_motor = None
        self.right_motor = None
        self.chip = None
        self.standby_line = None
        
        # Default pin configuration matching keyboard_motor_control.py
        self.left_in1_pin = 8
        self.left_in2_pin = 7
        self.left_pwm_pin = 12
        self.right_in1_pin = 23
        self.right_in2_pin = 24
        self.right_pwm_pin = 13
        self.standby_pin = 25
        self.gpio_chip_name = "gpiochip4"
        
        # Motor direction configuration
        self.left_reversed = True
        self.right_reversed = False
        
        # Max velocity that corresponds to 100% duty cycle
        self.max_velocity = 10.0  # rad/s

    def on_init(self, hardware_info):
        """Initialize the hardware interface."""
        if super().on_init(hardware_info) != HardwareInterfaceReturnValues.SUCCESS:
            return HardwareInterfaceReturnValues.ERROR
            
        # Override defaults with any provided parameters
        if hardware_info.hardware_parameters:
            self.gpio_chip_name = hardware_info.hardware_parameters.get("gpio_chip", self.gpio_chip_name)
            self.max_velocity = float(hardware_info.hardware_parameters.get("max_velocity", self.max_velocity))
            
        return HardwareInterfaceReturnValues.SUCCESS

    def on_activate(self, previous_state):
        """Activate motors."""
        try:
            # Initialize GPIO
            self.chip = gpiod.Chip(self.gpio_chip_name)
            self.standby_line = self.chip.get_line(self.standby_pin)
            self.standby_line.request(consumer="motors", type=gpiod.LINE_REQ_DIR_OUT)
            self.standby_line.set_value(1)  # Enable the driver
            
            # Initialize motors with shared standby line
            self.left_motor = Motor(
                self.left_in1_pin,
                self.left_in2_pin,
                self.left_pwm_pin,
                self.standby_pin,
                self.left_reversed,
                self.gpio_chip_name,
                shared_standby_line=self.standby_line
            )
            
            self.right_motor = Motor(
                self.right_in1_pin,
                self.right_in2_pin,
                self.right_pwm_pin,
                self.standby_pin,
                self.right_reversed,
                self.gpio_chip_name,
                shared_standby_line=self.standby_line
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motors: {str(e)}")
            return HardwareInterfaceReturnValues.ERROR
            
        return HardwareInterfaceReturnValues.SUCCESS

    def on_deactivate(self, previous_state):
        """Stop and cleanup motors."""
        if self.left_motor:
            self.left_motor.brake()
        if self.right_motor:
            self.right_motor.brake()
            
        if self.standby_line:
            self.standby_line.set_value(0)
            self.standby_line.release()
            
        if self.chip:
            self.chip.close()
            
        self.left_motor = None
        self.right_motor = None
        self.standby_line = None
        self.chip = None
        
        return HardwareInterfaceReturnValues.SUCCESS

    def export_command_interfaces(self):
        """Export velocity command interfaces."""
        return [
            self.create_command_interface(
                "left_wheel_joint",
                "velocity",
                self.set_velocity_for_wheel("left")
            ),
            self.create_command_interface(
                "right_wheel_joint",
                "velocity",
                self.set_velocity_for_wheel("right")
            )
        ]

    def read(self, time, period):
        """No feedback available without encoders."""
        return HardwareInterfaceReturnValues.SUCCESS

    def write(self, time, period):
        """Write velocity commands to motors."""
        if self.left_motor and self.right_motor:
            try:
                left_duty_cycle = self.velocity_to_duty_cycle(self.left_velocity_command)
                right_duty_cycle = self.velocity_to_duty_cycle(self.right_velocity_command)
                
                self.left_motor.drive(left_duty_cycle)
                self.right_motor.drive(right_duty_cycle)
            except Exception as e:
                self.get_logger().error(f"Error writing to motors: {str(e)}")
                return HardwareInterfaceReturnValues.ERROR
        
        return HardwareInterfaceReturnValues.SUCCESS

    def set_velocity_for_wheel(self, wheel):
        """Get velocity setter function for the given wheel."""
        def _set_velocity(value):
            if wheel == "left":
                self.left_velocity_command = value
            else:
                self.right_velocity_command = value
        return _set_velocity

    def velocity_to_duty_cycle(self, velocity):
        """Convert velocity to duty cycle."""
        clamped_velocity = max(-self.max_velocity, min(velocity, self.max_velocity))
        return (clamped_velocity / self.max_velocity) * 100.0

    def get_logger(self):
        """Get ROS logger or fallback to print."""
        try:
            if rclpy.ok():
                node = Node('motor_hardware_interface_logger')
                return node.get_logger()
        except:
            pass
            
        class DummyLogger:
            def info(self, msg): print(f"INFO: {msg}")
            def warn(self, msg): print(f"WARN: {msg}")
            def error(self, msg): print(f"ERROR: {msg}")
            def debug(self, msg): print(f"DEBUG: {msg}")
        return DummyLogger()


# This function is needed for pluginlib to identify the interface class
def export_plugin():
    return UnibotsMotorHardwareInterface 