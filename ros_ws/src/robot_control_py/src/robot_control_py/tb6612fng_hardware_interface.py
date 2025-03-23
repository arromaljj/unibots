#!/usr/bin/env python3

import time
import threading
import gpiod  # Add explicit gpiod import
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from hardware_interface.hardware_info import HardwareInfo
from hardware_interface.system_interface import SystemInterface
from hardware_interface.handle import HandleType
from hardware_interface.hardware_interface_return_values import HardwareInterfaceReturnValues

from .motor import Motor


class TB6612FNGHardwareInterface(SystemInterface):
    """
    Python implementation of the ROS2 hardware interface for the TB6612FNG motor driver.
    """

    def __init__(self):
        """Initialize the hardware interface"""
        super().__init__()
        self.left_velocity_command = 0.0
        self.right_velocity_command = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        
        # Motors
        self.left_motor = None
        self.right_motor = None
        
        # GPIO
        self.chip = None
        self.standby_line = None
        
        # Parameters
        self.left_in1_pin = 0
        self.left_in2_pin = 0
        self.left_pwm_pin = 0
        self.right_in1_pin = 0
        self.right_in2_pin = 0
        self.right_pwm_pin = 0
        self.standby_pin = 0
        self.left_reversed = False
        self.right_reversed = False
        self.gpio_chip_name = "gpiochip4"
        
        # Current state
        self.initialized = False
        self.hw_info = None
        
        # Max velocity (rad/s) that corresponds to 100% duty cycle
        self.max_velocity = 10.0

    def on_init(self, hardware_info):
        """
        Initialize the hardware interface from hardware info.
        
        Args:
            hardware_info: Hardware info structure containing parameters
            
        Returns:
            Hardware interface return value
        """
        if super().on_init(hardware_info) != HardwareInterfaceReturnValues.SUCCESS:
            return HardwareInterfaceReturnValues.ERROR
            
        # Store hardware info
        self.hw_info = hardware_info
        
        # Check if we have the expected number of joints
        if len(hardware_info.joints) != 2:
            self.get_logger().error("Expected exactly 2 joints (left and right wheels)")
            return HardwareInterfaceReturnValues.ERROR
        
        # Get parameters from hardware info
        try:
            self.gpio_chip_name = hardware_info.hardware_parameters.get("gpio_chip", "gpiochip4")
            self.left_in1_pin = int(hardware_info.hardware_parameters.get("left_in1_pin", "8"))
            self.left_in2_pin = int(hardware_info.hardware_parameters.get("left_in2_pin", "7"))
            self.left_pwm_pin = int(hardware_info.hardware_parameters.get("left_pwm_pin", "12"))
            self.right_in1_pin = int(hardware_info.hardware_parameters.get("right_in1_pin", "23"))
            self.right_in2_pin = int(hardware_info.hardware_parameters.get("right_in2_pin", "24"))
            self.right_pwm_pin = int(hardware_info.hardware_parameters.get("right_pwm_pin", "13"))
            self.standby_pin = int(hardware_info.hardware_parameters.get("standby_pin", "25"))
            
            
            # Optional parameters
            self.left_reversed = hardware_info.hardware_parameters.get("left_reversed", True)
            self.right_reversed = hardware_info.hardware_parameters.get("right_reversed", False)
            # Check if we want to override the max velocity
            if "max_velocity" in hardware_info.hardware_parameters:
                self.max_velocity = float(hardware_info.hardware_parameters.get("max_velocity"))
            
        except (KeyError, ValueError) as e:
            self.get_logger().error(f"Error parsing parameters: {str(e)}")
            return HardwareInterfaceReturnValues.ERROR
        
        self.get_logger().info(
            f"TB6612FNG Hardware Interface initialized with:\n" +
            f"  GPIO chip: {self.gpio_chip_name}\n" +
            f"  Left motor pins: IN1={self.left_in1_pin}, IN2={self.left_in2_pin}, PWM={self.left_pwm_pin}\n" +
            f"  Right motor pins: IN1={self.right_in1_pin}, IN2={self.right_in2_pin}, PWM={self.right_pwm_pin}\n" +
            f"  Standby pin: {self.standby_pin}\n" +
            f"  Left motor reversed: {self.left_reversed}\n" +
            f"  Right motor reversed: {self.right_reversed}\n" +
            f"  Max velocity: {self.max_velocity} rad/s"
        )
        
        # Mark as initialized
        self.initialized = True
        
        return HardwareInterfaceReturnValues.SUCCESS
    
    def on_configure(self, previous_state):
        """
        Configure the hardware interface.
        
        Args:
            previous_state: Previous lifecycle state
            
        Returns:
            Hardware interface return value
        """
        # Nothing to configure
        return HardwareInterfaceReturnValues.SUCCESS
        
    def on_cleanup(self, previous_state):
        """
        Clean up the hardware interface.
        
        Args:
            previous_state: Previous lifecycle state
            
        Returns:
            Hardware interface return value
        """
        # Clean up motors if they exist
        self.left_motor = None
        self.right_motor = None
        
        # Release the standby line if we have it
        if self.standby_line:
            try:
                self.standby_line.release()
                self.standby_line = None
            except Exception as e:
                self.get_logger().error(f"Error releasing standby line: {str(e)}")
        
        # Close the GPIO chip if we have it
        if self.chip:
            try:
                self.chip.close()
                self.chip = None
            except Exception as e:
                self.get_logger().error(f"Error closing GPIO chip: {str(e)}")
        
        return HardwareInterfaceReturnValues.SUCCESS
    
    def on_activate(self, previous_state):
        """
        Activate the hardware interface.
        
        Args:
            previous_state: Previous lifecycle state
            
        Returns:
            Hardware interface return value
        """
        # Create motor instances
        try:
            # First, try to open the GPIO chip
            self.get_logger().info(f"Opening GPIO chip {self.gpio_chip_name}")
            self.chip = gpiod.Chip(self.gpio_chip_name)
            
            # Get and configure the standby line
            self.get_logger().info(f"Configuring standby pin {self.standby_pin}")
            self.standby_line = self.chip.get_line(self.standby_pin)
            self.standby_line.request(consumer="motors", type=gpiod.LINE_REQ_DIR_OUT)
            self.standby_line.set_value(1)  # Enable the driver
            
            # For the left motor, create an instance and initialize with proper pins
            self.get_logger().info("Creating left motor")
            self.left_motor = Motor(
                self.left_in1_pin,
                self.left_in2_pin,
                self.left_pwm_pin,
                self.standby_pin,
                self.left_reversed,
                self.gpio_chip_name,
                shared_standby_line=self.standby_line
            )
            
            # Create right motor with shared standby line
            self.get_logger().info("Creating right motor")
            self.right_motor = Motor(
                self.right_in1_pin,
                self.right_in2_pin,
                self.right_pwm_pin,
                self.standby_pin,
                self.right_reversed,
                self.gpio_chip_name,
                shared_standby_line=self.standby_line
            )
            
            self.get_logger().info("Motors initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motors: {str(e)}")
            return HardwareInterfaceReturnValues.ERROR
            
        return HardwareInterfaceReturnValues.SUCCESS
    
    def on_deactivate(self, previous_state):
        """
        Deactivate the hardware interface.
        
        Args:
            previous_state: Previous lifecycle state
            
        Returns:
            Hardware interface return value
        """
        # Stop motors
        if self.left_motor:
            try:
                self.left_motor.brake()
            except Exception as e:
                self.get_logger().error(f"Error stopping left motor: {str(e)}")
                
        if self.right_motor:
            try:
                self.right_motor.brake()
            except Exception as e:
                self.get_logger().error(f"Error stopping right motor: {str(e)}")
        
        # Clean up motors
        self.left_motor = None
        self.right_motor = None
        
        # Disable the motor driver via standby
        if self.standby_line:
            try:
                self.standby_line.set_value(0)  # Disable the driver
            except Exception as e:
                self.get_logger().error(f"Error setting standby line: {str(e)}")
                
        # Release GPIO resources
        self.on_cleanup(previous_state)
        
        return HardwareInterfaceReturnValues.SUCCESS

    def export_state_interfaces(self):
        """
        Export state interfaces for the hardware.
        
        Returns:
            List of state interfaces
        """
        state_interfaces = []
        
        # Assuming joint names from hardware info
        left_joint_name = self.hw_info.joints[0].name
        right_joint_name = self.hw_info.joints[1].name
        
        # Add position and velocity state interfaces for both wheels
        state_interfaces.append(
            self.create_state_interface(
                left_joint_name, 
                "position", 
                self.get_position_for_wheel("left")
            )
        )
        state_interfaces.append(
            self.create_state_interface(
                left_joint_name, 
                "velocity", 
                self.get_velocity_for_wheel("left")
            )
        )
        state_interfaces.append(
            self.create_state_interface(
                right_joint_name, 
                "position", 
                self.get_position_for_wheel("right")
            )
        )
        state_interfaces.append(
            self.create_state_interface(
                right_joint_name, 
                "velocity", 
                self.get_velocity_for_wheel("right")
            )
        )
        
        return state_interfaces
    
    def export_command_interfaces(self):
        """
        Export command interfaces for the hardware.
        
        Returns:
            List of command interfaces
        """
        command_interfaces = []
        
        # Assuming joint names from hardware info
        left_joint_name = self.hw_info.joints[0].name
        right_joint_name = self.hw_info.joints[1].name
        
        # Add velocity command interfaces for both wheels
        command_interfaces.append(
            self.create_command_interface(
                left_joint_name,
                "velocity",
                self.set_velocity_for_wheel("left")
            )
        )
        command_interfaces.append(
            self.create_command_interface(
                right_joint_name,
                "velocity",
                self.set_velocity_for_wheel("right")
            )
        )
        
        return command_interfaces
    
    def read(self, time, period):
        """
        Read current state from the hardware.
        
        Args:
            time: Current time
            period: Period since last read
            
        Returns:
            Hardware interface return value
        """
        # Update position based on velocity and period
        # Note: In a real implementation, you might read encoder values instead
        period_seconds = period.nanoseconds / 1e9
        
        # Update positions based on current velocity
        self.left_position += self.left_velocity * period_seconds
        self.right_position += self.right_velocity * period_seconds
        
        # Apply some decay to velocity
        # This simulates friction and helps stop the robot when commands stop
        decay_factor = 0.95
        self.left_velocity *= decay_factor
        self.right_velocity *= decay_factor
        
        return HardwareInterfaceReturnValues.SUCCESS
    
    def write(self, time, period):
        """
        Write current commands to the hardware.
        
        Args:
            time: Current time
            period: Period since last write
            
        Returns:
            Hardware interface return value
        """
        # Update the velocity (apply command)
        self.left_velocity = self.left_velocity_command
        self.right_velocity = self.right_velocity_command
        
        if self.left_motor and self.right_motor:
            try:
                # Convert velocity to duty cycle (0-100)
                left_duty_cycle = self.velocity_to_duty_cycle(self.left_velocity_command)
                right_duty_cycle = self.velocity_to_duty_cycle(self.right_velocity_command)
                
                # Apply the duty cycle to the motors
                self.left_motor.drive(left_duty_cycle)
                self.right_motor.drive(right_duty_cycle)
            except Exception as e:
                self.get_logger().error(f"Error writing to motors: {str(e)}")
                return HardwareInterfaceReturnValues.ERROR
        
        return HardwareInterfaceReturnValues.SUCCESS
    
    def get_position_for_wheel(self, wheel):
        """
        Get position getter function for the given wheel.
        
        Args:
            wheel: Wheel to get position for (left or right)
            
        Returns:
            Function to get position
        """
        
        def _get_position():
            if wheel == "left":
                return self.left_position
            else:
                return self.right_position
        
        return _get_position
    
    def get_velocity_for_wheel(self, wheel):
        """
        Get velocity getter function for the given wheel.
        
        Args:
            wheel: Wheel to get velocity for (left or right)
            
        Returns:
            Function to get velocity
        """
        
        def _get_velocity():
            if wheel == "left":
                return self.left_velocity
            else:
                return self.right_velocity
        
        return _get_velocity
    
    def set_velocity_for_wheel(self, wheel):
        """
        Get velocity setter function for the given wheel.
        
        Args:
            wheel: Wheel to set velocity for (left or right)
            
        Returns:
            Function to set velocity
        """
        
        def _set_velocity(value):
            if wheel == "left":
                self.left_velocity_command = value
            else:
                self.right_velocity_command = value
        
        return _set_velocity
    
    def velocity_to_duty_cycle(self, velocity):
        """
        Convert velocity in rad/s to duty cycle (0-100).
        
        Args:
            velocity: Velocity in rad/s
            
        Returns:
            Duty cycle (0-100)
        """
        # Clamp the velocity to the max
        clamped_velocity = max(-self.max_velocity, min(velocity, self.max_velocity))
        
        # Convert to duty cycle (0-100)
        duty_cycle = (clamped_velocity / self.max_velocity) * 100.0
        
        return duty_cycle
    
    def get_logger(self):
        """
        Get the ROS logger instance.
        
        Returns:
            Logger instance
        """
        try:
            # Try to use the ROS node logger if running in a ROS context
            if rclpy.ok() and rclpy.utilities.get_default_context().ok():
                node = Node('tb6612fng_hardware_interface_logger')
                return node.get_logger()
            else:
                # Fallback to a dummy logger
                class DummyLogger:
                    def info(self, msg):
                        print(f"INFO: {msg}")
                    def warn(self, msg):
                        print(f"WARN: {msg}")
                    def error(self, msg):
                        print(f"ERROR: {msg}")
                    def debug(self, msg):
                        print(f"DEBUG: {msg}")
                
                return DummyLogger()
        except Exception:
            # Fallback to a dummy logger
            class DummyLogger:
                def info(self, msg):
                    print(f"INFO: {msg}")
                def warn(self, msg):
                    print(f"WARN: {msg}")
                def error(self, msg):
                    print(f"ERROR: {msg}")
                def debug(self, msg):
                    print(f"DEBUG: {msg}")
            
            return DummyLogger()


def export_plugin(plugin_name="robot_control_py/TB6612FNGHardwareInterface"):
    """
    Export the TB6612FNG hardware interface as a plugin.
    
    Args:
        plugin_name: Plugin name
        
    Returns:
        TB6612FNG hardware interface
    """
    return TB6612FNGHardwareInterface 