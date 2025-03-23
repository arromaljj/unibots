#!/usr/bin/env python3
from motor import Motor
import gpiod  # Missing import
from time import sleep

def main():
    # Create motor instances
    # Motor(IN1, IN2, PWM, STANDBY, Reverse)
    # For a robot with two motors
    
    # First, create the chip and get the standby line
    chip = gpiod.Chip("gpiochip4")
    standby_pin = 25
    standby_line = chip.get_line(standby_pin)
    standby_line.request(consumer="motors", type=gpiod.LINE_REQ_DIR_OUT)
    
    # Then create the motors, passing the standby_line to each
    left_motor = Motor(8, 7, 12, standby_pin, True, shared_standby_line=standby_line)  # Left motor
    right_motor = Motor(23, 24, 13, standby_pin, False, shared_standby_line=standby_line)  # Right motor (reversed)
    
    try:
        print("Starting motor test sequence...")
        
        # Drive forward
        print("Moving forward")
        left_motor.drive(70)   # 70% duty cycle
        right_motor.drive(70)  # 70% duty cycle
        sleep(2)
        
        # Drive backward
        print("Moving backward")
        left_motor.drive(-70)   # 70% duty cycle in reverse
        right_motor.drive(-70)  # 70% duty cycle in reverse
        sleep(2)
        
        # Turn right (left motor forward, right motor backward)
        print("Turning right")
        left_motor.drive(80)
        right_motor.drive(-80)
        sleep(1.5)
        
        # Turn left (left motor backward, right motor forward)
        print("Turning left")
        left_motor.drive(-80)
        right_motor.drive(80)
        sleep(1.5)
        
        # Variable speed demonstration
        print("Variable speed test")
        for speed in range(0, 101, 10):
            print(f"Speed: {speed}%")
            left_motor.drive(speed)
            right_motor.drive(speed)
            sleep(0.3)
            
        # Slow down
        for speed in range(100, -1, -10):
            print(f"Speed: {speed}%")
            left_motor.drive(speed)
            right_motor.drive(speed)
            sleep(0.3)
        
        # Short brake demonstration
        print("Testing brake")
        left_motor.drive(100)
        right_motor.drive(100)
        sleep(1)
        print("Braking!")
        left_motor.brake()
        right_motor.brake()
        sleep(2)
        
        # Standby mode demonstration
        print("Testing standby mode")
        # First drive motors
        left_motor.drive(50)
        right_motor.drive(50)
        sleep(1)
        
        # Enter standby
        print("Entering standby")
        left_motor.standby(True)
        # Only one standby pin is needed if motors share standby
        sleep(2)
        
        # Exit standby
        print("Exiting standby")
        left_motor.standby(False)
        sleep(1)
        
        # Final braking
        left_motor.brake()
        right_motor.brake()
        
        print("Test sequence complete!")
        
    except KeyboardInterrupt:
        # Handle Ctrl+C
        print("Program stopped by user")
    finally:
        # Always clean up
        print("Cleaning up...")
        left_motor.brake()
        right_motor.brake()
        del left_motor
        del right_motor
        # Release the shared standby line
        standby_line.release()
        chip.close()
        print("Done!")

if __name__ == "__main__":
    main()
