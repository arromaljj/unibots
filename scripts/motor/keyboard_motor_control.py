#!/usr/bin/env python3
from motor import Motor
import gpiod
import time
import sys
import tty
import termios
import threading

class KeyboardController:
    def __init__(self):
        # Setup motors
        self.chip = gpiod.Chip("gpiochip4")
        self.standby_pin = 25
        self.standby_line = self.chip.get_line(self.standby_pin)
        self.standby_line.request(consumer="motors", type=gpiod.LINE_REQ_DIR_OUT)
        
        # Create the motors, passing the standby_line to each
        self.left_motor = Motor(8, 7, 12, self.standby_pin, True, shared_standby_line=self.standby_line)
        self.right_motor = Motor(23, 24, 13, self.standby_pin, False, shared_standby_line=self.standby_line)
        
        # Control variables
        self.running = False
        self.listening = False
        self.test_duty_cycle = 20  # Starting duty cycle
        self.min_duty_cycle = 0
        self.max_duty_cycle = 100
        self.duty_step = 5  # Step size for duty cycle adjustment
        
    def getch(self):
        """Get a single character from standard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def keyboard_listener(self):
        """Thread function to listen for keyboard input"""
        while self.running:
            char = self.getch()
            
            # Handle escape sequences for arrow keys
            if char == '\x1b':  # Escape character
                try:
                    # Get the next two characters in the escape sequence
                    char = self.getch()  # Should be '['
                    if char == '[':
                        char = self.getch()  # Should be A, B, C, or D for arrows
                        if self.listening:
                            if char == 'D':  # Left arrow
                                print("Left motor forward")
                                self.left_motor.drive(self.test_duty_cycle)
                                self.right_motor.brake()
                            elif char == 'C':  # Right arrow
                                print("Right motor forward")
                                self.right_motor.drive(self.test_duty_cycle)
                                self.left_motor.brake()
                            elif char == 'A':  # Up arrow
                                print("Both motors forward")
                                self.left_motor.drive(self.test_duty_cycle)
                                self.right_motor.drive(self.test_duty_cycle)
                            elif char == 'B':  # Down arrow
                                print("Both motors backward")
                                self.left_motor.drive(-self.test_duty_cycle)
                                self.right_motor.drive(-self.test_duty_cycle)
                except:
                    pass  # Handle incomplete escape sequences
            
            # Handle spacebar to toggle listening mode
            elif char == ' ':  # Spacebar
                self.listening = not self.listening
                if self.listening:
                    print("Keyboard control activated! Use arrow keys to drive motors.")
                    print("Left arrow: Left motor | Right arrow: Right motor")
                    print("Up arrow: Both forward | Down arrow: Both backward")
                    print(f"Current duty cycle: {self.test_duty_cycle}%")
                else:
                    print("Keyboard control deactivated. Motors stopped.")
                    self.left_motor.brake()
                    self.right_motor.brake()
            
            # Handle duty cycle adjustment
            elif char == '+' or char == '=':  # Increase duty cycle (+ or = key)
                self.test_duty_cycle = min(self.test_duty_cycle + self.duty_step, self.max_duty_cycle)
                print(f"Duty cycle increased to: {self.test_duty_cycle}%")
            
            elif char == '-' or char == '_':  # Decrease duty cycle (- or _ key)
                self.test_duty_cycle = max(self.test_duty_cycle - self.duty_step, self.min_duty_cycle)
                print(f"Duty cycle decreased to: {self.test_duty_cycle}%")
            
            # Handle quit command
            elif char in ('q', 'Q'):  # Quit
                print("Quitting...")
                self.running = False
                self.left_motor.brake()
                self.right_motor.brake()
                break
            
            # If arrow keys are released, stop motors if we're in listening mode
            # Note: This is a simplified approach. In a full application, you might
            # want to handle key-up events differently.
            elif self.listening:
                self.left_motor.brake()
                self.right_motor.brake()
    
    def run(self):
        """Run the controller"""
        print("Motor Keyboard Controller")
        print("Press SPACEBAR to start/stop accepting keyboard input")
        print("Use ARROW KEYS to control motors")
        print("Use + and - keys to adjust motor speed")
        print(f"Starting duty cycle: {self.test_duty_cycle}%")
        print("Press Q to quit")
        
        self.running = True
        
        # Start keyboard listener in a separate thread
        keyboard_thread = threading.Thread(target=self.keyboard_listener)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        try:
            # Keep the main thread alive until running is False
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nProgram interrupted")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        self.running = False
        self.left_motor.brake()
        self.right_motor.brake()
        time.sleep(0.5)  # Give motors time to brake
        
        # Clean up GPIO resources
        del self.left_motor
        del self.right_motor
        self.standby_line.release()
        self.chip.close()
        print("Done!")

if __name__ == "__main__":
    controller = KeyboardController()
    controller.run()