# Raspberry Pi TB6612FNG Library
import gpiod
import time

class Motor:
    # Defaults
    hertz = 1000
    reverse = False  # Reverse flips the direction of the motor
    
    # Constructor with optional shared_standby_line parameter
    def __init__(self, in1, in2, pwm, standbyPin, reverse, chip_name="gpiochip4", shared_standby_line=None):
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.standbyPin = standbyPin
        self.reverse = reverse
        self.owns_standby = shared_standby_line is None
        
        # Open the GPIO chip
        self.chip = gpiod.Chip(chip_name)
        
        # Configure the GPIO lines
        self.in1_line = self.chip.get_line(in1)
        self.in2_line = self.chip.get_line(in2)
        self.pwm_line = self.chip.get_line(pwm)
        
        # Request the lines for output
        self.in1_line.request(consumer="motor", type=gpiod.LINE_REQ_DIR_OUT)
        self.in2_line.request(consumer="motor", type=gpiod.LINE_REQ_DIR_OUT)
        self.pwm_line.request(consumer="motor", type=gpiod.LINE_REQ_DIR_OUT)
        
        # Handle standby line - either use provided or get our own
        if shared_standby_line:
            self.standby_line = shared_standby_line
        else:
            # Request our own standby line
            self.standby_line = self.chip.get_line(standbyPin)
            self.standby_line.request(consumer="motor", type=gpiod.LINE_REQ_DIR_OUT)
        
        # Set standby high to enable the motor driver
        self.standby_line.set_value(1)
        
        # Store the current duty cycle
        self.current_duty_cycle = 0
        self._start_pwm()
    
    # Simulate PWM like in the original
    def _start_pwm(self):
        # Start a background thread for PWM
        import threading
        self.pwm_running = True
        self.pwm_thread = threading.Thread(target=self._pwm_thread_func)
        self.pwm_thread.daemon = True
        self.pwm_thread.start()
    
    def _pwm_thread_func(self):
        # Basic software PWM implementation
        cycle_time = 1.0 / self.hertz
        while self.pwm_running:
            if self.current_duty_cycle > 0:
                on_time = cycle_time * (self.current_duty_cycle / 100.0)
                off_time = cycle_time - on_time
                
                self.pwm_line.set_value(1)
                time.sleep(on_time)
                
                if off_time > 0:
                    self.pwm_line.set_value(0)
                    time.sleep(off_time)
            else:
                self.pwm_line.set_value(0)
                time.sleep(cycle_time)
    
    # Equivalent to ChangeDutyCycle
    def _set_duty_cycle(self, duty):
        self.current_duty_cycle = max(0, min(100, duty))
    
    # Speed from -100 to 100
    def drive(self, speed):
        # Negative speed for reverse, positive for forward
        # If necessary use reverse parameter in constructor
        dutyCycle = speed
        if(speed < 0):
            dutyCycle = dutyCycle * -1
        if(self.reverse):
            speed = speed * -1
        if(speed > 0):
            self.in1_line.set_value(1)
            self.in2_line.set_value(0)
        else:
            self.in1_line.set_value(0)
            self.in2_line.set_value(1)
        self._set_duty_cycle(dutyCycle)
    
    def brake(self):
        self._set_duty_cycle(0)
        self.in1_line.set_value(1)
        self.in2_line.set_value(1)
    
    def standby(self, value):
        self._set_duty_cycle(0)
        self.standby_line.set_value(1 if value else 0)
    
    def __del__(self):
        # Stop PWM thread
        self.pwm_running = False
        if hasattr(self, 'pwm_thread'):
            self.pwm_thread.join(timeout=1.0)
        
        # Release GPIO lines
        if hasattr(self, 'in1_line'):
            self.in1_line.release()
        if hasattr(self, 'in2_line'):
            self.in2_line.release()
        if hasattr(self, 'pwm_line'):
            self.pwm_line.release()
        
        # Only release the standby line if we own it
        if self.owns_standby and hasattr(self, 'standby_line'):
            self.standby_line.release()
        
        # Close the chip
        if hasattr(self, 'chip'):
            self.chip.close()