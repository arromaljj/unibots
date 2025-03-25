#include "motor_interface/motor_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unistd.h>
#include <string.h>

namespace motor_interface
{

MotorController::MotorController()
    : left_motor_in1_(0),
      left_motor_in2_(0),
      left_motor_pwm_(0),
      right_motor_in1_(0),
      right_motor_in2_(0),
      right_motor_pwm_(0),
      standby_pin_(0),
      chip_(nullptr),
      left_in1_line_(nullptr),
      left_in2_line_(nullptr),
      left_pwm_line_(nullptr),
      right_in1_line_(nullptr),
      right_in2_line_(nullptr),
      right_pwm_line_(nullptr),
      standby_line_(nullptr),
      wheel_separation_(0.0),
      wheel_radius_(0.0),
      left_reversed_(false),
      right_reversed_(false),
      min_pwm_(0),
      max_pwm_(100),
      gpio_initialized_(false),
      pwm_running_(false),
      should_exit_(false),
      left_motor_duty_cycle_(0.0),
      right_motor_duty_cycle_(0.0)
{
}

MotorController::~MotorController()
{
    cleanup();
}

bool MotorController::initialize(
    int left_in1_pin,
    int left_in2_pin,
    int left_pwm_pin,
    int right_in1_pin,
    int right_in2_pin,
    int right_pwm_pin,
    int standby_pin,
    double wheel_separation,
    double wheel_radius,
    bool left_reversed,
    bool right_reversed,
    int min_pwm,
    int max_pwm)
{
    // Store GPIO pins
    left_motor_in1_ = left_in1_pin;
    left_motor_in2_ = left_in2_pin;
    left_motor_pwm_ = left_pwm_pin;
    right_motor_in1_ = right_in1_pin;
    right_motor_in2_ = right_in2_pin;
    right_motor_pwm_ = right_pwm_pin;
    standby_pin_ = standby_pin;

    // Store motor parameters
    wheel_separation_ = wheel_separation;
    wheel_radius_ = wheel_radius;
    left_reversed_ = left_reversed;
    right_reversed_ = right_reversed;
    min_pwm_ = min_pwm;
    max_pwm_ = max_pwm;

    // Initialize GPIO
    if (!initialize_gpio())
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorController"), "Failed to initialize GPIO");
        return false;
    }

    // Start PWM thread
    start_pwm();

    return true;
}

void MotorController::cleanup()
{
    stop_pwm();
    cleanup_gpio();
}

void MotorController::set_velocity(double left_velocity, double right_velocity)
{
    // Scale velocities to PWM range
    double left_pwm = scale_pwm(left_velocity);
    double right_pwm = scale_pwm(right_velocity);

    // Apply direction reversal if needed
    if (left_reversed_) left_pwm = -left_pwm;
    if (right_reversed_) right_pwm = -right_pwm;

    // Control motors
    control_left_motor(left_pwm);
    control_right_motor(right_pwm);
}

void MotorController::set_standby(bool enable)
{
    if (standby_line_) {
        set_gpio_value(standby_line_, enable ? 1 : 0);
    }
}

void MotorController::stop()
{
    control_left_motor(0);
    control_right_motor(0);
}

bool MotorController::initialize_gpio()
{
    try
    {
        // Open GPIO chip - use the appropriate chip name for your board
        // For Raspberry Pi with 40-pin header, this is typically "gpiochip0"
        // For other boards, check with `gpiodetect` command
        RCLCPP_INFO(rclcpp::get_logger("MotorController"), "Attempting to open gpiochip4...");
        
        // Check if device exists and is accessible
        if (access("/dev/gpiochip4", R_OK | W_OK) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotorController"), 
                        "Cannot access /dev/gpiochip4: %s (errno: %d)", 
                        strerror(errno), errno);
            return false;
        }
        
        chip_ = gpiod_chip_open("/dev/gpiochip4");  // Use full path
        if (!chip_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotorController"), 
                        "Failed to open GPIO chip: %s (errno: %d)", 
                        strerror(errno), errno);
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("MotorController"), "Successfully opened gpiochip4");

        // Get GPIO lines
        left_in1_line_ = gpiod_chip_get_line(chip_, left_motor_in1_);
        left_in2_line_ = gpiod_chip_get_line(chip_, left_motor_in2_);
        left_pwm_line_ = gpiod_chip_get_line(chip_, left_motor_pwm_);
        right_in1_line_ = gpiod_chip_get_line(chip_, right_motor_in1_);
        right_in2_line_ = gpiod_chip_get_line(chip_, right_motor_in2_);
        right_pwm_line_ = gpiod_chip_get_line(chip_, right_motor_pwm_);
        standby_line_ = gpiod_chip_get_line(chip_, standby_pin_);

        // Check if all lines were acquired properly
        if (!left_in1_line_ || !left_in2_line_ || !left_pwm_line_ ||
            !right_in1_line_ || !right_in2_line_ || !right_pwm_line_ ||
            !standby_line_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotorController"), "Failed to get GPIO lines");
            cleanup_gpio();
            return false;
        }

        // Request lines for output
        if (gpiod_line_request_output(left_in1_line_, "motor_controller", 0) < 0 ||
            gpiod_line_request_output(left_in2_line_, "motor_controller", 0) < 0 ||
            gpiod_line_request_output(left_pwm_line_, "motor_controller", 0) < 0 ||
            gpiod_line_request_output(right_in1_line_, "motor_controller", 0) < 0 ||
            gpiod_line_request_output(right_in2_line_, "motor_controller", 0) < 0 ||
            gpiod_line_request_output(right_pwm_line_, "motor_controller", 0) < 0 ||
            gpiod_line_request_output(standby_line_, "motor_controller", 1) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotorController"), "Failed to request GPIO lines for output");
            cleanup_gpio();
            return false;
        }

        // Set standby high initially to enable the motor driver
        gpiod_line_set_value(standby_line_, 1);

        gpio_initialized_ = true;
        return true;
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorController"), 
                    "Exception during GPIO initialization: %s", e.what());
        cleanup_gpio();
        return false;
    }
}

void MotorController::cleanup_gpio()
{
    // Release all GPIO lines
    if (left_in1_line_) gpiod_line_release(left_in1_line_);
    if (left_in2_line_) gpiod_line_release(left_in2_line_);
    if (left_pwm_line_) gpiod_line_release(left_pwm_line_);
    if (right_in1_line_) gpiod_line_release(right_in1_line_);
    if (right_in2_line_) gpiod_line_release(right_in2_line_);
    if (right_pwm_line_) gpiod_line_release(right_pwm_line_);
    if (standby_line_) gpiod_line_release(standby_line_);

    // Reset line pointers to avoid double release
    left_in1_line_ = nullptr;
    left_in2_line_ = nullptr;
    left_pwm_line_ = nullptr;
    right_in1_line_ = nullptr;
    right_in2_line_ = nullptr;
    right_pwm_line_ = nullptr;
    standby_line_ = nullptr;

    // Close GPIO chip
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }

    gpio_initialized_ = false;
}

void MotorController::set_gpio_value(struct gpiod_line* line, int value)
{
    if (line) {
        if (gpiod_line_set_value(line, value) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MotorController"), "Failed to set GPIO value");
        }
    }
}

void MotorController::start_pwm()
{
    pwm_running_ = true;
    should_exit_ = false;
    pwm_thread_ = std::thread(&MotorController::pwm_thread_func, this);
}

void MotorController::stop_pwm()
{
    should_exit_ = true;
    if (pwm_thread_.joinable())
    {
        pwm_thread_.join();
    }
    pwm_running_ = false;
}

void MotorController::pwm_thread_func()
{
    const int hertz = 1000;
    const auto cycle_time = std::chrono::microseconds(1000000 / hertz);

    while (!should_exit_)
    {
        auto start_time = std::chrono::steady_clock::now();

        // Left motor PWM
        double left_duty = left_motor_duty_cycle_.load();
        if (left_duty > 0)
        {
            set_gpio_value(left_pwm_line_, 1);
            auto on_time = std::chrono::microseconds(static_cast<long long>(
                cycle_time.count() * left_duty / 100.0));
            std::this_thread::sleep_for(on_time);
            
            set_gpio_value(left_pwm_line_, 0);
            auto off_time = cycle_time - on_time;
            if (off_time.count() > 0) {
                std::this_thread::sleep_for(off_time);
            }
        }
        else
        {
            set_gpio_value(left_pwm_line_, 0);
            std::this_thread::sleep_for(cycle_time);
        }

        // Right motor PWM
        double right_duty = right_motor_duty_cycle_.load();
        if (right_duty > 0)
        {
            set_gpio_value(right_pwm_line_, 1);
            auto on_time = std::chrono::microseconds(static_cast<long long>(
                cycle_time.count() * right_duty / 100.0));
            std::this_thread::sleep_for(on_time);
            
            set_gpio_value(right_pwm_line_, 0);
            auto off_time = cycle_time - on_time;
            if (off_time.count() > 0) {
                std::this_thread::sleep_for(off_time);
            }
        }
        else
        {
            set_gpio_value(right_pwm_line_, 0);
            std::this_thread::sleep_for(cycle_time);
        }

        // Ensure we maintain the correct timing
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < cycle_time) {
            std::this_thread::sleep_for(cycle_time - elapsed);
        }
    }
}

void MotorController::control_left_motor(double speed)
{
    if (speed > 0)
    {
        set_gpio_value(left_in1_line_, 1);
        set_gpio_value(left_in2_line_, 0);
        left_motor_duty_cycle_ = std::abs(speed);
    }
    else if (speed < 0)
    {
        set_gpio_value(left_in1_line_, 0);
        set_gpio_value(left_in2_line_, 1);
        left_motor_duty_cycle_ = std::abs(speed);
    }
    else
    {
        set_gpio_value(left_in1_line_, 1);
        set_gpio_value(left_in2_line_, 1);
        left_motor_duty_cycle_ = 0;
    }
}

void MotorController::control_right_motor(double speed)
{
    if (speed > 0)
    {
        set_gpio_value(right_in1_line_, 1);
        set_gpio_value(right_in2_line_, 0);
        right_motor_duty_cycle_ = std::abs(speed);
    }
    else if (speed < 0)
    {
        set_gpio_value(right_in1_line_, 0);
        set_gpio_value(right_in2_line_, 1);
        right_motor_duty_cycle_ = std::abs(speed);
    }
    else
    {
        set_gpio_value(right_in1_line_, 1);
        set_gpio_value(right_in2_line_, 1);
        right_motor_duty_cycle_ = 0;
    }
}

double MotorController::scale_pwm(double speed)
{
    if (speed == 0)
        return 0;
        
    // Get the absolute value of the speed
    double abs_speed = std::abs(speed);
    
    // Scale the speed to the PWM range
    // Assuming speed is in rad/s, we'll convert to a percentage of max_pwm_
    // Adjust the max_rad_per_sec based on your robot's capabilities
    double max_rad_per_sec = 10.0;  
    double percentage = (abs_speed / max_rad_per_sec) * 100.0;
    
    // Apply the min/max PWM limits
    double scaled_speed = min_pwm_ + (percentage / 100.0) * (max_pwm_ - min_pwm_);
    
    // Clamp to valid range (0-100)
    scaled_speed = std::max(0.0, std::min(static_cast<double>(max_pwm_), scaled_speed));
    
    // Restore the sign
    return speed > 0 ? scaled_speed : -scaled_speed;
}

}  // namespace motor_interface