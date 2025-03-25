#ifndef MOTOR_CONTROLLER_HPP_
#define MOTOR_CONTROLLER_HPP_

#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <gpiod.h>

namespace motor_interface
{

class MotorController
{
public:
    MotorController();
    ~MotorController();

    bool initialize(
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
        int max_pwm);

    void cleanup();
    void set_velocity(double left_velocity, double right_velocity);
    void set_standby(bool enable);
    void stop();

private:
    bool initialize_gpio();
    void cleanup_gpio();
    void set_gpio_value(struct gpiod_line* line, int value);
    void start_pwm();
    void stop_pwm();
    void pwm_thread_func();
    void control_left_motor(double speed);
    void control_right_motor(double speed);
    double scale_pwm(double speed);

    // GPIO pins
    int left_motor_in1_;
    int left_motor_in2_;
    int left_motor_pwm_;
    int right_motor_in1_;
    int right_motor_in2_;
    int right_motor_pwm_;
    int standby_pin_;

    // libgpiod handles
    struct gpiod_chip* chip_;
    struct gpiod_line* left_in1_line_;
    struct gpiod_line* left_in2_line_;
    struct gpiod_line* left_pwm_line_;
    struct gpiod_line* right_in1_line_;
    struct gpiod_line* right_in2_line_;
    struct gpiod_line* right_pwm_line_;
    struct gpiod_line* standby_line_;

    // Motor parameters
    double wheel_separation_;
    double wheel_radius_;
    bool left_reversed_;
    bool right_reversed_;
    int min_pwm_;
    int max_pwm_;

    // State variables
    bool gpio_initialized_;
    bool pwm_running_;
    std::atomic<bool> should_exit_;
    std::atomic<double> left_motor_duty_cycle_;
    std::atomic<double> right_motor_duty_cycle_;
    std::thread pwm_thread_;
};

}  // namespace motor_interface

#endif  // MOTOR_CONTROLLER_HPP_