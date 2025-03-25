#include "motor_interface/unibots_motor_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/transition.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace motor_interface
{

UnibotsMotorHardwareInterface::UnibotsMotorHardwareInterface()
    : motor_controller_(std::make_unique<MotorController>())
{
}

UnibotsMotorHardwareInterface::~UnibotsMotorHardwareInterface()
{
}

rclcpp::Logger UnibotsMotorHardwareInterface::get_logger()
{
    return rclcpp::get_logger("UnibotsMotorHardwareInterface");
}

hardware_interface::CallbackReturn UnibotsMotorHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Get GPIO pins from URDF parameters
    int left_in1_pin = std::stoi(info_.hardware_parameters["left_in1_pin"]);
    int left_in2_pin = std::stoi(info_.hardware_parameters["left_in2_pin"]);
    int left_pwm_pin = std::stoi(info_.hardware_parameters["left_pwm_pin"]);
    int right_in1_pin = std::stoi(info_.hardware_parameters["right_in1_pin"]);
    int right_in2_pin = std::stoi(info_.hardware_parameters["right_in2_pin"]);
    int right_pwm_pin = std::stoi(info_.hardware_parameters["right_pwm_pin"]);
    int standby_pin = std::stoi(info_.hardware_parameters["standby_pin"]);

    // Get wheel parameters
    double wheel_separation = std::stod(info_.hardware_parameters["wheel_separation"]);
    double wheel_radius = std::stod(info_.hardware_parameters["wheel_radius"]);

    // Get motor direction parameters
    bool left_reversed = info_.hardware_parameters["left_reversed"] == "true";
    bool right_reversed = info_.hardware_parameters["right_reversed"] == "true";

    // Get PWM range parameters
    int min_pwm = std::stoi(info_.hardware_parameters["min_pwm"]);
    int max_pwm = std::stoi(info_.hardware_parameters["max_pwm"]);

    // Initialize motor controller
    if (!motor_controller_->initialize(
        left_in1_pin, left_in2_pin, left_pwm_pin,
        right_in1_pin, right_in2_pin, right_pwm_pin,
        standby_pin,
        wheel_separation, wheel_radius,
        left_reversed, right_reversed,
        min_pwm, max_pwm))
    {
        RCLCPP_ERROR(get_logger(), "Failed to initialize motor controller");
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // DiffBot has exactly one state and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu state interface. 1 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' have '%s' as state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnibotsMotorHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnibotsMotorHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
    motor_controller_->cleanup();
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnibotsMotorHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    
    // Set standby pin high to enable motors
    motor_controller_->set_standby(true);

    // Set default values for hardware
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_states_.begin(), hw_states_.end(), 0.0);

    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnibotsMotorHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    
    // Set standby pin low to disable motors
    motor_controller_->set_standby(false);

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnibotsMotorHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Shutting down ...please wait...");
    motor_controller_->cleanup();
    RCLCPP_INFO(get_logger(), "Successfully shut down!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnibotsMotorHardwareInterface::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Error handling ...please wait...");
    motor_controller_->cleanup();
    RCLCPP_INFO(get_logger(), "Error handled!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnibotsMotorHardwareInterface::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    // Update the state with the current velocity
    if (info_.joints.size() >= 2)
    {
        hw_states_[0] = hw_commands_[0];  // Left wheel velocity
        hw_states_[1] = hw_commands_[1];  // Right wheel velocity
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnibotsMotorHardwareInterface::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    // Write commands to the motors
    if (info_.joints.size() >= 2)
    {
        motor_controller_->set_velocity(hw_commands_[0], hw_commands_[1]);
    }

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> UnibotsMotorHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UnibotsMotorHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_commands_[i]));
    }
    return command_interfaces;
}

}  // namespace motor_interface

PLUGINLIB_EXPORT_CLASS(motor_interface::UnibotsMotorHardwareInterface, hardware_interface::SystemInterface) 