#ifndef UNIBOTS_MOTOR_HARDWARE_INTERFACE_HPP_
#define UNIBOTS_MOTOR_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/transition.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <vector>
#include <memory>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "motor_interface/motor_controller.hpp"

namespace motor_interface
{

class UnibotsMotorHardwareInterface : public hardware_interface::SystemInterface
{
public:
    UnibotsMotorHardwareInterface();
    ~UnibotsMotorHardwareInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    rclcpp::Logger get_logger();

    std::unique_ptr<MotorController> motor_controller_;
    std::vector<double> hw_states_;
    std::vector<double> hw_commands_;
    
    // ROS2 node and publisher
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> motor_state_pub_;
};

}  // namespace motor_interface

#endif  // UNIBOTS_MOTOR_HARDWARE_INTERFACE_HPP_ 