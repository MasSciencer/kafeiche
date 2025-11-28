#ifndef KAFEICHE_DRIVERS_CONTROLLER_HPP
#define KAFEICHE_DRIVERS_CONTROLLER_HPP

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace kafeiche_drivers
{

class DiffKfc : public hardware_interface::SystemInterface
{
public:
    // -------------------------
    // ROS2-Control lifecycle
    // -------------------------
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:

    // =============================
    // Internal structure of wheels
    // =============================
    struct Wheel
    {
        std::string name;
        double velocity = 0.0;  // state
        double command  = 0.0;  // target velocity from controller
    };

    Wheel left_wheel_;
    Wheel right_wheel_;

    // ROS2 node used by hardware interface
    rclcpp::Node::SharedPtr node_;

    // Publishers → servo_motor (target speeds)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_cmd_;

    // Subscribers ← servo_motor (current speeds)
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_state_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_right_state_;
};

} // namespace kafeiche_drivers

#endif // KAFEICHE_DRIVERS_CONTROLLER_HPP
