#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/float64.hpp"

#include "kafeiche_drivers/controller.hpp"

namespace kafeiche_drivers
{

hardware_interface::CallbackReturn DiffKfc::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // -------------------------
    // Wheel internal structure
    // -------------------------
    left_wheel_.name     = "left_wheel_joint";
    right_wheel_.name    = "right_wheel_joint";
    

    left_wheel_.velocity = 0.0;
    right_wheel_.velocity = 0.0;

    left_wheel_.command  = 0.0;
    right_wheel_.command = 0.0;

    // Создаём Node для общения с servo_motor
    node_ = std::make_shared<rclcpp::Node>("kfc_hw_interface");

    // Publishers → servo_motor (target velocities)
    pub_left_cmd_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/kfc/left_wheel/target_velocity", rclcpp::QoS(10));

    pub_right_cmd_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/kfc/right_wheel/target_velocity", rclcpp::QoS(10));

    // Subscriptions ← servo_motor (current velocities)
    sub_left_state_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/kfc/left_wheel/current_velocity",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64::SharedPtr msg)
        {
            left_wheel_.velocity = msg->data;
        });

    sub_right_state_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/kfc/right_wheel/current_velocity",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64::SharedPtr msg)
        {
            right_wheel_.velocity = msg->data;
        });

    RCLCPP_INFO(node_->get_logger(), "DiffKfc hardware_interface initialized");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffKfc::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            left_wheel_.name,
            hardware_interface::HW_IF_VELOCITY,
            &left_wheel_.velocity));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            right_wheel_.name,
            hardware_interface::HW_IF_VELOCITY,
            &right_wheel_.velocity));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffKfc::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            left_wheel_.name,
            hardware_interface::HW_IF_VELOCITY,
            &left_wheel_.command));

    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            right_wheel_.name,
            hardware_interface::HW_IF_VELOCITY,
            &right_wheel_.command));

    return command_interfaces;
}

hardware_interface::CallbackReturn DiffKfc::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(node_->get_logger(), "DiffKfc configured");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/* =====================================================
   READ: вызывается DiffDriveController для получения
         фактических скоростей колёс
   ===================================================== */
hardware_interface::return_type DiffKfc::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    // Ничего не делаем — скорости обновляются в callback подписки
    return hardware_interface::return_type::OK;
}

/* =====================================================
   WRITE: вызывается DiffDriveController,
          чтобы отправить целевые скорости
          → servo_motor.cpp
   ===================================================== */
hardware_interface::return_type DiffKfc::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    std_msgs::msg::Float64 msg_left;
    msg_left.data = left_wheel_.command;
    pub_left_cmd_->publish(msg_left);

    std_msgs::msg::Float64 msg_right;
    msg_right.data = right_wheel_.command;
    pub_right_cmd_->publish(msg_right);

    return hardware_interface::return_type::OK;
}

}  // namespace kafeiche_drivers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kafeiche_drivers::DiffKfc, hardware_interface::SystemInterface)
