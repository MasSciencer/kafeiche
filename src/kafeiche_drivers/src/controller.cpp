#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// directly control hardware
#include "kafeiche_drivers/motor.hpp"
#include "kafeiche_drivers/encoder.hpp"

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

    // initialize wheel metadata and command/state variables
    left_wheel_.name     = "left_wheel_joint";
    right_wheel_.name    = "right_wheel_joint";

    left_wheel_.velocity = 0.0;
    right_wheel_.velocity = 0.0;

    left_wheel_.command  = 0.0;
    right_wheel_.command = 0.0;

    // create hardware interfaces
    encoder_ = std::make_shared<EncoderClass>();

    motor_left_  = std::make_shared<MotorClass>(
        MOTOR_LEFT_DIR_PIN,
        MOTOR_LEFT_STEP_PIN,
        MOTOR_ENABLE_PIN,
        false);

    motor_right_ = std::make_shared<MotorClass>(
        MOTOR_RIGHT_DIR_PIN,
        MOTOR_RIGHT_STEP_PIN,
        MOTOR_ENABLE_PIN,
        true);

    // start with motors disabled
    motor_left_->setEnabled(false);
    motor_right_->setEnabled(false);

    RCLCPP_INFO(rclcpp::get_logger("DiffKfc"), "DiffKfc hardware_interface initialized");
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
    RCLCPP_INFO(rclcpp::get_logger("DiffKfc"), "DiffKfc configured");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffKfc::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    // read angular velocities from encoder (rad/s) measured at the motor shaft
    // the controller publishes wheel joint velocities, so we must convert
    // through the gearbox.  the motor/wheel gearbox ratio is defined in
    // motor.hpp as GEAR_RATIO (motor revs per wheel rev).
    float left_vel_rad = encoder_->getVelocity(EncoderChannel::LEFT);
    float right_vel_rad = encoder_->getVelocity(EncoderChannel::RIGHT);

    // convert to wheel angular velocity: ω_wheel = ω_motor / GEAR_RATIO
    left_wheel_.velocity  = left_vel_rad / GEAR_RATIO;
    right_wheel_.velocity = right_vel_rad / GEAR_RATIO;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffKfc::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    motor_left_->setSpeed(left_wheel_.command);
    motor_right_->setSpeed(right_wheel_.command);

    // enable if any motor has a nonzero setpoint
    bool any_active = motor_left_->isActive() || motor_right_->isActive();
    motor_left_->setEnabled(any_active);
    motor_right_->setEnabled(any_active);

    return hardware_interface::return_type::OK;
}

}  // namespace kafeiche_drivers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kafeiche_drivers::DiffKfc, hardware_interface::SystemInterface)
