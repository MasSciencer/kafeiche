#ifndef KAFEICHE_DRIVERS_CONTROLLER_HPP
#define KAFEICHE_DRIVERS_CONTROLLER_HPP

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"

#include "kafeiche_drivers/motor.hpp"
#include "kafeiche_drivers/encoder.hpp"

namespace kafeiche_drivers
{

/**
 * @brief ROS2 Control system interface for differential drive.
 *
 * Implements **angular velocity** state and command interfaces (rad/s)
 * for left and right wheels. Interacts with MotorClass and EncoderClass to
 * control hardware directly.
 */
class DiffKfc : public hardware_interface::SystemInterface
{
public:
    // lifecycle callbacks
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

    struct Wheel
    {
        std::string name;
        double velocity = 0.0;  // state (rad/s)
        double command  = 0.0;  // target angular velocity from controller
    };

    Wheel left_wheel_;
    Wheel right_wheel_;

    // direct hardware objects (motors + shared encoder)
    std::shared_ptr<MotorClass> motor_left_;
    std::shared_ptr<MotorClass> motor_right_;
    std::shared_ptr<EncoderClass> encoder_;
};

} // namespace kafeiche_drivers

#endif // KAFEICHE_DRIVERS_CONTROLLER_HPP
