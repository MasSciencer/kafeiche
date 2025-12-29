#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "kafeiche_drivers/motor.hpp"
#include "kafeiche_drivers/encoder.hpp"

#include <memory>
#include <chrono>

/* ============================================================
   Класс источника данных одного колеса (двигатель + энкодер)
   ============================================================ */

class WheelIO {
public:
    WheelIO(int dir_pin, int step_pin, int en_pin, bool invert, EncoderChannel encoder_ch)
        : motor_(std::make_shared<StepperClass>(dir_pin, step_pin, en_pin, invert)),
          encoder_(std::make_shared<EncoderClass>()),
          encoder_channel_(encoder_ch)
    {}

    void setTargetVelocity(double vel_mps) {
        motor_->setSpeed(vel_mps);
    }

    double readWheelVelocity() {
        return encoder_->getVelocity(encoder_channel_) * WHEEL_RADIUS_M;
    }

private:
    std::shared_ptr<StepperClass> motor_;
    std::shared_ptr<EncoderClass> encoder_;
    EncoderChannel encoder_channel_;
};


/* ============================================================
                      ROS2 Node
   ============================================================ */

class ServoMotorNode : public rclcpp::Node {
public:
    ServoMotorNode()
        : Node("servo_motor"),
          wheel_left_(MOTOR_LEFT_DIR_PIN,
                      MOTOR_LEFT_STEP_PIN,
                      MOTOR_ENABLE_PIN,
                      false,
                      EncoderChannel::LEFT),

          wheel_right_(MOTOR_RIGHT_DIR_PIN,
                       MOTOR_RIGHT_STEP_PIN,
                       MOTOR_ENABLE_PIN,
                       true,
                       EncoderChannel::RIGHT)
    {
        /* --- подписки команд скорости --- */
        sub_left_target_vel_ = create_subscription<std_msgs::msg::Float64>(
            "/kfc/left_wheel/target_velocity",
            rclcpp::QoS(10),
            std::bind(&ServoMotorNode::leftTargetCallback, this, std::placeholders::_1));

        sub_right_target_vel_ = create_subscription<std_msgs::msg::Float64>(
            "/kfc/right_wheel/target_velocity",
            rclcpp::QoS(10),
            std::bind(&ServoMotorNode::rightTargetCallback, this, std::placeholders::_1));

        /* --- публикации фактической скорости --- */
        pub_left_vel_ = create_publisher<std_msgs::msg::Float64>(
            "/kfc/left_wheel/current_velocity", rclcpp::QoS(10));

        pub_right_vel_ = create_publisher<std_msgs::msg::Float64>(
            "/kfc/right_wheel/current_velocity", rclcpp::QoS(10));

        /* --- таймер опроса энкодеров --- */
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ServoMotorNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "servo_motor node started");
    }

private:
    /* ---------------------- Callbacks ---------------------- */

    void leftTargetCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        wheel_left_.setTargetVelocity(msg->data);
    }

    void rightTargetCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        wheel_right_.setTargetVelocity(msg->data);
    }

    void timerCallback() {
        auto left_vel  = wheel_left_.readWheelVelocity();
        auto right_vel = wheel_right_.readWheelVelocity();

        std_msgs::msg::Float64 msg_left;
        std_msgs::msg::Float64 msg_right;

        msg_left.data  = left_vel;
        msg_right.data = right_vel;

        pub_left_vel_->publish(msg_left);
        pub_right_vel_->publish(msg_right);
    }

    /* ---------------------- Members ---------------------- */

    WheelIO wheel_left_;
    WheelIO wheel_right_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_target_vel_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_right_target_vel_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_vel_;

    rclcpp::TimerBase::SharedPtr timer_;
};


/* ============================================================
                           main()
   ============================================================ */

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoMotorNode>());
    rclcpp::shutdown();
    return 0;
}