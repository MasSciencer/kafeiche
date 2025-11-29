#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include <wiringPi.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// GPIO layout
constexpr uint8_t MOTOR_LEFT_DIR_PIN = 26;
constexpr uint8_t MOTOR_LEFT_STEP_PIN = 13;
constexpr uint8_t MOTOR_ENABLE_PIN = 12;
constexpr uint8_t MOTOR_RIGHT_DIR_PIN = 27;
constexpr uint8_t MOTOR_RIGHT_STEP_PIN = 4;

// Physical constraints
constexpr double WHEEL_RADIUS_M = 0.075;
constexpr double GEAR_RATIO = 3.7;
constexpr uint16_t MAX_RPS = 20;
constexpr uint16_t MIN_RPS = 0.2;

// Stepper configuration
constexpr uint16_t STEPS_PER_REV = 200;
constexpr uint16_t MICROSTEP = 2; // 1/1 microstepping

class StepperClass {
public:
    StepperClass(int8_t direction_pin, int8_t step_pin, int8_t enable_pin, bool invert_direction = false);
    ~StepperClass();

    void setSpeed(double target_linear_vel_mps);

private:
    void initializePins();
    void run();
    void stopMotor();
    uint16_t rpsToDelayMicros(uint16_t rps) const;

    std::thread motor_thread_;
    std::atomic<bool> running_{true};
    std::atomic<int16_t> target_rps_{0};

    int8_t direction_pin_;
    int8_t step_pin_;
    int8_t enable_pin_;

    bool invert_direction_;
};

inline StepperClass::StepperClass(int8_t direction_pin,
                                                  int8_t step_pin,
                                                  int8_t enable_pin,
                                                  bool invert_direction)
    : direction_pin_(direction_pin),
      step_pin_(step_pin),
      enable_pin_(enable_pin),
      invert_direction_(invert_direction)
{
    if (wiringPiSetupGpio() == -1) {
        throw std::runtime_error("Failed to initialize wiringPi");
    }

    initializePins();
    motor_thread_ = std::thread(&StepperClass::run, this);
}

inline StepperClass::~StepperClass() {
    running_ = false;
    if (motor_thread_.joinable()) {
        motor_thread_.join();
    }
}

inline void StepperClass::initializePins() {
    pinMode(direction_pin_, OUTPUT);
    pinMode(step_pin_, OUTPUT);
    pinMode(enable_pin_, OUTPUT);
}

inline void StepperClass::setSpeed(double target_linear_vel_mps) {
    const double wheel_circumference = 2.0 * M_PI * WHEEL_RADIUS_M;
    const double wheel_rps =
        (target_linear_vel_mps * GEAR_RATIO) / wheel_circumference;
    target_rps_ = static_cast<int16_t>(wheel_rps);
}

void StepperClass::run() {
    while (running_) {

        int16_t target = target_rps_;

        if (target == 0) {
            stopMotor();
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // маленькая пауза
            continue;
        }

        int16_t rps = std::clamp<int16_t>(std::abs(target), MIN_RPS, MAX_RPS);

        bool dir = (target > 0);
        if (invert_direction_) {
            dir = !dir;
        }

        uint16_t step_delay = rpsToDelayMicros(rps);

        digitalWrite(enable_pin_, LOW);
        digitalWrite(direction_pin_, dir ? HIGH : LOW);

        digitalWrite(step_pin_, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_pin_, LOW);
        delayMicroseconds(step_delay);
    }
}



inline void StepperClass::stopMotor() {
    digitalWrite(enable_pin_, HIGH);
}

inline uint16_t StepperClass::rpsToDelayMicros(uint16_t rps) const {
    if (rps == 0) return 0;
    const uint32_t steps_per_second = (static_cast<uint32_t>(rps) * STEPS_PER_REV * MICROSTEP);
    return static_cast<uint16_t>(1000000U / steps_per_second);
}

#endif  // MOTOR_HPP