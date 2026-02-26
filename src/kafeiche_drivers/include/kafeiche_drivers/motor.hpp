#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <algorithm>
#include <stdexcept>
#include <mutex>
#include <vector>
#include <numeric>
#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "kafeiche_drivers/pigpio_utils.hpp"  // shared helper for pigpio connection

// GPIO pins (BCM numbering)
constexpr uint8_t MOTOR_LEFT_DIR_PIN   = 26;
constexpr uint8_t MOTOR_LEFT_STEP_PIN  = 13;
constexpr uint8_t MOTOR_RIGHT_DIR_PIN  = 27;
constexpr uint8_t MOTOR_RIGHT_STEP_PIN = 4;
constexpr uint8_t MOTOR_ENABLE_PIN     = 12; // common enable for both motors (active LOW)

// Physical parameters
constexpr double WHEEL_RADIUS_M = 0.075;   ///< wheel radius in meters
constexpr double GEAR_RATIO     = 3.7;     ///< gearbox ratio

// Nominal motor limits
constexpr double MIN_MOTOR_RPS = 0;
constexpr double MAX_MOTOR_RPS = 20.0;

// Stepper configuration
constexpr uint16_t STEPS_PER_REV = 200;
constexpr uint16_t MICROSTEP     = 32;

// helper that constructs one shared waveform for all registered motors
namespace {
struct WaveManager {
    static std::mutex mutex;
    static int pi;
    static int wave_id;
    static std::vector<uint8_t> step_pins;
    static std::vector<double> speeds;
    static std::vector<bool> dirs;

    // make sure we have a handle to pigpiod
    static void ensurePi() {
        if (pi < 0) {
            pi = getPiHandle();
        }
    }

    static size_t registerMotor(uint8_t step_pin) {
        std::lock_guard<std::mutex> lock(mutex);
        ensurePi();
        size_t idx = step_pins.size();
        step_pins.push_back(step_pin);
        speeds.push_back(0.0);
        dirs.push_back(true);
        return idx;
    }

    static void setSpeed(size_t idx, double rps, bool dir) {
        std::lock_guard<std::mutex> lock(mutex);
        if (idx >= speeds.size()) return;
        if (speeds[idx] == rps && dirs[idx] == dir) {
            return; // nothing changed
        }
        speeds[idx] = rps;
        dirs[idx] = dir;
        rebuildWave();
    }

    static void rebuildWave() {
        // stop old waveform
        if (wave_id >= 0) {
            wave_tx_stop(pi);
            wave_delete(pi, wave_id);
            wave_id = -1;
        }

        // build timeline events for each motor
        struct Event { uint32_t time; uint32_t on; uint32_t off; };
        std::vector<Event> events;

        // determine pattern duration from half periods (lcm) capped at 100ms
        uint64_t pattern_us = 100000; // start with 100ms cap
        bool first = true;
        for (size_t i = 0; i < step_pins.size(); ++i) {
            double rps = speeds[i];
            if (rps <= 0.0) continue;
            double steps_per_sec = rps * STEPS_PER_REV * MICROSTEP;
            if (steps_per_sec <= 0.0) continue;
            uint64_t half = static_cast<uint64_t>(std::max(1.0, std::round(1e6 / (2.0 * steps_per_sec))));
            if (first) {
                pattern_us = half;
                first = false;
            } else {
                pattern_us = std::lcm(pattern_us, half);
            }
            if (pattern_us > 100000) {
                pattern_us = 100000;
                break;
            }
        }

        // generate toggles within the computed pattern duration
        for (size_t i = 0; i < step_pins.size(); ++i) {
            double rps = speeds[i];
            if (rps <= 0.0) continue;
            double steps_per_sec = rps * STEPS_PER_REV * MICROSTEP;
            if (steps_per_sec <= 0.0) continue;
            double half = 1e6 / (2.0 * steps_per_sec);
            if (half < 1.0) half = 1.0;
            double t = half;
            bool high = false;
            uint32_t mask = 1u << step_pins[i];
            while (t < static_cast<double>(pattern_us)) {
                if (high) {
                    events.push_back({static_cast<uint32_t>(t), 0, mask});
                } else {
                    events.push_back({static_cast<uint32_t>(t), mask, 0});
                }
                high = !high;
                t += half;
            }
        }

        if (events.empty()) {
            return;
        }

        std::sort(events.begin(), events.end(), [](const Event &a, const Event &b){
            return a.time < b.time;
        });

        std::vector<gpioPulse_t> pulses;
        uint32_t lastTime = 0;
        uint32_t onMask = 0;
        uint32_t offMask = 0;
        for (auto &e : events) {
            if (e.time != lastTime) {
                uint32_t delay = e.time - lastTime;
                pulses.push_back({onMask, offMask, delay});
                lastTime = e.time;
                onMask = 0;
                offMask = 0;
            }
            onMask |= e.on;
            offMask |= e.off;
        }
        // final stretch to fill pattern_us
        uint32_t remaining = (uint32_t)(pattern_us > lastTime ? (pattern_us - lastTime) : 0);
        if (remaining > 0) {
            pulses.push_back({onMask, offMask, remaining});
        }

        wave_clear(pi);
        wave_add_generic(pi, pulses.size(), pulses.data());
        wave_id = wave_create(pi);
        if (wave_id >= 0) {
            wave_send_using_mode(pi, wave_id, PI_WAVE_MODE_REPEAT);
        }
    }
};

// static member definitions
std::mutex WaveManager::mutex;
int WaveManager::pi = -1;
int WaveManager::wave_id = -1;
std::vector<uint8_t> WaveManager::step_pins;
std::vector<double> WaveManager::speeds;
std::vector<bool> WaveManager::dirs;
} // anonymous namespace


/**
 * @brief Controls a stepper motor via pigpiod waves in a background thread.
 *
 * Each instance registers its step pin with a global WaveManager that
 * composes a single pigpio waveform for all motors.  This avoids the
 * limitation that the pigpio daemon can only transmit one repeating wave at
 * a time – without the manager a second motor would overwrite the first.
 *
 * The class accepts **angular velocity** commands (rad/s) and converts them
 * into stepper pulses using wave chains so that DMA generates high-frequency
 * pulses without CPU intervention. The enable pin is shared between motors
 * and is driven low when any motor is active.
 */
class MotorClass {
public:
    MotorClass(uint8_t direction_pin,
               uint8_t step_pin,
               uint8_t enable_pin,
               bool invert_direction = false);

    ~MotorClass();

    /// Set the desired angular velocity in radians per second.
    void setSpeed(double target_ang_vel_rad_s);

    /// Enable or disable the motor driver (active low).
    void setEnabled(bool enabled);

    /// Returns true if the motor should currently be stepping.
    bool isActive() const {
        return std::abs(target_motor_rps_.load(std::memory_order_relaxed)) > 0.01;
    }

private:
    int pi_ = -1;  ///< pigpiod handle obtained at construction

private:
    void initializePins();
    void run();

    std::thread motor_thread_;
    std::atomic<bool> running_{true};
    std::atomic<double> target_motor_rps_{0.0};
    std::atomic<bool> enabled_{false};   ///< read-only in run thread

    uint8_t direction_pin_;
    uint8_t step_pin_;
    uint8_t enable_pin_;
    bool invert_direction_;

    size_t index_{0};  ///< index in WaveManager arrays
};

inline MotorClass::MotorClass(uint8_t direction_pin,
                              uint8_t step_pin,
                              uint8_t enable_pin,
                              bool invert_direction)
    : direction_pin_(direction_pin),
      step_pin_(step_pin),
      enable_pin_(enable_pin),
      invert_direction_(invert_direction)
{
    // obtain a handle to the pigpiod daemon (throws on failure)
    pi_ = getPiHandle();

    // register this motor's step pin so the shared waveform can be built
    index_ = WaveManager::registerMotor(step_pin);

    initializePins();
    motor_thread_ = std::thread(&MotorClass::run, this);
}

inline MotorClass::~MotorClass() {
    running_ = false;
    if (motor_thread_.joinable()) {
        motor_thread_.join();
    }
    // stop this motor in the shared waveform
    WaveManager::setSpeed(index_, 0.0, false);
    // ensure driver is disabled on destruction
    gpio_write(pi_, enable_pin_, PI_HIGH);  // disable (active LOW)
}

inline void MotorClass::initializePins() {

    set_mode(pi_, direction_pin_, PI_OUTPUT);
    set_mode(pi_, step_pin_,     PI_OUTPUT);
    set_mode(pi_, enable_pin_,   PI_OUTPUT);

    gpio_write(pi_, step_pin_,   PI_LOW);
    gpio_write(pi_, enable_pin_, PI_HIGH);  // initially disabled
}

inline void MotorClass::setEnabled(bool enabled) {
    enabled_.store(enabled, std::memory_order_relaxed);
    gpio_write(pi_, enable_pin_, enabled ? PI_LOW : PI_HIGH);
}

inline void MotorClass::setSpeed(double target_ang_vel_rad_s) {
    // convert angular velocity (rad/s) to motor revolutions per second
    // linear speed = omega * R, circumference = 2*pi*R, so motor_rps =
    // (omega * R * GEAR_RATIO) / (2*pi*R) = (omega * GEAR_RATIO) / (2*pi)
    double motor_rps = (target_ang_vel_rad_s * GEAR_RATIO) / (2.0 * M_PI);
    target_motor_rps_.store(motor_rps, std::memory_order_relaxed);
}

void MotorClass::run() {
    while (running_) {
        double target = target_motor_rps_.load(std::memory_order_relaxed);
        bool enabled = enabled_.load(std::memory_order_relaxed);

        if (!enabled || std::abs(target) < 0.01) {
            WaveManager::setSpeed(index_, 0.0, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        double motor_rps = std::clamp(std::abs(target), MIN_MOTOR_RPS, MAX_MOTOR_RPS);
        bool dir = (target > 0.0);
        if (invert_direction_) dir = !dir;

        gpio_write(pi_, direction_pin_, dir ? PI_HIGH : PI_LOW);
        WaveManager::setSpeed(index_, motor_rps, dir);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // ensure stopped when thread exits
    WaveManager::setSpeed(index_, 0.0, false);
}


#endif // MOTOR_HPP
