#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <cmath>
#include <cstdint>
#include <chrono>
#include <stdexcept>
#include <pigpiod_if2.h>

#include "kafeiche_drivers/pigpio_utils.hpp"  // shared helper for pigpio connection

// SPI configuration for magnetic encoders
constexpr int SPI_CHANNEL_LEFT   = 0;   ///< corresponds to physical CE1
constexpr int SPI_CHANNEL_RIGHT  = 1;   ///< corresponds to physical CE0
constexpr int SPI_SPEED          = 500000;

// pigpio SPI flags: mode 3 corresponds to CPOL|CPHA.
constexpr int SPI_FLAGS_MODE3 = 3;  ///< SPI mode 3 (CPOL|CPHA)
constexpr uint8_t REGx03     = 0x83;
constexpr uint8_t REGx04     = 0x84;
constexpr int MT6816_CPR     = 16384;


enum class EncoderChannel : int {
    LEFT  = SPI_CHANNEL_LEFT,
    RIGHT = SPI_CHANNEL_RIGHT
};

/**
 * @brief Structure holding both position and velocity from one measurement.
 */
struct EncoderMeasurement {
    float position;  ///< cumulative position (radians)
    float velocity;  ///< angular velocity (rad/s)
};

/**
 * @brief Simple SPI reader for MT6816 magnetic encoders.
 *
 * Provides angular delta and velocity measurements in radians and rad/s.
 */
class EncoderClass {
public:
    EncoderClass();
    ~EncoderClass();

    /**
     * @brief Reads the encoder and returns both cumulative position and velocity.
     * @param ch LEFT or RIGHT channel.
     * @return EncoderMeasurement containing position and velocity.
     *
     * The method performs a single SPI read, computes the angular delta since the
     * previous call (unwrapping through 2π), updates the cumulative position and
     * velocity, and returns both values. This ensures that position and velocity
     * are derived from the same Δt and Δangle.
     */
    EncoderMeasurement getMeasurement(EncoderChannel ch);

private:
    int pi_ = -1;  ///< pigpiod handle

    struct EncState {
        float prev_angle = 0.0f;
        float velocity   = 0.0f;
        float cumulative_position = 0.0f;
        std::chrono::steady_clock::time_point prev_time;
    };

    EncState left_;
    EncState right_;
    unsigned char buffer_[2];

    int spi_handle_left_;
    int spi_handle_right_;

    uint16_t read_raw_angle(EncoderChannel ch);
    float    read_angle(EncoderChannel ch);
    EncState& state(EncoderChannel ch);

    inline float direction(EncoderChannel ch) const {
        return (ch == EncoderChannel::LEFT) ? -1.0f : 1.0f;
    }
};

/* ======================= implementation ======================= */

EncoderClass::EncoderClass()
{
    // connect to pigpiod daemon and cache the handle
    pi_ = getPiHandle();

    // open two SPI channels (SPI0 CE0 and CE1) via the daemon
    spi_handle_right_ = spi_open(pi_, SPI_CHANNEL_RIGHT, SPI_SPEED, SPI_FLAGS_MODE3);
    spi_handle_left_  = spi_open(pi_, SPI_CHANNEL_LEFT,  SPI_SPEED, SPI_FLAGS_MODE3);

    if (spi_handle_right_ < 0 || spi_handle_left_ < 0) {
        int err = (spi_handle_right_ < 0) ? spi_handle_right_ : spi_handle_left_;
        throw std::runtime_error(
            std::string("Failed to open SPI channel(s) (pigpio error ") +
            std::to_string(err) + ") – is pigpiod running and initialized?"
        );
    }

    auto now = std::chrono::steady_clock::now();
    left_.prev_time  = now;
    right_.prev_time = now;

    // Initialise previous angles (first measurement will yield delta = 0)
    left_.prev_angle  = read_angle(EncoderChannel::LEFT);
    right_.prev_angle = read_angle(EncoderChannel::RIGHT);
}

EncoderClass::~EncoderClass()
{
    if (spi_handle_left_ >= 0) spi_close(pi_, spi_handle_left_);
    if (spi_handle_right_ >= 0) spi_close(pi_, spi_handle_right_);
}

EncoderClass::EncState& EncoderClass::state(EncoderChannel ch)
{
    return (ch == EncoderChannel::LEFT) ? left_ : right_;
}

uint16_t EncoderClass::read_raw_angle(EncoderChannel ch) {
    int handle = (ch == EncoderChannel::LEFT) ? spi_handle_left_ : spi_handle_right_;
    buffer_[0] = REGx03;
    buffer_[1] = 0x00;
    spi_xfer(pi_, handle, reinterpret_cast<char *>(buffer_),
            reinterpret_cast<char *>(buffer_), 2);
    uint16_t reg03 = buffer_[1];
    buffer_[0] = REGx04;
    buffer_[1] = 0x00;
    spi_xfer(pi_, handle, reinterpret_cast<char *>(buffer_),
            reinterpret_cast<char *>(buffer_), 2);
    uint16_t reg04 = buffer_[1];
    return (reg03 << 6) | (reg04 >> 2);
}

float EncoderClass::read_angle(EncoderChannel ch) {
    uint16_t raw = read_raw_angle(ch);
    return raw * (2.0f * M_PI / MT6816_CPR);
}

/* ======================= public API ======================= */

EncoderMeasurement EncoderClass::getMeasurement(EncoderChannel ch) {
    EncState& s = state(ch);
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - s.prev_time).count();

    float angle = read_angle(ch);
    float delta = angle - s.prev_angle;
    // unwrap through 2π
    if (delta > M_PI) delta -= 2.0f * M_PI;
    else if (delta < -M_PI) delta += 2.0f * M_PI;
    delta *= direction(ch);

    s.cumulative_position += delta;
    float velocity = 0.0f;
    if (dt > 0.0) {
        velocity = static_cast<float>(delta / dt);
    }
    s.velocity = velocity;
    s.prev_angle = angle;
    s.prev_time = now;

    return {s.cumulative_position, velocity};
}

#endif // ENCODER_HPP