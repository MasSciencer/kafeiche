#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <cmath>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cstdint>
#include <chrono>

#define CHANNEL0 0
#define CHANNEL1 1
#define SPI_SPEED 500000
#define REGx03 0x83
#define REGx04 0x84
#define MT6816_CPR 16384

enum class EncoderChannel : int {
    LEFT  = CHANNEL1,
    RIGHT = CHANNEL0
};

class EncoderClass {
public:
    EncoderClass();

    // Публичные методы
    float getDelta(EncoderChannel ch);       // дельта угла (рад)
    float getVelocity(EncoderChannel ch);    // рад/с

private:
    struct EncState {
        float prev_angle = 0.0f;
        float velocity   = 0.0f;
        std::chrono::steady_clock::time_point prev_time;
    };

    EncState left_;
    EncState right_;
    unsigned char buffer_[2];

    uint16_t read_raw_angle(int spi_ch);
    float read_angle(EncoderChannel ch);
    EncState& state(EncoderChannel ch);
};

EncoderClass::EncoderClass() {
    wiringPiSPISetupMode(CHANNEL0, SPI_SPEED, 3);
    wiringPiSPISetupMode(CHANNEL1, SPI_SPEED, 3);

    auto now = std::chrono::steady_clock::now();
    left_.prev_time  = now;
    right_.prev_time = now;

    left_.prev_angle  = read_angle(EncoderChannel::LEFT);
    right_.prev_angle = read_angle(EncoderChannel::RIGHT);
}

// --- private helpers ---
EncoderClass::EncState& EncoderClass::state(EncoderChannel ch) {
    return (ch == EncoderChannel::LEFT) ? left_ : right_;
}

uint16_t EncoderClass::read_raw_angle(int spi_ch) {
    buffer_[0] = REGx03;
    buffer_[1] = 0x00;
    wiringPiSPIDataRW(spi_ch, buffer_, 2);
    uint16_t reg03 = buffer_[1];

    buffer_[0] = REGx04;
    buffer_[1] = 0x00;
    wiringPiSPIDataRW(spi_ch, buffer_, 2);
    uint16_t reg04 = buffer_[1];

    return (reg03 << 6) | (reg04 >> 2);
}

float EncoderClass::read_angle(EncoderChannel ch) {
    uint16_t raw = read_raw_angle((int)ch);
    return raw * (2.0f * M_PI / MT6816_CPR);
}

// --- public API ---
float EncoderClass::getDelta(EncoderChannel ch) {
    EncState& s = state(ch);

    float angle = read_angle(ch);
    float delta = angle - s.prev_angle;

    // unwrapping
    if (delta > M_PI)       delta -= 2 * M_PI;
    else if (delta < -M_PI) delta += 2 * M_PI;

    s.prev_angle = angle;
    return delta;
}

float EncoderClass::getVelocity(EncoderChannel ch) {
    EncState& s = state(ch);
    
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - s.prev_time).count();

    if (dt <= 0.0) return s.velocity;

    float delta = getDelta(ch);
    s.velocity = delta / dt;
    s.prev_time = now;

    return s.velocity;
}

#endif
