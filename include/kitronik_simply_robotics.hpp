#pragma once
#include <cstdint>
#include "hardware/pwm.h"
#include "hardware/pio.h"

class SimplePWMMotor {
public:
    SimplePWMMotor() = default;
    SimplePWMMotor(uint forwardPin, uint reversePin, uint startFreq = 100);

    void on(char direction, int speed_percent); // 'f', 'r', '-'
    void off();

private:
    void setFreq(uint freq_hz);
    static uint16_t speedToLevel(int speed_percent);

    uint fwdPin_ = 0, revPin_ = 0;
    uint fwdSlice_ = 0, revSlice_ = 0;
    uint freq_ = 100;
};

class StepperMotor {
public:
    StepperMotor() = default;
    StepperMotor(SimplePWMMotor* coilA, SimplePWMMotor* coilB);

    void step(char direction = 'f');
    void halfStep(char direction = 'f');

private:
    SimplePWMMotor* coils_[2] = {nullptr, nullptr};
    int state_ = 0;
};

class PIOServo {
public:
    PIOServo() = default;
    PIOServo(uint servoPin);

    void registerServo();
    void deregisterServo();

    void goToPosition(int degrees);   // 0..180
    void goToRadians(float radians);  // 0..pi
    void goToPeriod(int period_us);   // 500..2500

    bool isActive() const;

private:
    void initSM_(uint servoPin);

    PIO pio_ = nullptr;
    int sm_ = -1;
    uint offset_ = 0;
    bool active_ = false;

    static constexpr int kPulseTrainUs = 20000;
    static constexpr int kMinPulseUs   = 500;
    static constexpr int kMaxPulseUs   = 2500;
    static constexpr float kPi = 3.1416f;
};

class KitronikSimplyRobotics {
public:
    explicit KitronikSimplyRobotics(bool centreServos = true);

    SimplePWMMotor motors[4];
    StepperMotor   steppers[2];
    PIOServo       servos[8];
};
