#include "kitronik_simply_robotics.hpp"
#include "pico/stdlib.h"
#include "servo_pwm.pio.h"   // generado por pico_generate_pio_header()
#include <algorithm>
#include "hardware/clocks.h"


// ---------------------- SimplePWMMotor ----------------------

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

SimplePWMMotor::SimplePWMMotor(uint forwardPin, uint reversePin, uint startFreq)
: fwdPin_(forwardPin), revPin_(reversePin), freq_(startFreq) {

    gpio_set_function(fwdPin_, GPIO_FUNC_PWM);
    gpio_set_function(revPin_, GPIO_FUNC_PWM);

    fwdSlice_ = pwm_gpio_to_slice_num(fwdPin_);
    revSlice_ = pwm_gpio_to_slice_num(revPin_);

    // Wrap 16-bit style (0..65535)
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, 65535);

    pwm_init(fwdSlice_, &cfg, true);
    pwm_init(revSlice_, &cfg, true);

    setFreq(freq_);
    off();
}

void SimplePWMMotor::setFreq(uint freq_hz) {
    // PWM freq = clk_sys / (div * (wrap+1))
    // Con wrap=65535, ajustamos div.
    // div = clk_sys / (freq * 65536)
    const float clk = (float)clock_get_hz(clk_sys);
    float div = clk / ((float)freq_hz * 65536.0f);
    if (div < 1.0f) div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(fwdSlice_, div);
    pwm_set_clkdiv(revSlice_, div);
    freq_ = freq_hz;
}

uint16_t SimplePWMMotor::speedToLevel(int speed_percent) {
    speed_percent = clampi(speed_percent, 0, 100);
    // 0..100 => 0..65535
    return (uint16_t)(speed_percent * 655.35f);
}

void SimplePWMMotor::on(char direction, int speed_percent) {
    speed_percent = clampi(speed_percent, 0, 100);

    // “adaptive frequency” como tu MicroPython
    uint freq = 100;
    if (speed_percent < 15) freq = 20;
    else if (speed_percent < 20) freq = 50;
    setFreq(freq);

    uint16_t level = speedToLevel(speed_percent);

    if (direction == 'f') {
        pwm_set_gpio_level(fwdPin_, level);
        pwm_set_gpio_level(revPin_, 0);
    } else if (direction == 'r') {
        pwm_set_gpio_level(fwdPin_, 0);
        pwm_set_gpio_level(revPin_, level);
    } else if (direction == '-') {
        pwm_set_gpio_level(fwdPin_, 0);
        pwm_set_gpio_level(revPin_, 0);
    } else {
        // sin exceptions: si llega basura, apaga
        off();
    }
}

void SimplePWMMotor::off() { on('-', 0); }

// ---------------------- StepperMotor ----------------------

StepperMotor::StepperMotor(SimplePWMMotor* coilA, SimplePWMMotor* coilB) {
    coils_[0] = coilA;
    coils_[1] = coilB;
    state_ = 0;
}

static inline char seq4(int state, int coil) {
    // stepSequence = [["f","-"], ["-","r"], ["r","-"], ["-","f"]]
    static const char s[4][2] = {
        {'f','-'},
        {'-','r'},
        {'r','-'},
        {'-','f'}
    };
    return s[state][coil];
}

static inline char seq8(int state, int coil) {
    // halfStepSequence = [["f","-"],["f","r"],["-","r"],["r","r"],["r","-"],["r","f"],["-","f"],["f","f"]]
    static const char s[8][2] = {
        {'f','-'},
        {'f','r'},
        {'-','r'},
        {'r','r'},
        {'r','-'},
        {'r','f'},
        {'-','f'},
        {'f','f'}
    };
    return s[state][coil];
}

void StepperMotor::step(char direction) {
    if (!coils_[0] || !coils_[1]) return;

    if (direction == 'f') state_++;
    else if (direction == 'r') state_--;
    else return;

    if (state_ > 3) state_ = 0;
    if (state_ < 0) state_ = 3;

    for (int i = 0; i < 2; i++) coils_[i]->on(seq4(state_, i), 100);
}

void StepperMotor::halfStep(char direction) {
    if (!coils_[0] || !coils_[1]) return;

    if (direction == 'f') state_++;
    else if (direction == 'r') state_--;
    else return;

    if (state_ > 7) state_ = 0;
    if (state_ < 0) state_ = 7;

    for (int i = 0; i < 2; i++) coils_[i]->on(seq8(state_, i), 100);
}

// ---------------------- PIOServo ----------------------

// Alloc simple de 8 SM: pio0(0..3) y pio1(0..3)
static bool g_used_sm_pio0[4] = {false,false,false,false};
static bool g_used_sm_pio1[4] = {false,false,false,false};

static bool claim_sm(PIO& pio_out, int& sm_out) {
    for (int sm = 0; sm < 4; sm++) {
        if (!g_used_sm_pio0[sm]) { g_used_sm_pio0[sm] = true; pio_out = pio0; sm_out = sm; return true; }
    }
    for (int sm = 0; sm < 4; sm++) {
        if (!g_used_sm_pio1[sm]) { g_used_sm_pio1[sm] = true; pio_out = pio1; sm_out = sm; return true; }
    }
    return false;
}

PIOServo::PIOServo(uint servoPin) { initSM_(servoPin); }

void PIOServo::initSM_(uint servoPin) {
    if (!claim_sm(pio_, sm_)) {
        // sin exceptions: no hay SM disponible -> deja servo “muerto”
        sm_ = -1; pio_ = nullptr; active_ = false;
        return;
    }

    // Cargar programa
    offset_ = pio_add_program(pio_, &servo_pwm_program);

    pio_sm_config c = servo_pwm_program_get_default_config(offset_);

    // pin de salida por sideset
    sm_config_set_sideset_pins(&c, servoPin);
    pio_gpio_init(pio_, servoPin);
    pio_sm_set_consecutive_pindirs(pio_, sm_, servoPin, 1, true);

    // Frecuencia: 2 MHz como MicroPython
    // clkdiv = clk_sys / 2,000,000
    float div = (float)clock_get_hz(clk_sys) / 2000000.0f;
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio_, sm_, offset_, &c);

    // Preload pulseTrain en ISR: 20000us
    // En micropython: put(pulseTrain), exec("pull()"), exec("mov(isr, osr)")
    pio_sm_put_blocking(pio_, sm_, kPulseTrainUs);
    pio_sm_exec(pio_, sm_, pio_encode_pull(false, false));
    pio_sm_exec(pio_, sm_, pio_encode_mov(pio_isr, pio_osr));

    // default off
    deregisterServo();
}

bool PIOServo::isActive() const { return active_; }

void PIOServo::registerServo() {
    if (sm_ < 0) return;
    pio_sm_set_enabled(pio_, sm_, true);
    active_ = true;
}

void PIOServo::deregisterServo() {
    if (sm_ < 0) return;
    pio_sm_set_enabled(pio_, sm_, false);
    active_ = false;
}

void PIOServo::setPulseRangeUs(int min_us, int max_us) {
    // clamp básico y coherencia
    min_us = clampi(min_us, kMinPulseUs, kMaxPulseUs);
    max_us = clampi(max_us, kMinPulseUs, kMaxPulseUs);
    if (max_us < min_us) std::swap(max_us, min_us);

    minPulseUs_ = min_us;
    maxPulseUs_ = max_us;
}

void PIOServo::setPulseOffsetUs(int offset_us) {
    // trim razonable, por si quieres afinar sin romper
    offsetUs_ = clampi(offset_us, -500, 500);
}

void PIOServo::goToPosition(int degrees) {
    degrees = clampi(degrees, 0, 180);

    const float t = (float)degrees / 180.0f;
    int pulse = (int)((1.0f - t) * (float)minPulseUs_ + t * (float)maxPulseUs_);

    pulse += offsetUs_;
    goToPeriod(pulse);
}

void PIOServo::goToRadians(float radians) {
    if (radians < 0.0f) radians = 0.0f;
    if (radians > kPi)  radians = kPi;

    const float t = radians / kPi;
    int pulse = (int)((1.0f - t) * (float)minPulseUs_ + t * (float)maxPulseUs_);

    pulse += offsetUs_;
    goToPeriod(pulse);
}

void PIOServo::goToPeriod(int period_us) {
    // ahora clamp por instancia (y con offset ya aplicado)
    period_us = clampi(period_us, minPulseUs_, maxPulseUs_);
    if (!active_) return;
    pio_sm_put_blocking(pio_, sm_, (uint32_t)period_us);
}

// ---------------------- KitronikSimplyRobotics ----------------------

KitronikSimplyRobotics::KitronikSimplyRobotics(bool centreServos) {
    // Motores: (GP2,GP5) (GP4,GP3) (GP6,GP9) (GP8,GP7)
    motors[0] = SimplePWMMotor(2, 5, 100);
    motors[1] = SimplePWMMotor(4, 3, 100);
    motors[2] = SimplePWMMotor(6, 9, 100);
    motors[3] = SimplePWMMotor(8, 7, 100);

    steppers[0] = StepperMotor(&motors[0], &motors[1]);
    steppers[1] = StepperMotor(&motors[2], &motors[3]);

    // Servos: 15,14,13,12,19,18,17,16 (servo 0..7)
    servos[0] = PIOServo(15);
    servos[1] = PIOServo(14);
    servos[2] = PIOServo(13);
    servos[3] = PIOServo(12);
    servos[4] = PIOServo(19);
    servos[5] = PIOServo(18);
    servos[6] = PIOServo(17);
    servos[7] = PIOServo(16);

    for (int i = 0; i < 8; i++) {
        servos[i].registerServo();
        if (centreServos) servos[i].goToPosition(90);
    }
}
