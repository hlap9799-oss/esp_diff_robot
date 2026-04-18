#include <esp32_motor.hpp>

namespace {
constexpr int8_t kQuadratureDelta[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};
}

// ===== Class Implementation =====
esp32_motor::esp32_motor(int _pinA, int _pinB,
                         mcpwm_unit_t _unit, mcpwm_timer_t _timer,
                         uint32_t _freq)
{
    pinA = (gpio_num_t)_pinA;
    pinB = (gpio_num_t)_pinB;
    unit = _unit;
    timer = _timer;
    freq = _freq;

    sigA = (timer == MCPWM_TIMER_0) ? MCPWM0A :
           (timer == MCPWM_TIMER_1) ? MCPWM1A : MCPWM2A;
    sigB = (timer == MCPWM_TIMER_0) ? MCPWM0B :
           (timer == MCPWM_TIMER_1) ? MCPWM1B : MCPWM2B;
}

void IRAM_ATTR esp32_motor::EncoderAISR(void* arg)
{
    static_cast<esp32_motor*>(arg)->handleEncoderInterrupt();
}

void IRAM_ATTR esp32_motor::EncoderBISR(void* arg)
{
    static_cast<esp32_motor*>(arg)->handleEncoderInterrupt();
}

uint8_t IRAM_ATTR esp32_motor::readEncoderState() const
{
    const uint32_t gpioInA = (encoder.pinA < 32) ? REG_READ(GPIO_IN_REG) : REG_READ(GPIO_IN1_REG);
    const uint32_t gpioInB = (encoder.pinB < 32) ? REG_READ(GPIO_IN_REG) : REG_READ(GPIO_IN1_REG);
    const uint8_t a = (gpioInA >> ((encoder.pinA < 32) ? encoder.pinA : (encoder.pinA - 32))) & 0x01;
    const uint8_t b = (gpioInB >> ((encoder.pinB < 32) ? encoder.pinB : (encoder.pinB - 32))) & 0x01;
    return static_cast<uint8_t>((a << 1) | b);
}

void esp32_motor::init()
{
    mcpwm_gpio_init(unit, sigA, pinA);
    mcpwm_gpio_init(unit, sigB, pinB);

    mcpwm_config_t config;
    config.frequency = freq;
    config.cmpr_a = 0;
    config.cmpr_b = 0;
    config.counter_mode = MCPWM_UP_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(unit, timer, &config);
}

void IRAM_ATTR esp32_motor::handleEncoderInterrupt()
{
    if (!encoder.enabled) return;

    const uint8_t currentState = readEncoderState();
    const uint8_t transition = static_cast<uint8_t>((encoder.state << 2) | currentState);
    encoder.pulseCount += kQuadratureDelta[transition];
    encoder.state = currentState;
}

void esp32_motor::updateEncoderSpeed()
{
    if (!encoder.enabled || encoder.pulsesPerRevolution == 0) {
        encoder.speedRps = 0.0f;
        return;
    }

    const uint32_t now = millis();
    if (encoder.lastSpeedUpdateMs == 0) {
        encoder.lastSpeedUpdateMs = now;
        encoder.lastPulseCount = encoder.pulseCount;
        encoder.speedRps = 0.0f;
        return;
    }

    const uint32_t elapsedMs = now - encoder.lastSpeedUpdateMs;
    if (elapsedMs == 0) return;

    const long pulseDelta = encoder.pulseCount - encoder.lastPulseCount;
    encoder.speedRps = (pulseDelta * 1000.0f) / (encoder.pulsesPerRevolution * elapsedMs);
    encoder.lastPulseCount = encoder.pulseCount;
    encoder.lastSpeedUpdateMs = now;
}

void esp32_motor::Run(int duty)
{
    if (!initialized) init();
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;

    int dutyA = 0, dutyB = 0;

    if (duty > 0) {
        dutyA = duty;
        dutyB = 0;
    } else if (duty < 0) {
        dutyA = 0;
        dutyB = -duty;
    } else {
        Stop();
        return;
    }

    if (reversed) {
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, dutyB);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, dutyA);
    } else {
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, dutyA);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, dutyB);
    }
}

void esp32_motor::Stop()
{
    if (!initialized) init();
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);
}

void esp32_motor::Brake()
{
    if (!initialized) init();
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 100);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 100);
}

void esp32_motor::Reverse()
{
    reversed = !reversed;
}

void esp32_motor::SetSpeed(float targetSpeed, float currentSpeed)
{
    if (UsingEncoder()) {
        updateEncoderSpeed();
        currentSpeed = encoder.speedRps;
    }

    if (!UsingEncoder() ||
        (speedPid.kp == 0.0f && speedPid.ki == 0.0f && speedPid.kd == 0.0f)) {
        Run((int)targetSpeed);
        return;
    }

    const uint32_t now = millis();
    float dt = 0.01f;
    if (speedPidState.previousUpdateMs != 0 && now > speedPidState.previousUpdateMs) {
        dt = (now - speedPidState.previousUpdateMs) / 1000.0f;
    }

    const float error = targetSpeed - currentSpeed;
    speedPidState.integral += error * dt;
    const float derivative = (error - speedPidState.previousError) / dt;
    const float output =
        speedPid.kp * error +
        speedPid.ki * speedPidState.integral +
        speedPid.kd * derivative;

    speedPidState.previousError = error;
    speedPidState.previousUpdateMs = now;

    Run((int)constrain(output, -100.0f, 100.0f));
}

void esp32_motor::SetEncoderPins(int encoderPinA, int encoderPinB)
{
    encoder.pinA = encoderPinA;
    encoder.pinB = encoderPinB;
    encoder.pulseCount = 0;
    encoder.lastPulseCount = 0;
    encoder.speedRps = 0.0f;
    encoder.lastSpeedUpdateMs = 0;
    encoder.state = 0;

    pinMode(encoder.pinA, INPUT_PULLUP);
    pinMode(encoder.pinB, INPUT_PULLUP);
    encoder.state = readEncoderState();
    encoder.enabled = true;
    attachInterruptArg(digitalPinToInterrupt(encoder.pinA), EncoderAISR, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(encoder.pinB), EncoderBISR, this, CHANGE);
}

void esp32_motor::SetEncoderPulsesPerRevolution(uint32_t pulsesPerRevolution)
{
    encoder.pulsesPerRevolution = pulsesPerRevolution;
}

void esp32_motor::SetSpeedPID(float kp, float ki, float kd)
{
    speedPid.kp = kp;
    speedPid.ki = ki;
    speedPid.kd = kd;
}

bool esp32_motor::UsingEncoder() const
{
    return encoder.enabled;
}

uint32_t esp32_motor::EncoderPulsesPerRevolution() const
{
    return encoder.pulsesPerRevolution;
}

long esp32_motor::EncoderPulseCount() const
{
    return encoder.pulseCount;
}

float esp32_motor::EncoderSpeedRPS()
{
    updateEncoderSpeed();
    return encoder.speedRps;
}

void esp32_motor::resetEncoder()
{
    encoder.pulseCount = 0;
    encoder.lastPulseCount = 0;
    // encoder.speedRps = 0.0f;
    encoder.lastSpeedUpdateMs = 0;
}

void esp32_motor::GetSpeedPID(float &kp, float &ki, float &kd) const
{
    kp = speedPid.kp;
    ki = speedPid.ki;
    kd = speedPid.kd;
}

// ===== Global Motor Objects =====
esp32_motor Motor1(MOTOR1_PIN_A, MOTOR1_PIN_B, MOTOR1_MCPWM_UNIT, MOTOR1_MCPWM_TIMER);
esp32_motor Motor2(MOTOR2_PIN_A, MOTOR2_PIN_B, MOTOR2_MCPWM_UNIT, MOTOR2_MCPWM_TIMER);
esp32_motor Motor3(MOTOR3_PIN_A, MOTOR3_PIN_B, MOTOR3_MCPWM_UNIT, MOTOR3_MCPWM_TIMER);
esp32_motor Motor4(MOTOR4_PIN_A, MOTOR4_PIN_B, MOTOR4_MCPWM_UNIT, MOTOR4_MCPWM_TIMER);
