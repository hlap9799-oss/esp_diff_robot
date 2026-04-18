#ifndef _ESP32_MOTOR_H_
#define _ESP32_MOTOR_H_

#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/gpio_reg.h"

// ==== Motor Pin Mapping ====
#define MOTOR1_PIN_A 35
#define MOTOR1_PIN_B 36
#define MOTOR2_PIN_A 37
#define MOTOR2_PIN_B 38
#define MOTOR3_PIN_A 39
#define MOTOR3_PIN_B 40
#define MOTOR4_PIN_A 41
#define MOTOR4_PIN_B 42

// ==== MCPWM Mapping ====
#define MOTOR1_MCPWM_UNIT MCPWM_UNIT_0
#define MOTOR1_MCPWM_TIMER MCPWM_TIMER_0
#define MOTOR2_MCPWM_UNIT MCPWM_UNIT_0
#define MOTOR2_MCPWM_TIMER MCPWM_TIMER_1
#define MOTOR3_MCPWM_UNIT MCPWM_UNIT_0
#define MOTOR3_MCPWM_TIMER MCPWM_TIMER_2
#define MOTOR4_MCPWM_UNIT MCPWM_UNIT_1
#define MOTOR4_MCPWM_TIMER MCPWM_TIMER_0

// ==== Motor Class ====
class esp32_motor
{
private:
    struct EncoderConfig
    {
        int pinA = -1;
        int pinB = -1;
        uint32_t pulsesPerRevolution = 0;
        volatile long pulseCount = 0;
        long lastPulseCount = 0;
        float speedRps = 0.0f;
        uint32_t lastSpeedUpdateMs = 0;
        uint8_t state = 0;
        bool enabled = false;
    };

    struct PIDConfig
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
    };

    struct PIDState
    {
        float integral = 0.0f;
        float previousError = 0.0f;
        uint32_t previousUpdateMs = 0;
    };

    gpio_num_t pinA, pinB;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t sigA, sigB;
    uint32_t freq;
    EncoderConfig encoder;
    PIDConfig speedPid;
    PIDState speedPidState;
    bool reversed = false;
    bool initialized = false;

    static void IRAM_ATTR EncoderAISR(void* arg);
    static void IRAM_ATTR EncoderBISR(void* arg);
    uint8_t IRAM_ATTR readEncoderState() const;
    void IRAM_ATTR handleEncoderInterrupt();
    void updateEncoderSpeed();
public:
    esp32_motor(int pinA, int pinB,
                mcpwm_unit_t unit = MCPWM_UNIT_0,
                mcpwm_timer_t timer = MCPWM_TIMER_0,
                uint32_t freq = 1000);

    void init();
    void Run(int duty);   // -100 ~ +100 (%)
    void Stop();          // duty = 0%
    void Brake();         // duty = 100% both sides (short brake)
    void Reverse();       // toggle direction logic
    void SetSpeed(float targetSpeed, float currentSpeed = 0.0f);
    void SetEncoderPins(int encoderPinA, int encoderPinB);
    void SetEncoderPulsesPerRevolution(uint32_t pulsesPerRevolution = 0);
    void SetSpeedPID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
    bool UsingEncoder() const;
    uint32_t EncoderPulsesPerRevolution() const;
    long EncoderPulseCount() const;
    float EncoderSpeedRPS();
    void resetEncoder();
    void GetSpeedPID(float &kp, float &ki, float &kd) const;
};

// ==== Export global motors ====
extern esp32_motor Motor1;
extern esp32_motor Motor2;
extern esp32_motor Motor3;
extern esp32_motor Motor4;

#endif
