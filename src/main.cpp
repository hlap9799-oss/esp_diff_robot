#include <main.h>

namespace {
constexpr uint32_t kSerialBaud = 921600;
constexpr uint32_t kSerialWaitTimeoutMs = 10000;
constexpr uint32_t kLogIntervalMs = 500;
constexpr float kDesiredLinearCmPerSec = 10.0f;
constexpr float kDesiredAngularRadPerSec = 0.0f;
constexpr float kPidKp = 2.0f;
constexpr float kPidKi = 0.3f;
constexpr float kPidKd = 0.05f;

differential_controller::DifferentialKinematics* kinematics = nullptr;

void configureMotor(esp32_motor& motor, bool reversed, int encoderPinA, int encoderPinB, uint32_t pulsesPerRevolution)
{
    if (reversed) {
        motor.Reverse();
    }

    motor.SetEncoderPins(encoderPinA, encoderPinB);
    motor.SetEncoderPulsesPerRevolution(pulsesPerRevolution);
    motor.Stop();
}
}

void setup()
{
    Serial.begin(kSerialBaud);
    delay(1500);

    const uint32_t serialWaitStart = millis();
    while (!Serial && millis() - serialWaitStart < kSerialWaitTimeoutMs) {
        delay(10);
    }

    Serial.println();
    Serial.println("Pose logger boot");

    pinMode(0, INPUT);
    attachInterrupt(digitalPinToInterrupt(0), [](){leftMotor.resetEncoder(); rightMotor.resetEncoder();kinematics->ResetPose(0.0f, 0.0f, 0.0f);}, RISING);

    configureMotor(leftMotor, leftMotorReversed, leftEncoderPinA, leftEncoderPinB, leftMotorPulsesPerRevolution);
    configureMotor(rightMotor, rightMotorReversed, rightEncoderPinA, rightEncoderPinB, rightMotorPulsesPerRevolution);

    static differential_controller::DifferentialKinematics localKinematics(leftMotor, rightMotor);
    kinematics = &localKinematics;
    kinematics->SetWheelConfig(
        leftWheelPerimeterCm,
        rightWheelPerimeterCm,
        wheelBaseCm,
        leftMotorPulsesPerRevolution,
        rightMotorPulsesPerRevolution);
    kinematics->SetSpeedPID(kPidKp, kPidKi, kPidKd);
    kinematics->ResetPose(0.0f, 0.0f, 0.0f);
    kinematics->SetKinematicTargets(kDesiredLinearCmPerSec, kDesiredAngularRadPerSec);
}

void loop()
{
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();

    if (kinematics != nullptr) {
        kinematics->Update();

        if (now - lastLogMs >= kLogIntervalMs) {
            lastLogMs = now;
            kinematics->PrintTelemetry();
        }
    }

    delay(10);
}
