#ifndef DIFFERENTIAL_KINEMATICS_HPP
#define DIFFERENTIAL_KINEMATICS_HPP

#include <Arduino.h>
#include <esp32_motor.hpp>

namespace differential_controller {

struct Pose {
    float x = 0.0f;
    float y = 0.0f;
    float headingRad = 0.0f;
};

struct KinematicState {
    float leftTargetRps = 0.0f;
    float rightTargetRps = 0.0f;
    float leftMeasuredRps = 0.0f;
    float rightMeasuredRps = 0.0f;
    float leftTargetCmPerSec = 0.0f;
    float rightTargetCmPerSec = 0.0f;
    float leftMeasuredCmPerSec = 0.0f;
    float rightMeasuredCmPerSec = 0.0f;
    float linearCmPerSec = 0.0f;
    float angularRadPerSec = 0.0f;
    float deltaLeftCm = 0.0f;
    float deltaRightCm = 0.0f;
    float deltaDistanceCm = 0.0f;
    float deltaHeadingRad = 0.0f;
};

class DifferentialKinematics {
public:
    DifferentialKinematics(esp32_motor& leftMotorRef,
                            esp32_motor& rightMotorRef);

    void SetWheelConfig(float leftWheelPerimeterCm,
                        float rightWheelPerimeterCm,
                        float wheelBaseCm,
                        float leftPulsesPerRevolution,
                        float rightPulsesPerRevolution);
    void SetSpeedPID(float kp, float ki, float kd);
    void SetWheelTargetsRps(float leftRps, float rightRps);
    void SetWheelTargetsCmPerSec(float leftCmPerSec, float rightCmPerSec);
    void SetKinematicTargets(float linearCmPerSec, float angularRadPerSec);
    void ResetPose(float initialX = 0.0f, float initialY = 0.0f,
                   float initialHeadingRad = 0.0f);
    void Update();
    void UpdatePose();
    void PrintPose(Stream& output = Serial) const;
    void PrintState(Stream& output = Serial) const;
    void PrintTelemetry(Stream& output = Serial) const;
    void ReadPose(float& outX, float& outY, float& outHeadingRad) const;

    const Pose& GetPose() const;
    const KinematicState& GetState() const;

private:
    esp32_motor& left;
    esp32_motor& right;
    Pose pose;
    KinematicState state;
    long lastLeftCount;
    long lastRightCount;
    uint32_t lastUpdateMs;

    float leftWheelPerimeterCm = 0.0f;
    float rightWheelPerimeterCm = 0.0f;
    float wheelBaseCm = 0.0f;
    float leftCmPerPulse = 0.0f;
    float rightCmPerPulse = 0.0f;

    static float NormalizeAngle(float angle);
    void computePoseFromEncoders();
    void computeDerivedState();
    void applyMotorTargets();
};

} // namespace differential_controller

#endif // DIFFERENTIAL_KINEMATICS_HPP
