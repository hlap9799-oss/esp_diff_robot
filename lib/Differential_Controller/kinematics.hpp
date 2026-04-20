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
    float leftMotorPwmPercent = 0.0f;
    float rightMotorPwmPercent = 0.0f;
};

// Heading PID State
struct HeadingPIDState {
    float kp = 0.5f;
    float ki = 0.1f;
    float kd = 0.1f;
    float integral = 0.0f;
    float previousError = 0.0f;
    uint32_t lastUpdateMs = 0;
};

// Position PID State (keep robot on straight line)
struct PositionPIDState {
    float kp = 0.5f;
    float ki = 0.05f;
    float kd = 0.1f;
    float integral = 0.0f;
    float previousError = 0.0f;
    uint32_t lastUpdateMs = 0;
};

// Movement PIDF State (with feedforward)
struct MovementPIDFState {
    float kp = 2.0f;
    float ki = 0.3f;
    float kd = 0.05f;
    float kf = 0.1f;  // Feedforward term (velocity compensation)
    float integral = 0.0f;
    float previousError = 0.0f;
    uint32_t lastUpdateMs = 0;
};

// Movement state
enum class MovementState {
    IDLE,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    HOLDING_POSITION
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
    void SetHeadingPID(float kp, float ki, float kd);
    void SetPositionPID(float kp, float ki, float kd);
    void SetMovementPIDF(float kp, float ki, float kd, float kf);
    void SetWheelTargetsRps(float leftRps, float rightRps);
    void SetWheelTargetsCmPerSec(float leftCmPerSec, float rightCmPerSec);
    void SetKinematicTargets(float linearCmPerSec, float angularRadPerSec);
    void ResetPose(float initialX = 0.0f, float initialY = 0.0f,
                   float initialHeadingRad = 0.0f);
    
    // Movement control with heading correction
    void MoveForward(float distanceCm, float targetHeadingRad = 0.0f, float speedCmPerSec = 10.0f);
    void MoveBackward(float distanceCm, float targetHeadingRad = 0.0f, float speedCmPerSec = 10.0f);
    void HoldPosition(float targetHeadingRad = 0.0f);
    void Stop();
    
    MovementState GetMovementState() const { return movementState; }
    bool IsMovementComplete() const;
    
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

    // Heading control
    HeadingPIDState headingPID;
    float targetHeadingRad = 0.0f;
    
    // Position control (keep robot on straight line)
    PositionPIDState positionPID;
    float targetPositionY = 0.0f;
    
    // Movement PIDF control
    MovementPIDFState movementPIDF;
    
    // Movement tracking
    MovementState movementState = MovementState::IDLE;
    float movementStartX = 0.0f;
    float movementStartY = 0.0f;
    float movementStartDistance = 0.0f;
    float targetDistance = 0.0f;
    float targetLinearSpeed = 0.0f;
    int movementDirection = 0; // 1 for forward, -1 for backward

    static float NormalizeAngle(float angle);
    void computePoseFromEncoders();
    void computeDerivedState();
    void applyMotorTargets();
    float computeHeadingCorrection();
    float computePositionCorrection();
    float computeMovementCorrection();
    float getTotalDistance() const;
};

} // namespace differential_controller

#endif // DIFFERENTIAL_KINEMATICS_HPP
