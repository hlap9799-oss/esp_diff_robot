#include <kinematics.hpp>
#include <cmath>

namespace differential_controller {

DifferentialKinematics::DifferentialKinematics(esp32_motor& leftMotorRef,
                                             esp32_motor& rightMotorRef)
    : left(leftMotorRef),
      right(rightMotorRef),
      pose(),
      state(),
      lastLeftCount(left.EncoderPulseCount()),
      lastRightCount(right.EncoderPulseCount()),
      lastUpdateMs(millis())
{
}

void DifferentialKinematics::SetWheelConfig(float leftWheelPerimeter,
                                            float rightWheelPerimeter,
                                            float baseCm,
                                            float leftPulsesPerRevolution,
                                            float rightPulsesPerRevolution)
{
    leftWheelPerimeterCm = leftWheelPerimeter;
    rightWheelPerimeterCm = rightWheelPerimeter;
    wheelBaseCm = baseCm;
    leftCmPerPulse = (leftPulsesPerRevolution != 0.0f) ? leftWheelPerimeterCm / leftPulsesPerRevolution : 0.0f;
    rightCmPerPulse = (rightPulsesPerRevolution != 0.0f) ? rightWheelPerimeterCm / rightPulsesPerRevolution : 0.0f;
}

void DifferentialKinematics::SetSpeedPID(float kp, float ki, float kd)
{
    left.SetSpeedPID(kp, ki, kd);
    right.SetSpeedPID(kp, ki, kd);
}

void DifferentialKinematics::SetWheelTargetsRps(float leftRps, float rightRps)
{
    state.leftTargetRps = leftRps;
    state.rightTargetRps = rightRps;
    state.leftTargetCmPerSec = leftRps * leftWheelPerimeterCm;
    state.rightTargetCmPerSec = rightRps * rightWheelPerimeterCm;
}

void DifferentialKinematics::SetWheelTargetsCmPerSec(float leftCmPerSec, float rightCmPerSec)
{
    state.leftTargetCmPerSec = leftCmPerSec;
    state.rightTargetCmPerSec = rightCmPerSec;
    state.leftTargetRps = (leftWheelPerimeterCm != 0.0f) ? leftCmPerSec / leftWheelPerimeterCm : 0.0f;
    state.rightTargetRps = (rightWheelPerimeterCm != 0.0f) ? rightCmPerSec / rightWheelPerimeterCm : 0.0f;
}

void DifferentialKinematics::SetKinematicTargets(float linearCmPerSec, float angularRadPerSec)
{
    state.linearCmPerSec = linearCmPerSec;
    state.angularRadPerSec = angularRadPerSec;

    const float halfBase = wheelBaseCm * 0.5f;
    const float leftCm = linearCmPerSec - angularRadPerSec * halfBase;
    const float rightCm = linearCmPerSec + angularRadPerSec * halfBase;
    SetWheelTargetsCmPerSec(leftCm, rightCm);
}

void DifferentialKinematics::ResetPose(float initialX, float initialY, float initialHeadingRad)
{
    pose.x = initialX;
    pose.y = initialY;
    pose.headingRad = NormalizeAngle(initialHeadingRad);
    lastLeftCount = left.EncoderPulseCount();
    lastRightCount = right.EncoderPulseCount();
    lastUpdateMs = millis();
    state = KinematicState();
}

void DifferentialKinematics::Update()
{
    computeDerivedState();
    applyMotorTargets();
    computePoseFromEncoders();
    lastUpdateMs = millis();
}

void DifferentialKinematics::UpdatePose()
{
    computePoseFromEncoders();
}

void DifferentialKinematics::PrintPose(Stream& output) const
{
    output.print("pose x(cm)=");
    output.print(pose.x, 3);
    output.print(" y(cm)=");
    output.print(pose.y, 3);
    output.print(" heading(rad)=");
    output.print(pose.headingRad, 4);
    output.print(" heading(deg)=");
    output.println(pose.headingRad * 180.0f / PI, 2);
}

void DifferentialKinematics::PrintTelemetry(Stream& output) const
{
    output.print("TL,");
    output.print(pose.x, 3);
    output.print(',');
    output.print(pose.y, 3);
    output.print(',');
    output.print(pose.headingRad, 4);
    output.print(',');
    output.print(state.linearCmPerSec, 3);
    output.print(',');
    output.print(state.angularRadPerSec, 4);
    output.print(',');
    output.print(state.leftMeasuredCmPerSec, 3);
    output.print(',');
    output.print(state.rightMeasuredCmPerSec, 3);
    output.print(',');
    output.print(state.leftTargetRps, 3);
    output.print(',');
    output.print(state.rightTargetRps, 3);
    output.print(',');
    output.print(state.leftMeasuredRps, 3);
    output.print(',');
    output.print(state.rightMeasuredRps, 3);
    output.print(',');
    output.print(state.deltaLeftCm, 3);
    output.print(',');
    output.print(state.deltaRightCm, 3);
    output.println();
}

void DifferentialKinematics::PrintState(Stream& output) const
{
    output.print("target L rps=");
    output.print(state.leftTargetRps, 3);
    output.print(" target R rps=");
    output.print(state.rightTargetRps, 3);
    output.print(" measured L rps=");
    output.print(state.leftMeasuredRps, 3);
    output.print(" measured R rps=");
    output.println(state.rightMeasuredRps, 3);

    output.print("target L cm/s=");
    output.print(state.leftTargetCmPerSec, 2);
    output.print(" target R cm/s=");
    output.print(state.rightTargetCmPerSec, 2);
    output.print(" measured L cm/s=");
    output.print(state.leftMeasuredCmPerSec, 2);
    output.print(" measured R cm/s=");
    output.println(state.rightMeasuredCmPerSec, 2);

    output.print("linear cm/s=");
    output.print(state.linearCmPerSec, 2);
    output.print(" angular rad/s=");
    output.println(state.angularRadPerSec, 4);

    output.print("delta L(cm)=");
    output.print(state.deltaLeftCm, 3);
    output.print(" delta R(cm)=");
    output.print(state.deltaRightCm, 3);
    output.print(" delta heading(rad)=");
    output.println(state.deltaHeadingRad, 4);
}

void DifferentialKinematics::ReadPose(float& outX, float& outY, float& outHeadingRad) const
{
    outX = pose.x;
    outY = pose.y;
    outHeadingRad = pose.headingRad;
}

const Pose& DifferentialKinematics::GetPose() const
{
    return pose;
}

const KinematicState& DifferentialKinematics::GetState() const
{
    return state;
}

float DifferentialKinematics::NormalizeAngle(float angle)
{
    while (angle <= -PI) angle += 2.0f * PI;
    while (angle > PI) angle -= 2.0f * PI;
    return angle;
}

void DifferentialKinematics::computeDerivedState()
{
    state.leftMeasuredRps = left.EncoderSpeedRPS();
    state.rightMeasuredRps = right.EncoderSpeedRPS();
    state.leftMeasuredCmPerSec = state.leftMeasuredRps * leftWheelPerimeterCm;
    state.rightMeasuredCmPerSec = state.rightMeasuredRps * rightWheelPerimeterCm;
    state.linearCmPerSec = (state.leftMeasuredCmPerSec + state.rightMeasuredCmPerSec) * 0.5f;
    state.angularRadPerSec = (wheelBaseCm != 0.0f) ?
        (state.rightMeasuredCmPerSec - state.leftMeasuredCmPerSec) / wheelBaseCm : 0.0f;
}

void DifferentialKinematics::applyMotorTargets()
{
    left.SetSpeed(state.leftTargetRps, state.leftMeasuredRps);
    right.SetSpeed(state.rightTargetRps, state.rightMeasuredRps);
}

void DifferentialKinematics::computePoseFromEncoders()
{
    const long currentLeftCount = left.EncoderPulseCount();
    const long currentRightCount = right.EncoderPulseCount();

    const long deltaLeftCount = currentLeftCount - lastLeftCount;
    const long deltaRightCount = currentRightCount - lastRightCount;

    const float deltaLeftCm = deltaLeftCount * leftCmPerPulse;
    const float deltaRightCm = deltaRightCount * rightCmPerPulse;

    state.deltaLeftCm = deltaLeftCm;
    state.deltaRightCm = deltaRightCm;
    state.deltaDistanceCm = 0.5f * (deltaLeftCm + deltaRightCm);
    state.deltaHeadingRad = (wheelBaseCm != 0.0f) ?
        (deltaRightCm - deltaLeftCm) / wheelBaseCm : 0.0f;

    const float headingMid = pose.headingRad + state.deltaHeadingRad * 0.5f;
    pose.x += state.deltaDistanceCm * cos(headingMid);
    pose.y += state.deltaDistanceCm * sin(headingMid);
    pose.headingRad = NormalizeAngle(pose.headingRad + state.deltaHeadingRad);

    lastLeftCount = currentLeftCount;
    lastRightCount = currentRightCount;
}

} // namespace differential_controller
