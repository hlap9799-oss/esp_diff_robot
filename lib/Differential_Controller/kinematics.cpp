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

void DifferentialKinematics::SetHeadingPID(float kp, float ki, float kd)
{
    headingPID.kp = kp;
    headingPID.ki = ki;
    headingPID.kd = kd;
    headingPID.integral = 0.0f;
    headingPID.previousError = 0.0f;
}

void DifferentialKinematics::SetPositionPID(float kp, float ki, float kd)
{
    positionPID.kp = kp;
    positionPID.ki = ki;
    positionPID.kd = kd;
    positionPID.integral = 0.0f;
    positionPID.previousError = 0.0f;
}

void DifferentialKinematics::SetMovementPIDF(float kp, float ki, float kd, float kf)
{
    movementPIDF.kp = kp;
    movementPIDF.ki = ki;
    movementPIDF.kd = kd;
    movementPIDF.kf = kf;
    movementPIDF.integral = 0.0f;
    movementPIDF.previousError = 0.0f;
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
    headingPID.integral = 0.0f;
    headingPID.previousError = 0.0f;
}

void DifferentialKinematics::MoveForward(float distanceCm, float targetHeadingRad, float speedCmPerSec)
{
    movementState = MovementState::MOVING_FORWARD;
    movementStartX = pose.x;
    movementStartY = pose.y;
    targetDistance = distanceCm;
    targetLinearSpeed = speedCmPerSec;
    targetHeadingRad = NormalizeAngle(targetHeadingRad);
    movementDirection = 1;
    headingPID.integral = 0.0f;
    headingPID.previousError = 0.0f;
}

void DifferentialKinematics::MoveBackward(float distanceCm, float targetHeadingRad, float speedCmPerSec)
{
    movementState = MovementState::MOVING_BACKWARD;
    movementStartX = pose.x;
    movementStartY = pose.y;
    targetDistance = distanceCm;
    targetLinearSpeed = speedCmPerSec;
    targetHeadingRad = NormalizeAngle(targetHeadingRad);
    movementDirection = -1;
    headingPID.integral = 0.0f;
    headingPID.previousError = 0.0f;
}

void DifferentialKinematics::HoldPosition(float targetHeadingRad)
{
    movementState = MovementState::HOLDING_POSITION;
    targetLinearSpeed = 0.0f;
    targetHeadingRad = NormalizeAngle(targetHeadingRad);
    headingPID.integral = 0.0f;
    headingPID.previousError = 0.0f;
}

void DifferentialKinematics::Stop()
{
    movementState = MovementState::IDLE;
    SetKinematicTargets(0.0f, 0.0f);
    headingPID.integral = 0.0f;
    headingPID.previousError = 0.0f;
}

bool DifferentialKinematics::IsMovementComplete() const
{
    if (movementState == MovementState::IDLE || movementState == MovementState::HOLDING_POSITION) {
        return true;
    }
    // Calculate Euclidean distance traveled (handles rotation)
    float deltaX = pose.x - movementStartX;
    float deltaY = pose.y - movementStartY;
    float distanceTraveled = sqrt(deltaX * deltaX + deltaY * deltaY);
    return distanceTraveled >= targetDistance;
}

float DifferentialKinematics::getTotalDistance() const
{
    return (state.deltaLeftCm + state.deltaRightCm) * 0.5f;
}

float DifferentialKinematics::computeHeadingCorrection()
{
    float headingError = NormalizeAngle(targetHeadingRad - pose.headingRad);
    uint32_t now = millis();
    uint32_t dt = (headingPID.lastUpdateMs != 0) ? (now - headingPID.lastUpdateMs) : 0;
    headingPID.lastUpdateMs = now;

    if (dt > 0) {
        float dtSec = dt / 1000.0f;
        headingPID.integral += headingError * dtSec;
        
        // Limit integral term
        float maxIntegral = 1.0f;
        if (headingPID.integral > maxIntegral) headingPID.integral = maxIntegral;
        if (headingPID.integral < -maxIntegral) headingPID.integral = -maxIntegral;
        
        float derivative = (dt > 0) ? (headingError - headingPID.previousError) / dtSec : 0.0f;
        float correction = headingPID.kp * headingError + 
                          headingPID.ki * headingPID.integral + 
                          headingPID.kd * derivative;
        headingPID.previousError = headingError;
        
        return correction;
    }
    return 0.0f;
}

float DifferentialKinematics::computePositionCorrection()
{
    // Position error in Y (lateral deviation from straight line)
    // We want Y to stay near targetPositionY (usually 0)
    float positionError = targetPositionY - pose.y;
    
    uint32_t now = millis();
    uint32_t dt = (positionPID.lastUpdateMs != 0) ? (now - positionPID.lastUpdateMs) : 0;
    positionPID.lastUpdateMs = now;

    if (dt > 0) {
        float dtSec = dt / 1000.0f;
        positionPID.integral += positionError * dtSec;
        
        // Limit integral term
        float maxIntegral = 1.0f;
        if (positionPID.integral > maxIntegral) positionPID.integral = maxIntegral;
        if (positionPID.integral < -maxIntegral) positionPID.integral = -maxIntegral;
        
        float derivative = (dt > 0) ? (positionError - positionPID.previousError) / dtSec : 0.0f;
        float correction = positionPID.kp * positionError + 
                          positionPID.ki * positionPID.integral + 
                          positionPID.kd * derivative;
        positionPID.previousError = positionError;
        
        // Return angular correction to straighten the robot's path
        return correction;
    }
    return 0.0f;
}

float DifferentialKinematics::computeMovementCorrection()
{
    // Distance error in centimeters
    float distanceTraveled = abs(getTotalDistance() - movementStartDistance);
    float distanceError = targetDistance - distanceTraveled;
    
    uint32_t now = millis();
    uint32_t dt = (movementPIDF.lastUpdateMs != 0) ? (now - movementPIDF.lastUpdateMs) : 0;
    movementPIDF.lastUpdateMs = now;

    if (dt > 0) {
        float dtSec = dt / 1000.0f;
        movementPIDF.integral += distanceError * dtSec;
        
        // Limit integral term
        float maxIntegral = 5.0f;
        if (movementPIDF.integral > maxIntegral) movementPIDF.integral = maxIntegral;
        if (movementPIDF.integral < -maxIntegral) movementPIDF.integral = -maxIntegral;
        
        float derivative = (dt > 0) ? (distanceError - movementPIDF.previousError) / dtSec : 0.0f;
        
        // PIDF: PID + feedforward term
        // Feedforward compensates for expected velocity to maintain target speed
        float feedforward = movementPIDF.kf * targetLinearSpeed;
        
        float correction = movementPIDF.kp * distanceError + 
                          movementPIDF.ki * movementPIDF.integral + 
                          movementPIDF.kd * derivative +
                          feedforward;
        movementPIDF.previousError = distanceError;
        
        return correction;
    }
    return 0.0f;
}


void DifferentialKinematics::Update()
{
    computeDerivedState();
    
    // Handle movement states
    if (movementState == MovementState::MOVING_FORWARD || 
        movementState == MovementState::MOVING_BACKWARD ||
        movementState == MovementState::HOLDING_POSITION) {
        
        // Check if movement is complete
        if (movementState != MovementState::HOLDING_POSITION && IsMovementComplete()) {
            movementState = MovementState::IDLE;
            SetKinematicTargets(0.0f, 0.0f);
        } else {
            // Compute dual corrections:
            // 1. Heading correction (maintain target heading)
            float headingCorrection = computeHeadingCorrection();
            
            // 2. Position correction (keep robot on straight line)
            // NOTE: Only enable position correction during forward movement
            // During backward movement, disable it to prevent unwanted rotation
            float positionCorrection = (movementDirection > 0) ? computePositionCorrection() : 0.0f;
            
            // Combine corrections: sum them for total angular control
            float totalAngularCorrection = headingCorrection + positionCorrection;
            
            // Apply corrected targets with combined heading + position maintenance
            float linearSpeed = (movementState == MovementState::HOLDING_POSITION) ? 0.0f : 
                               (movementDirection * targetLinearSpeed);
            SetKinematicTargets(linearSpeed, totalAngularCorrection);
        }
    }
    
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
    output.print(',');
    output.print(state.leftMotorPwmPercent, 2);
    output.print(',');
    output.print(state.rightMotorPwmPercent, 2);
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
    
    // Calculate PWM percentage based on target RPS (0-10 RPS = 0-100% PWM)
    const float maxRpsForPwm = 10.0f;
    state.leftMotorPwmPercent = (state.leftTargetRps / maxRpsForPwm) * 100.0f;
    state.rightMotorPwmPercent = (state.rightTargetRps / maxRpsForPwm) * 100.0f;
    
    // Clamp PWM to -100 to 100 range
    if (state.leftMotorPwmPercent > 100.0f) state.leftMotorPwmPercent = 100.0f;
    if (state.leftMotorPwmPercent < -100.0f) state.leftMotorPwmPercent = -100.0f;
    if (state.rightMotorPwmPercent > 100.0f) state.rightMotorPwmPercent = 100.0f;
    if (state.rightMotorPwmPercent < -100.0f) state.rightMotorPwmPercent = -100.0f;
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
