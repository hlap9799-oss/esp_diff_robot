#include <main.h>

namespace {
constexpr uint32_t kSerialBaud = 921600;
constexpr uint32_t kSerialWaitTimeoutMs = 10000;
constexpr uint32_t kLogIntervalMs = 200;
constexpr float kPidKp = 100.0f;
constexpr float kPidKi = 0.0f;
constexpr float kPidKd = 0.0f;
constexpr float kHeadingPidKp = 2.0f;
constexpr float kHeadingPidKi = 0.2f;
constexpr float kHeadingPidKd = 1.0f;
constexpr float kPositionPidKp = 0.5f;
constexpr float kPositionPidKi = 0.05f;
constexpr float kPositionPidKd = 0.1f;
constexpr float kMovementPidFKp = 30.0f;
constexpr float kMovementPidFKi = 0.3f;
constexpr float kMovementPidFKd = 0.05f;
constexpr float kMovementPidFKf = 50.0f;

differential_controller::DifferentialKinematics* kinematics = nullptr;
uint32_t movementSequenceStartMs = 0;
bool movementSequenceActive = false;
int sequenceStep = 0; // 0: idle, 1: forward 20cm, 2: stop 1sec, 3: backward 25cm

void configureMotor(esp32_motor& motor, bool reversed, int encoderPinA, int encoderPinB, uint32_t pulsesPerRevolution)
{
    if (reversed) {
        motor.Reverse();
    }

    motor.SetEncoderPins(encoderPinA, encoderPinB);
    motor.SetEncoderPulsesPerRevolution(pulsesPerRevolution);
    motor.Stop();
}

void handleSerialCommand(const String& command)
{
    if (command.isEmpty()) return;
    
    if (command == "RESET") {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
        if (kinematics != nullptr) {
            kinematics->ResetPose(0.0f, 0.0f, 0.0f);
        }
        Serial.println("ACK:RESET");
    }
    else if (command == "RUN") {
        if (!movementSequenceActive) {
            movementSequenceActive = true;
            sequenceStep = 1;
            movementSequenceStartMs = millis();
            if (kinematics != nullptr) {
                // ===== MOVEMENT SEQUENCE =====
                // STEP 1: Move forward - ADJUST DISTANCE HERE
                kinematics->MoveForward(20.0f, 0.0f, 15.0f);  // 20cm forward
            }
            Serial.println("ACK:RUN");
            Serial.println("Step 1: Moving forward 20 cm");
        }
    }
    else if (command == "STOP") {
        movementSequenceActive = false;
        sequenceStep = 0;
        if (kinematics != nullptr) {
            kinematics->Stop();
        }
        Serial.println("ACK:STOP");
    }
    else if (command.startsWith("PID_SPEED,")) {
        // Format: PID_SPEED,kp,ki,kd
        String params = command.substring(10);
        float values[3];
        int count = 0;
        int lastIdx = 0;
        
        for (int i = 0; i <= params.length(); i++) {
            if (params[i] == ',' || i == params.length()) {
                values[count] = params.substring(lastIdx, i).toFloat();
                count++;
                lastIdx = i + 1;
                if (count >= 3) break;
            }
        }
        
        if (count == 3 && kinematics != nullptr) {
            kinematics->SetSpeedPID(values[0], values[1], values[2]);
            Serial.print("ACK:PID_SPEED,");
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.println(values[2], 3);
        }
    }
    else if (command.startsWith("PID_HEADING,")) {
        // Format: PID_HEADING,kp,ki,kd
        String params = command.substring(12);
        float values[3];
        int count = 0;
        int lastIdx = 0;
        
        for (int i = 0; i <= params.length(); i++) {
            if (params[i] == ',' || i == params.length()) {
                values[count] = params.substring(lastIdx, i).toFloat();
                count++;
                lastIdx = i + 1;
                if (count >= 3) break;
            }
        }
        
        if (count == 3 && kinematics != nullptr) {
            kinematics->SetHeadingPID(values[0], values[1], values[2]);
            Serial.print("ACK:PID_HEADING,");
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.println(values[2], 3);
        }
    }
    else if (command.startsWith("PID_POSITION,")) {
        // Format: PID_POSITION,kp,ki,kd
        String params = command.substring(13);
        float values[3];
        int count = 0;
        int lastIdx = 0;
        
        for (int i = 0; i <= params.length(); i++) {
            if (params[i] == ',' || i == params.length()) {
                values[count] = params.substring(lastIdx, i).toFloat();
                count++;
                lastIdx = i + 1;
                if (count >= 3) break;
            }
        }
        
        if (count == 3 && kinematics != nullptr) {
            kinematics->SetPositionPID(values[0], values[1], values[2]);
            Serial.print("ACK:PID_POSITION,");
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.println(values[2], 3);
        }
    }
    else if (command.startsWith("PIDF_MOVE,")) {
        // Format: PIDF_MOVE,kp,ki,kd,kf
        String params = command.substring(10);
        float values[4];
        int count = 0;
        int lastIdx = 0;
        
        for (int i = 0; i <= params.length(); i++) {
            if (params[i] == ',' || i == params.length()) {
                values[count] = params.substring(lastIdx, i).toFloat();
                count++;
                lastIdx = i + 1;
                if (count >= 4) break;
            }
        }
        
        if (count == 4 && kinematics != nullptr) {
            kinematics->SetMovementPIDF(values[0], values[1], values[2], values[3]);
            Serial.print("ACK:PIDF_MOVE,");
            Serial.print(values[0], 3); Serial.print(",");
            Serial.print(values[1], 3); Serial.print(",");
            Serial.print(values[2], 3); Serial.print(",");
            Serial.println(values[3], 3);
        }
    }
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
    Serial.println("=== Differential Robot - Forward/Backward with PID/PIDF Control ===");

    pinMode(0, INPUT);
    attachInterrupt(digitalPinToInterrupt(0), [](){
        // IO0 button: Start movement sequence
        if (!movementSequenceActive && kinematics != nullptr) {
            movementSequenceActive = true;
            sequenceStep = 1;
            movementSequenceStartMs = millis();
            // ===== MOVEMENT SEQUENCE =====
            // STEP 1: Move forward - ADJUST DISTANCE HERE
            kinematics->MoveForward(20.0f, 0.0f, 15.0f);  // 20cm forward
            Serial.println("ACK:RUN");
            Serial.println("Step 1: Moving forward 20 cm");
        }
    }, RISING);

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
    kinematics->SetHeadingPID(kHeadingPidKp, kHeadingPidKi, kHeadingPidKd);
    kinematics->SetPositionPID(kPositionPidKp, kPositionPidKi, kPositionPidKd);
    kinematics->SetMovementPIDF(kMovementPidFKp, kMovementPidFKi, kMovementPidFKd, kMovementPidFKf);
    kinematics->ResetPose(0.0f, 0.0f, 0.0f);
    
    Serial.println("Robot initialized. Ready for commands (RUN/STOP/RESET)");
    Serial.println("Commands:");
    Serial.println("  RUN - Start sequence: forward 20cm, stop 1sec, backward 25cm");
    Serial.println("  STOP - Stop current movement");
    Serial.println("  RESET - Reset encoder and pose");
    Serial.println("  PID_SPEED,kp,ki,kd - Set speed PID");
    Serial.println("  PID_HEADING,kp,ki,kd - Set heading PID");
    Serial.println("  PID_POSITION,kp,ki,kd - Set position PID (forward-only, keep straight line)");
    Serial.println("  PIDF_MOVE,kp,ki,kd,kf - Set movement PIDF");
    Serial.println("  IO0 button - Trigger movement sequence (RISING edge)");
}

void loop()
{
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();

    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        handleSerialCommand(command);
    }

    if (kinematics != nullptr) {
        kinematics->Update();

        // Handle movement sequence
        if (movementSequenceActive) {
            if (sequenceStep == 1 && kinematics->IsMovementComplete()) {
                // Step 1: Forward 20cm complete → Start Stop (Step 2)
                sequenceStep = 2;
                movementSequenceStartMs = millis();  // Reset timer for 1-second delay
                kinematics->Stop();
                Serial.println("Step 2: Stopping for 1 second (PWM=0)");
            } 
            else if (sequenceStep == 2) {
                // Step 2: Wait 1 second
                uint32_t elapsedMs = millis() - movementSequenceStartMs;
                if (elapsedMs >= 1000) {  // 1 second delay
                    // Step 2 complete → Start backward (Step 3)
                    sequenceStep = 3;
                    // ===== MOVEMENT SEQUENCE =====
                    // STEP 3: Move backward - ADJUST DISTANCE HERE
                    Serial.println("Step 3: Moving backward 25 cm");
                    kinematics->MoveBackward(25.0f, 0.0f, 15.0f);  // 25cm backward
                }
            }
            else if (sequenceStep == 3 && kinematics->IsMovementComplete()) {
                // Step 3: Backward 25cm complete → Sequence done
                sequenceStep = 0;
                movementSequenceActive = false;
                kinematics->Stop();
                Serial.println("Movement sequence complete!");
                kinematics->PrintPose();
            }
        }

        // Log telemetry
        if (now - lastLogMs >= kLogIntervalMs) {
            lastLogMs = now;
            kinematics->PrintTelemetry();
        }
    }

    delay(10);
}
