#include <Arduino.h>
#include <esp32_motor.hpp>
#include <kinematics.hpp>

#define leftMotorReversed true
#define rightMotorReversed false
#define leftMotor Motor2
#define rightMotor Motor1
#define leftEncoderPinA 6
#define leftEncoderPinB 7
#define rightEncoderPinA 2
#define rightEncoderPinB 1
#define leftMotorPulsesPerRevolution 937.2
#define rightMotorPulsesPerRevolution 937.2
#define leftWheelPerimeterCm 21.1
#define rightWheelPerimeterCm 21.1
#define wheelBaseCm 13.45

#define COLOR_SENSOR_SCL_PIN 9
#define COLOR_SENSOR_SDA_PIN 8

extern HWCDC Serial;
