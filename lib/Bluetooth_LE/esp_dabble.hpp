#ifndef _ESP32_DABBLE_H_
#define _ESP32_DABBLE_H_

#if defined(ESP32)

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Standard Dabble/HM-10 UUIDs
#define DABBLE_SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define DABBLE_CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

// ==== Dabble Gamepad Button Bits (Byte 1) ====
#define DABBLE_BUTTON_START    0x01  // Bit 0
#define DABBLE_BUTTON_SELECT   0x02  // Bit 1
#define DABBLE_BUTTON_TRIANGLE 0x04  // Bit 2
#define DABBLE_BUTTON_CIRCLE   0x08  // Bit 3
#define DABBLE_BUTTON_CROSS    0x10  // Bit 4
#define DABBLE_BUTTON_SQUARE   0x20  // Bit 5

// ==== Dabble Gamepad D-Pad Bits (Byte 2) ====
#define DABBLE_UP              0x01  // Bit 0
#define DABBLE_DOWN            0x02  // Bit 1
#define DABBLE_LEFT            0x04  // Bit 2
#define DABBLE_RIGHT           0x08  // Bit 3

// ==== Dabble Gamepad Class ====
class DabbleGamepad
{
private:
    uint8_t buttonState = 0;      // Byte 1: START, SELECT, TRIANGLE, CIRCLE, CROSS, SQUARE
    uint8_t previousButtonState = 0;
    uint8_t dpadState = 0;        // Byte 2: UP, DOWN, LEFT, RIGHT
    uint8_t previousDpadState = 0;
    bool connected = false;
    
    // Last received raw data for debugging
    uint8_t lastRawByte0 = 0;
    uint8_t lastRawByte1 = 0;
    uint8_t lastRawByte2 = 0;
    
public:
    static DabbleGamepad* instance;
    
    DabbleGamepad();
    void begin(String deviceName = "ESP32_Robot");
    void update();
    bool isConnected();
    
    // Button state query functions (Byte 1)
    bool isStartPressed();
    bool isSelectPressed();
    bool isTrianglePressed();
    bool isCirclePressed();
    bool isCrossPressed();
    bool isSquarePressed();
    
    // D-pad state query functions (Byte 2)
    bool isUpPressed();
    bool isDownPressed();
    bool isLeftPressed();
    bool isRightPressed();
    
    // Raw button state
    uint8_t getButtonState();
    uint8_t getPreviousButtonState();
    uint8_t getDpadState();
    uint8_t getPreviousDpadState();
    
    // Button change detection
    bool isStartJustPressed();
    bool isSelectJustPressed();
    bool isTriangleJustPressed();
    bool isCircleJustPressed();
    bool isCrossJustPressed();
    bool isSquareJustPressed();
    
    // State logging
    String getInputStateString();
    String getRawDataString();
    
    // Internal: Update button state from BLE packet
    void _updateButtonState(uint8_t functionId, uint8_t byte1, uint8_t byte2);
    void _setConnected(bool conn);
};

extern DabbleGamepad dabbleGamepad;

#endif
#endif
