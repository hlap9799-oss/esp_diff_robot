#if defined(ESP32)

#include "esp_dabble.hpp"

DabbleGamepad dabbleGamepad;
DabbleGamepad* DabbleGamepad::instance = nullptr;

// BLE Server and Characteristic pointers
static BLEServer *pServer = nullptr;
static BLECharacteristic *pCharacteristic = nullptr;

// Volatile buffer for rapid BLE data (prevent callback interruption)
static volatile uint8_t bleBuffer[4] = {0, 0, 0, 0};
static volatile bool newDataFlag = false;
static volatile uint8_t bleBufferLen = 0;

// Callbacks for BLE events
class DabbleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        Serial.println("[Dabble] Client connected");
        if (DabbleGamepad::instance) {
            DabbleGamepad::instance->_setConnected(true);
        }
    }

    void onDisconnect(BLEServer* pServer) override {
        Serial.println("[Dabble] Client disconnected");
        if (DabbleGamepad::instance) {
            DabbleGamepad::instance->_setConnected(false);
        }
        // Restart advertising on disconnect
        BLEDevice::startAdvertising();
    }
};

// Callbacks for characteristic writes (packet reception)
class DabbleCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        
        // Fast path: just store data in volatile buffer and set flag
        // Processing happens in update() to avoid callback bloat
        if (value.length() >= 3) {
            bleBufferLen = (value.length() > 4) ? 4 : value.length();
            for (int i = 0; i < bleBufferLen; i++) {
                bleBuffer[i] = (uint8_t)value[i];
            }
            newDataFlag = true;
        }
    }
};

DabbleGamepad::DabbleGamepad()
    : buttonState(0), previousButtonState(0), dpadState(0), previousDpadState(0), connected(false)
{
    instance = this;
}

void DabbleGamepad::begin(String deviceName)
{
    Serial.println("[Dabble] Initializing BLE...");
    
    BLEDevice::init(deviceName.c_str());
    
    // Create server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new DabbleServerCallbacks());
    
    // Create service
    BLEService *pService = pServer->createService(DABBLE_SERVICE_UUID);
    
    // Create characteristic
    pCharacteristic = pService->createCharacteristic(
        DABBLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ   |
        BLECharacteristic::PROPERTY_WRITE  |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_WRITE_NR
    );
    
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new DabbleCharacteristicCallbacks());
    
    pService->start();
    
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(DABBLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();
    
    Serial.println("[Dabble] BLE initialized. Waiting for Dabble app connection...");
}

void DabbleGamepad::update()
{
    // Store previous states to detect "just pressed" events
    previousButtonState = buttonState;
    previousDpadState = dpadState;
    
    // Process new BLE data if available (non-blocking)
    if (newDataFlag) {
        newDataFlag = false;  // Clear flag immediately
        
        // Expected format: [functionId] [buttons] [dpad] [reserved...]
        // Minimum 3 bytes required
        if (bleBufferLen >= 3) {
            uint8_t functionId = bleBuffer[0];
            uint8_t buttons = bleBuffer[1];
            uint8_t dpad = bleBuffer[2];
            
            // Debug log (happens in main loop, not in interrupt)
            Serial.print("[Dabble] RX: ");
            for (int i = 0; i < bleBufferLen; i++) {
                Serial.printf("%02X ", bleBuffer[i]);
            }
            Serial.println();
            
            // Update button state
            if (instance) {
                instance->_updateButtonState(functionId, buttons, dpad);
            }
        }
    }
}

bool DabbleGamepad::isConnected()
{
    return connected;
}

// Button state query functions (Byte 1)
bool DabbleGamepad::isStartPressed()
{
    return (buttonState & DABBLE_BUTTON_START) != 0;
}

bool DabbleGamepad::isSelectPressed()
{
    return (buttonState & DABBLE_BUTTON_SELECT) != 0;
}

bool DabbleGamepad::isTrianglePressed()
{
    return (buttonState & DABBLE_BUTTON_TRIANGLE) != 0;
}

bool DabbleGamepad::isCirclePressed()
{
    return (buttonState & DABBLE_BUTTON_CIRCLE) != 0;
}

bool DabbleGamepad::isCrossPressed()
{
    return (buttonState & DABBLE_BUTTON_CROSS) != 0;
}

bool DabbleGamepad::isSquarePressed()
{
    return (buttonState & DABBLE_BUTTON_SQUARE) != 0;
}

// D-pad state query functions (Byte 2)
bool DabbleGamepad::isUpPressed()
{
    return (dpadState & DABBLE_UP) != 0;
}

bool DabbleGamepad::isDownPressed()
{
    return (dpadState & DABBLE_DOWN) != 0;
}

bool DabbleGamepad::isLeftPressed()
{
    return (dpadState & DABBLE_LEFT) != 0;
}

bool DabbleGamepad::isRightPressed()
{
    return (dpadState & DABBLE_RIGHT) != 0;
}

// Raw button state
uint8_t DabbleGamepad::getButtonState()
{
    return buttonState;
}

uint8_t DabbleGamepad::getPreviousButtonState()
{
    return previousButtonState;
}

uint8_t DabbleGamepad::getDpadState()
{
    return dpadState;
}

uint8_t DabbleGamepad::getPreviousDpadState()
{
    return previousDpadState;
}

// Button change detection (Byte 1)
bool DabbleGamepad::isStartJustPressed()
{
    return ((buttonState & DABBLE_BUTTON_START) != 0) && ((previousButtonState & DABBLE_BUTTON_START) == 0);
}

bool DabbleGamepad::isSelectJustPressed()
{
    return ((buttonState & DABBLE_BUTTON_SELECT) != 0) && ((previousButtonState & DABBLE_BUTTON_SELECT) == 0);
}

bool DabbleGamepad::isTriangleJustPressed()
{
    return ((buttonState & DABBLE_BUTTON_TRIANGLE) != 0) && ((previousButtonState & DABBLE_BUTTON_TRIANGLE) == 0);
}

bool DabbleGamepad::isCircleJustPressed()
{
    return ((buttonState & DABBLE_BUTTON_CIRCLE) != 0) && ((previousButtonState & DABBLE_BUTTON_CIRCLE) == 0);
}

bool DabbleGamepad::isCrossJustPressed()
{
    return ((buttonState & DABBLE_BUTTON_CROSS) != 0) && ((previousButtonState & DABBLE_BUTTON_CROSS) == 0);
}

bool DabbleGamepad::isSquareJustPressed()
{
    return ((buttonState & DABBLE_BUTTON_SQUARE) != 0) && ((previousButtonState & DABBLE_BUTTON_SQUARE) == 0);
}

String DabbleGamepad::getInputStateString()
{
    String state = "Gamepad: ";
    bool hasInput = false;
    
    // Buttons (Byte 1)
    if (isStartPressed()) { state += "START "; hasInput = true; }
    if (isSelectPressed()) { state += "SELECT "; hasInput = true; }
    if (isTrianglePressed()) { state += "TRI "; hasInput = true; }
    if (isCirclePressed()) { state += "CIR "; hasInput = true; }
    if (isCrossPressed()) { state += "CRO "; hasInput = true; }
    if (isSquarePressed()) { state += "SQU "; hasInput = true; }
    
    // D-pad (Byte 2)
    if (isUpPressed()) { state += "UP "; hasInput = true; }
    if (isDownPressed()) { state += "DOWN "; hasInput = true; }
    if (isLeftPressed()) { state += "LEFT "; hasInput = true; }
    if (isRightPressed()) { state += "RIGHT "; hasInput = true; }
    
    // If no buttons pressed
    if (!hasInput) state += "IDLE";
    
    return state;
}

String DabbleGamepad::getRawDataString()
{
    String raw = "Raw: ";
    raw += "[";
    raw += String(lastRawByte0, HEX);
    raw += " ";
    raw += String(lastRawByte1, HEX);
    raw += " ";
    raw += String(lastRawByte2, HEX);
    raw += "]";
    return raw;
}

void DabbleGamepad::_updateButtonState(uint8_t functionId, uint8_t byte1, uint8_t byte2)
{
    previousButtonState = buttonState;
    previousDpadState = dpadState;
    
    lastRawByte0 = functionId;  // Always 0x01 (GAMEPAD_DIGITAL)
    lastRawByte1 = byte1;       // Button state: START, SELECT, TRIANGLE, CIRCLE, CROSS, SQUARE
    lastRawByte2 = byte2;       // D-pad state: UP, DOWN, LEFT, RIGHT
    
    buttonState = byte1;
    dpadState = byte2;
}

void DabbleGamepad::_setConnected(bool conn)
{
    connected = conn;
}

#endif
