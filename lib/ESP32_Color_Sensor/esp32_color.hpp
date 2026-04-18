#ifndef ESP32_COLOR_HPP
#define ESP32_COLOR_HPP

#include <Arduino.h>
#include <Wire.h>
#include <TCS34725.h>

#define COLOR_SENSOR_SCL_PIN 9
#define COLOR_SENSOR_SDA_PIN 8

class esp32_color
{
public:
    struct RGB
    {
        float r;
        float g;
        float b;
    };

private:
    TCS34725 sensor;
    TwoWire* wire;
    int sdaPin;
    int sclPin;
    float integrationTimeMs;
    TCS34725::Gain gainMode;
    bool initialized = false;
    bool attached = false;

public:
    esp32_color(int sdaPin = COLOR_SENSOR_SDA_PIN,
                int sclPin = COLOR_SENSOR_SCL_PIN,
                float integrationTimeMs = 50.0f,
                TCS34725::Gain gainMode = TCS34725::Gain::X04);

    void init();
    void scanI2C(Stream& serial = Serial);
    bool attachedStatus() const;
    bool available();
    RGB readNormalized();
    void printNormalized(Stream& serial = Serial);
    const TCS34725::RawData& raw() const;
};

extern esp32_color ColorSensor;

#endif
