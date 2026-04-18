#include <esp32_color.hpp>

esp32_color::esp32_color(int _sdaPin,
                         int _sclPin,
                         float _integrationTimeMs,
                         TCS34725::Gain _gainMode)
{
    wire = &Wire;
    sdaPin = _sdaPin;
    sclPin = _sclPin;
    integrationTimeMs = _integrationTimeMs;
    gainMode = _gainMode;
}

void esp32_color::init()
{
    if (initialized) return;

    wire->setPins(sdaPin, sclPin);
    wire->begin();

    attached = sensor.attach(*wire);
    if (attached) {
        sensor.integrationTime(integrationTimeMs);
        sensor.gain(gainMode);
    }

    initialized = true;
}

void esp32_color::scanI2C(Stream& serial)
{
    if (!initialized) init();

    serial.println("Scanning I2C bus...");
    for (uint8_t address = 1; address < 127; ++address) {
        wire->beginTransmission(address);
        if (wire->endTransmission() == 0) {
            serial.printf("I2C device found at address 0x%02X\n", address);
        }
        delay(5);
    }
}

bool esp32_color::attachedStatus() const
{
    return attached;
}

bool esp32_color::available()
{
    if (!initialized) init();
    if (!attached) return false;
    return sensor.available();
}

esp32_color::RGB esp32_color::readNormalized()
{
    if (!initialized) init();

    RGB rgb = {0.0f, 0.0f, 0.0f};
    if (!attached) return rgb;

    if (sensor.available()) {
        const auto& color = sensor.color();
        rgb.r = color.r;
        rgb.g = color.g;
        rgb.b = color.b;
        return rgb;
    }

    const auto& data = sensor.raw();
    if (data.c == 0) return rgb;

    rgb.r = (255.0f * data.r) / data.c;
    rgb.g = (255.0f * data.g) / data.c;
    rgb.b = (255.0f * data.b) / data.c;
    return rgb;
}

void esp32_color::printNormalized(Stream& serial)
{
    const RGB rgb = readNormalized();
    serial.printf("RGB normalized: %.1f %.1f %.1f\n", rgb.r, rgb.g, rgb.b);
}

const TCS34725::RawData& esp32_color::raw() const
{
    return sensor.raw();
}

esp32_color ColorSensor;
