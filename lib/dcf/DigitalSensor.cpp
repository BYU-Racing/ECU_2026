#include "DigitalSensor.h"

DigitalSensor::DigitalSensor(const ReservedIDs id, const bool criticality, const uint8_t pin, const uint32_t readInterval)
{
    this->id = id;
    this->criticality = criticality;
    this->readInterval = readInterval;
    this->pin = pin;
    pinMode(pin, INPUT_PULLDOWN);
}

Health DigitalSensor::healthCheck()
{
    pinMode(pin, INPUT_PULLDOWN);
    if (digitalRead(pin) == 0) {
        pinMode(pin, INPUT_PULLUP);
        if(digitalRead(pin) == 0) {
            pinMode(pin, INPUT_PULLDOWN);
            return HEALTHY;
        }
    }
    pinMode(pin, INPUT_PULLDOWN);
    if (criticality) { return CRITICAL; }
    return UNRESPONSIVE;
}

SensorData DigitalSensor::read()
{
    lastRead = millis();
    const uint8_t buf[sizeof(uint8_t)] = { digitalRead(pin) };
    SensorData sensorData = SensorData(id, 1);
    sensorData.setMsg(buf, sizeof(uint8_t));
    return sensorData;
}
