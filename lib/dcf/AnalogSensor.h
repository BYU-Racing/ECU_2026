#ifndef ANALOGSENSOR_H
#define ANALOGSENSOR_H

#include <cstdint>
#include "Sensor.h"

class AnalogSensor : public Sensor
{
    uint8_t pin = 0;
public:
    AnalogSensor(ReservedIDs id, bool criticality, uint8_t pin, uint32_t readInterval);
    [[nodiscard]] Health healthCheck() override;
    SensorData read() override;
    bool ready() override;
};

#endif //ANALOGSENSOR_H
