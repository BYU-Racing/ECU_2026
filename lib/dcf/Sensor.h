#ifndef SENSOR_H
#define SENSOR_H

#include <cstdint>
#include <FlexCAN_T4.h>
#include <Reserved.h>
#include "SensorData.h"

/** Representations of a sensor's health */
enum Health : uint8_t
{
    /** Health cannot be determined; use for default initializations */
    UNKNOWN,
    /** A critical sensor is unresponsive */
    CRITICAL,
    /** A non-critical sensor is unresponsive */
    UNRESPONSIVE,
    /** Limited operating functionality */
    DEGRADED,
    /** Full operating functionality */
    HEALTHY,
};

/**
 * Interface Sensors must implement
 */
class Sensor
{
protected:
    /** the sensor id */
    uint32_t id = ReservedIDs::INVALIDId;
    /** the sensor criticality */
    bool criticality = false;
    /** the interval a sensor should read at */
    uint32_t readInterval = 0;
    /** the last time a read occured */
    uint32_t lastRead = 0;
public:
    virtual ~Sensor() = default;
    /** @return the sensor criticality */
    [[nodiscard]] bool isCritical() const;
    /** @return the health of a sensor object */
    [[nodiscard]] virtual Health healthCheck();
    /**
     * Get the ready state of the sensor prior to reading
     * @return the ready state of the sensor
     */
    virtual bool ready();
    /**
     * "Fill" a sensorData object with sensor information for sending over a CAN bus
     * @return sensorData object containing the sensor data and CAN msgs
     */
    virtual SensorData read() = 0;
    /**
     * Print a CAN message data with sensor context (i.e. Sensor Name; named, scaled values with units)
     * @param canMsg msg to debug
     */
    virtual void debugPrint(const CAN_message_t& canMsg) const;
};

#endif //SENSOR_H
