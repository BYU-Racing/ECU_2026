#ifndef DATACOLLECTOR_H
#define DATACOLLECTOR_H

#include <FlexCAN_T4.h>
#include <Sensor.h>
#include <Reserved.h>

constexpr uint32_t HEALTH_CHECK_REQ_ID = 100;
constexpr size_t MAX_BUF_SIZE = 8;

class DataCollector {
    uint32_t id = ReservedIDs::INVALIDId;
    size_t numSensors = 0;
    Sensor** sensors = nullptr;
    FlexCAN_T4<CAN1, RX_SIZE_256>* motorCAN = nullptr;
    FlexCAN_T4<CAN2, RX_SIZE_256>* dataCAN = nullptr;
    CAN_message_t healthMsg;
    bool debug = false;
    /**
     * Get sensor data and send over CAN line.
     * sensor->debugPrint(CAN_messate_t) is called if debug is true.
     * @param sensor Sensor to read from
     */
    void read(Sensor* sensor) const;
    /**
     * Get a sensors health and ability to be read form
     * @param sensor Sensor to update the DataCollector health with
     * @return true if sensor can be read from, false otherwise
     */
    static Health healthCheck(const Sensor* sensor);
public:
    /** begin() must be called after initialization for CAN functionality */
    DataCollector(ReservedIDs id, size_t numSensors, Sensor** sensors, bool debug = false);
    ~DataCollector() = default; // No pointers are owned, no `delete` necessary
    /**
     * Poll all of the sensors given to the DataCollector.
     * Calls `healthCheck()` if a health check request is received via CAN
     */
    void checkSensors();
    /**
     * Assigns a CAN line to read/write to/from.
     * Must be called prior to checkSensors() or healthCheck().
     */
    void begin(FlexCAN_T4<CAN1, RX_SIZE_256>* motorCAN, FlexCAN_T4<CAN2, RX_SIZE_256>* dataCAN);
    /** Sends a health check message to the CAN line */
    void healthCheck();
};

#endif // DATACOLLECTOR_H
