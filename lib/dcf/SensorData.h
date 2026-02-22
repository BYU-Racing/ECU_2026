#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <cstdint>
#include <FlexCAN_T4.h>
#include "Reserved.h"

class SensorData
{
    uint32_t id = ReservedIDs::INVALIDId;
    size_t msgCount = 0;
    CAN_message_t* msgs = nullptr;
    CAN_message_t newMsg(const uint8_t* buf, size_t len) const;
public:
    SensorData() = default;
    SensorData(uint32_t id, size_t msgCount);
    SensorData(const SensorData& other);
    SensorData& operator=(const SensorData& other);
    ~SensorData();
    size_t getMsgCount() const;
    CAN_message_t* getMsgs() const;
    bool setMsg(const uint8_t* buf, size_t len, size_t msgIndex=0);
};

#endif //SENSORDATA_H
