#ifndef RVC_H
#define RVC_H

#include <cstdint>
#include <cstddef>
#include <Adafruit_BNO08x_RVC.h>
#include "Sensor.h"

constexpr size_t rvcMsgCount = 6;

class RVC final : public Sensor
{
    Adafruit_BNO08x_RVC* rvc = nullptr;
    BNO08x_RVC_Data* heading = nullptr;
    static void printValue(const char label[], float value, const char units[]);
    static void setMsg(SensorData* sensorData, uint8_t* msgIndex, float value, RVCSubIDs subSensorId);
public:
    RVC(ReservedIDs id, bool criticality, uint32_t readInterval, Adafruit_BNO08x_RVC* rvc);
    ~RVC() override;
    void begin(HardwareSerial* serial);
    [[nodiscard]] Health healthCheck() override;
    bool ready() override;
    SensorData read() override;
    void debugPrint(const CAN_message_t& canMsg) const override;
};

#endif //RVC_H
