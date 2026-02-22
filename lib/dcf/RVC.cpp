#include "RVC.h"
#include <Arduino.h>
#include <BufferPacker.h>

RVC::RVC(const ReservedIDs id, const bool criticality, const uint32_t readInterval, Adafruit_BNO08x_RVC* rvc)
{
    this->id = id;
    this->criticality = criticality;
    this->readInterval = readInterval;
    this->rvc = rvc;
    heading = new BNO08x_RVC_Data;
}

RVC::~RVC() { delete heading; }

void RVC::begin(HardwareSerial* serial) { rvc->begin(serial); }

Health RVC::healthCheck()
{
    if (rvc != nullptr) { return HEALTHY; } // Definitely not a true inidication of *healthy* operation
    if (criticality) { return CRITICAL; }
    return UNRESPONSIVE;
}

bool RVC::ready()
{
    if (millis() - lastRead >= readInterval)
    {
        return rvc->read(heading);
    }
    return false;
}

void RVC::setMsg(SensorData* sensorData, uint8_t* msgIndex, const float value, const RVCSubIDs subSensorId)
{
    uint8_t buf[sizeof(RVCSubIDs) + sizeof(float)];
    BufferPacker<sizeof(RVCSubIDs) + sizeof(float)> packer;
    packer.pack(subSensorId);
    packer.pack(value);
    packer.deepCopyTo(buf);
    sensorData->setMsg(buf, sizeof(RVCSubIDs) + sizeof(float), *msgIndex);
    (*msgIndex)++;
}

SensorData RVC::read()
{
    lastRead = millis();
    SensorData sensorData = SensorData(id, rvcMsgCount);
    uint8_t msgIndex = 0;
    setMsg(&sensorData, &msgIndex, heading->x_accel, RVCSubIDs::X_AccelId);
    setMsg(&sensorData, &msgIndex, heading->y_accel, RVCSubIDs::Y_AccelId);
    setMsg(&sensorData, &msgIndex, heading->z_accel, RVCSubIDs::Z_AccelId);
    setMsg(&sensorData, &msgIndex, heading->roll, RVCSubIDs::RollId);
    setMsg(&sensorData, &msgIndex, heading->pitch, RVCSubIDs::PitchId);
    setMsg(&sensorData, &msgIndex, heading->yaw, RVCSubIDs::YawId);
    return sensorData;
}

void RVC::printValue(const char label[], const float value, const char units[])
{
    Serial.print(label);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(units);
}

void RVC::debugPrint(const CAN_message_t& canMsg) const
{
    Serial.println("RVC CAN Message:");
    Serial.print("Timestamp: ");
    Serial.println(canMsg.timestamp);
    BufferPacker<> unpacker(canMsg.buf);
    const RVCSubIDs id = unpacker.unpack<RVCSubIDs>();
    const float value = unpacker.unpack<float>();
    switch (id)
    {
    case RVCSubIDs::X_AccelId:
        printValue("X Acceleration", value, "m/s^2");
        break;
    case RVCSubIDs::Y_AccelId:
        printValue("Y Acceleration", value, "m/s^2");
        break;
    case RVCSubIDs::Z_AccelId:
        printValue("Z Acceleration", value, "m/s^2");
        break;
    case RVCSubIDs::RollId:
        printValue("Roll", value, "deg");
        break;
    case RVCSubIDs::PitchId:
        printValue("Pitch", value, "deg");
        break;
    case RVCSubIDs::YawId:
        printValue("Yaw", value, "deg");
        break;
    default:
        break;
    }
    Serial.println();
}
