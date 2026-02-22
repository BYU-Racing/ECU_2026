#include "SensorData.h"

SensorData::SensorData(const uint32_t id, const size_t msgCount)
{
    this->id = id;
    this->msgCount = msgCount;
    if (msgCount > 0)
    {
        msgs = new CAN_message_t[msgCount];
    }
}
SensorData::SensorData(const SensorData& other)
{
    id = other.id;
    msgCount = other.msgCount;
    if (msgCount > 0 && other.msgs != nullptr)
    {
        msgs = new CAN_message_t[msgCount];
        for (size_t i = 0; i < msgCount; i++)
        {
            msgs[i] = other.msgs[i];
        }
    }
}
SensorData& SensorData::operator=(const SensorData& other)
{
    if (this != &other)
    {
        delete[] msgs;
        id = other.id;
        msgCount = other.msgCount;
        if (msgCount > 0 && other.msgs != nullptr)
        {
            msgs = new CAN_message_t[msgCount];
            for (size_t i = 0; i < msgCount; i++)
            {
                msgs[i] = other.msgs[i];
            }
        }
    }
    return *this;
}
SensorData::~SensorData()
{
    delete[] msgs;
}
size_t SensorData::getMsgCount() const { return msgCount; }
CAN_message_t* SensorData::getMsgs() const { return msgs; }
CAN_message_t SensorData::newMsg(const uint8_t* buf, const size_t len) const
{
    CAN_message_t msg;
    msg.id = static_cast<uint32_t>(id);
    const size_t maxLen = len > 8 ? 8 : len; // Len can't exceed 8
    msg.len = maxLen;
    for (size_t i = 0; i < maxLen; i++)
    {
        msg.buf[i] = buf[i];
    }
    return msg;
}
bool SensorData::setMsg(const uint8_t* buf, const size_t len, const size_t msgIndex)
{
    if (buf == nullptr || msgIndex >= msgCount) { return false; }
    const CAN_message_t msg = newMsg(buf, len);
    msgs[msgIndex] = msg;
    return true;
}
