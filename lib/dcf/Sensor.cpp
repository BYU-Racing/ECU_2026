#include "Sensor.h"

#define NUM_BITS 7

bool Sensor::isCritical() const { return criticality; }

bool Sensor::ready() { return millis() - lastRead >= readInterval; }

void Sensor::debugPrint(const CAN_message_t& canMsg) const
{
    Serial.println("CAN Message:");
    Serial.print("Timestamp: ");
    Serial.println(canMsg.timestamp);
    Serial.print("ID: ");
    Serial.println(canMsg.id);
    Serial.print("Len: ");
    Serial.println(canMsg.len);
    Serial.print("Data: ");
    for (size_t i = 0; i < canMsg.len; i++)
    {
        // Print each bit in each byte with leading 0s
        for (int8_t bit = NUM_BITS; bit >= 0; bit--) { Serial.print(bitRead(canMsg.buf[i], bit)); }
        // Print space if there is another byte to print
        if (i < canMsg.len - 1)
        {
            Serial.print(" ");
        } else
        {
            Serial.println();
        }
    }
    Serial.println();
}
