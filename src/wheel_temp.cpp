#include "Arduino.h"
#include "FlexCAN_T4.h"

FlexCAN_T4<CAN2, RX_SIZE_256> dataCAN;

constexpr uint32_t CAN_BAUD_RATE = 250000;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    dataCAN.begin();
    dataCAN.setBaudRate(CAN_BAUD_RATE);
}

void loop() {
    
}
