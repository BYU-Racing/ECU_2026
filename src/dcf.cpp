#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "DigitalSensor.h"
#include "AnalogSensor.h"
#include "DataCollector.h"

constexpr bool THROTTLE_1_CRITICALITY = true;
constexpr uint8_t THROTTLE_1_PIN = 18;

constexpr bool THROTTLE_2_CRITICALITY = true;
constexpr uint8_t THROTTLE_2_PIN = 19;

constexpr uint32_t THROTTLE_INTERVAL = 100;

AnalogSensor throttle1 = AnalogSensor(ReservedIDs::Throttle1PositionId, THROTTLE_1_CRITICALITY, THROTTLE_1_PIN, THROTTLE_INTERVAL);
AnalogSensor throttle2 = AnalogSensor(ReservedIDs::Throttle2PositionId, THROTTLE_1_CRITICALITY, THROTTLE_2_PIN, THROTTLE_INTERVAL);

constexpr bool BRAKE_CRITICALITY = true;
constexpr uint8_t BRAKE_PIN = 20;
constexpr uint32_t BRAKE_INTERVAL = 100;

AnalogSensor brake = AnalogSensor(ReservedIDs::BrakePressureId, BRAKE_CRITICALITY, BRAKE_PIN, BRAKE_INTERVAL);

constexpr bool SWITCH_CRITICALITY = true;
constexpr uint8_t SWITCH_PIN = 21;
constexpr uint32_t SWITCH_INTERVAL = 100;

DigitalSensor startSwitch = DigitalSensor(ReservedIDs::StartSwitchId, SWITCH_CRITICALITY, SWITCH_PIN, SWITCH_INTERVAL);

constexpr size_t NUM_SENSORS = 4;
Sensor* SENSORS[] = {
    &throttle1,
    &throttle2,
    &brake,
    &startSwitch,
};
constexpr bool DEBUG = false;
FlexCAN_T4<CAN1, RX_SIZE_256> motorCAN;
FlexCAN_T4<CAN2, RX_SIZE_256> dataCAN;

DataCollector dc = DataCollector(ReservedIDs::DCFId, NUM_SENSORS, SENSORS, DEBUG);

constexpr uint32_t CAN_BAUD_RATE = 250000;
constexpr uint32_t SERIAL_BAUD_RATE = 9600;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    motorCAN.begin();
    motorCAN.setBaudRate(CAN_BAUD_RATE);
    dataCAN.begin();
    dataCAN.setBaudRate(CAN_BAUD_RATE);
    dc.begin(&motorCAN, &dataCAN);
    Serial.println("START");
}

void loop() {
    dc.checkSensors();
}
