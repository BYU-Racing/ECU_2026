#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "DigitalSensor.h"
#include "AnalogSensor.h"

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

Sensor *SENSORS[] = {
    &throttle1,
    &throttle2,
    &brake,
    &startSwitch,
};
constexpr size_t NUM_SENSORS = sizeof(SENSORS) / sizeof(SENSORS[0]);
constexpr bool DEBUG = false;
FlexCAN_T4<CAN1, RX_SIZE_256> motorCAN;
FlexCAN_T4<CAN2, RX_SIZE_256> dataCAN;

constexpr uint32_t CAN_BAUD_RATE = 250000;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    motorCAN.begin();
    motorCAN.setBaudRate(CAN_BAUD_RATE);
    dataCAN.begin();
    dataCAN.setBaudRate(CAN_BAUD_RATE);
    Serial.println("START");
}

void sensorHealthCheck() {
    Health status[NUM_SENSORS];
    for (size_t sensorIndex = 0; sensorIndex < NUM_SENSORS; sensorIndex++)
    {
        status[sensorIndex] = SENSORS[sensorIndex]->healthCheck();
    }
    CAN_message_t healthMsg;
    healthMsg.id = ReservedIDs::DCFId;
    healthMsg.len = NUM_SENSORS;
    /* We currently can only report up to 8 statuses, since we only
     * use a single message. */
    static_assert(NUM_SENSORS <= 8);
    for (size_t i = 0; i < NUM_SENSORS; i++)
    {
        healthMsg.buf[i] = status[i];
    }
    dataCAN.write(healthMsg);
}

void processSensors() {
    for (size_t i = 0; i < NUM_SENSORS; i++)
    {
        Sensor *sensor = SENSORS[i];
        if (sensor->ready())
        {
            const SensorData data = sensor->read();
            const CAN_message_t *msgs = data.getMsgs();
            for (size_t msgIndex = 0; msgIndex < data.getMsgCount(); msgIndex++)
            {
                if (sensor->isCritical())
                {
                    motorCAN.write(msgs[msgIndex]);
                }
                else
                {
                    dataCAN.write(msgs[msgIndex]);
                }
                if (DEBUG)
                {
                    sensor->debugPrint(msgs[msgIndex]);
                }
            }
        }
    }
}

void loop()
{
    processSensors();
}
