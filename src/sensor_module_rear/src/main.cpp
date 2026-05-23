#include <Arduino.h>
#include "can.h"
#include "flow_sensor.h"
#include "temp_sensor.h"
#include "pressure_sensor.h"

// Arduino Pins
#define TEMP_1_PIN 16
#define TEMP_2_PIN 18
#define FLOW_PIN 14
#define PRESSURE_PIN 12

// Can IDs (check spreadsheet to assign available ids)
#define TEMP_1_CAN_ID 0x414
#define TEMP_2_CAN_ID 0x415
#define FLOW_CAN_ID 0x416
#define PRESSURE_CAN_ID 0x417

// Initialize CAN function
FlexCAN_T4<CAN2, RX_SIZE_256> can1;
FloatBytes fb;

// Create a Teensy IntervalTimer (hardware timer)
IntervalTimer flowRateTimer;

void setup() {
  Serial.begin(115200);
  pinMode(FLOW_PIN, INPUT_PULLUP);
  pinMode(TEMP_1_PIN,INPUT);
  pinMode(TEMP_2_PIN,INPUT);
  pinMode(PRESSURE_PIN,INPUT);
  can1.begin();
  can1.setBaudRate(250000);
  while (!Serial) {
    ;
  }
  Serial.println("Setup complete");
  flowRateTimer.begin(flowISR, FLOW_SENSOR_PERIOD);
}

void loop() {
  float temp_1 = temp_sensor_get_temp(TEMP_1_PIN, 1);
  can1.write(can_format_message(TEMP_1_CAN_ID, temp_1));
  float temp_2 = temp_sensor_get_temp(TEMP_2_PIN, 1);
  can1.write(can_format_message(TEMP_2_CAN_ID, temp_2));

  float flow_rate = flowRate();
  can1.write(can_format_message(FLOW_CAN_ID, flow_rate));

  float pressure = pressure_sensor_get_pressure(PRESSURE_PIN);
  can1.write(can_format_message(PRESSURE_CAN_ID, pressure));
  delay(1000);
}