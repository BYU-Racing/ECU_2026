#include <Arduino.h>
#include "temp.h"
#include "can.h"
#include "speed.h"
#include "steering.h"

// Arduino Pins
#define TEMP_PIN 16
#define SPEED_PIN 14
#define STEERING_PIN 12

// Can IDs (check spreadsheet to assign available ids)
#define TEMP_CAN_ID 0x410
#define SPEED_CAN_ID 0x411
#define STEERING_CAN_ID 0x412

// Initialize CAN function
FlexCAN_T4<CAN2, RX_SIZE_256> can1;
FloatBytes fb;

// Create a Teensy IntervalTimer (hardware timer)
IntervalTimer wheelSpeedTimer;

void setup() {
  pinMode(TEMP_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  setSpeedPin(SPEED_PIN);
  Serial.begin(9600);
  can1.begin();
  can1.setBaudRate(250000);
  while (!Serial) {
    ;
  }
  Serial.println("Setup complete");
  wheelSpeedTimer.begin(speedISR, SPEED_SENSOR_PERIOD);
}

void loop() {
  float brake_temp = temp_sensor_get_temp(TEMP_PIN);
  can1.write(can_format_message(TEMP_CAN_ID, brake_temp));

  float wheel_speed = 1;//getSpeed();
  can1.write(can_format_message(SPEED_CAN_ID, wheel_speed));

  float steering_angle = angle_sensor_get_voltage(STEERING_PIN);
  can1.write(can_format_message(STEERING_CAN_ID, steering_angle));
  
  Serial.println("Running");
  delay(1000);
/*
  Serial.print("Temp: ");
  Serial.print(brake_temp);
  Serial.print(", Speed: ");
  Serial.print(wheel_speed);
  Serial.print(", Steering angle: ");
  Serial.println(steering_angle);
*/

}