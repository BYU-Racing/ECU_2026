#pragma once
#include <Arduino.h>

#define TEENSY_CONSTANT 1023
#define TS_VOLTAGE_MAX_CONSTANT 3.3
#define DIVIDER_CONSTANT .5
#define INPUT_VOLTAGE 5

#define CENTER_VOLTAGE 2 // voltage while steering straight
#define VOLTAGE_AT_MAX_RIGHT 5 //voltage at maximum rotation
#define HALF_ANGLE_RANGE 90 //angle between steering straight and maximum rotation
#define ANGLE_CONVERSION_FACTOR (VOLTAGE_AT_MAX_RIGHT - CENTER_VOLTAGE) / (INPUT_VOLTAGE / 2) * HALF_ANGLE_RANGE //multiplier for voltage to angle calculation

float angle_sensor_get_voltage(uint8_t pin);

float angle_sensor_get_angle(uint8_t pin);