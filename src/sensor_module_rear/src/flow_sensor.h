#pragma once
#include <Arduino.h>

#define SAMPLE_FREQUENCY_HZ 1000000
#define FLOW_SENSOR_PERIOD ((1.0f/SAMPLE_FREQUENCY_HZ)*1000000.0f)
#define FLOW_SENSOR_CALIBRATION 7.5
#define LITER_TIME_MULTIPLYER 1 //1 = per min a change to 60 would be per second
#define FLOW_SENSOR_NUM_DATA 50 //number of rises that it uses to find average

void setFlowPin(uint8_t inp);

void flowISR();

float flowRate();