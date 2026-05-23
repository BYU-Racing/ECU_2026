#include "steering.h"

float angle_sensor_get_voltage(uint8_t pin){
    float out = analogRead(pin);
    out = (out/TEENSY_CONSTANT)*TS_VOLTAGE_MAX_CONSTANT/DIVIDER_CONSTANT;
    return(out);
}

float angle_sensor_get_angle(uint8_t pin){
    float out = angle_sensor_get_voltage(pin);
    out = (out - CENTER_VOLTAGE) * ANGLE_CONVERSION_FACTOR;
    return(out);
}