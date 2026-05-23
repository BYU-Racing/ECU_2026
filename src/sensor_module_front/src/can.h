#pragma once
#include "FlexCAN_T4.h"

union FloatBytes {
  float value;
  byte bytes[4];
};

void write_float_le(CAN_message_t* msg, float* value);

CAN_message_t can_format_message(int id, float value);
