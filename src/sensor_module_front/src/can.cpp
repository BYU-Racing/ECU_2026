#include "can.h"

void write_float_le(CAN_message_t* msg, float* value) {
  memcpy(msg->buf, value, sizeof(float));
}

CAN_message_t can_format_message(int id, float value) {
  CAN_message_t canMsg;
  canMsg.id = id;
  canMsg.len = sizeof(float);
  write_float_le(&canMsg, &value);
  return canMsg;
}
