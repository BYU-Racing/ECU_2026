#include "can.hpp"

#include <cstdint>
#include <bit>

#include "FlexCAN_T4.h"

#include "util.hpp"

CAN_message_t empty_can_message(uint8_t id) {
    CAN_message_t result = {0};
    result.id = id;
    return result;
}

/* u = unsigned int, 16 = 16 bits, le = little endian. */
uint16_t read_u16_le(uint8_t* buf) {
    return buf[0] | buf[1] << 8;
}

void write_u16_le(uint8_t* buf, uint16_t value) {
    buf[0] = value & 0xFF;
    buf[1] = value >> 8;
}

uint32_t read_u32_le(uint8_t* buf) {
    return buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
}

void write_u32_le(uint8_t* buf, uint32_t value) {
    buf[0] =  value        & 0xFF;
    buf[1] = (value >>  8) & 0xFF;
    buf[2] = (value >> 16) & 0xFF;
    buf[3] = (value >> 24) & 0xFF;
}

float read_f32_le(uint8_t* buf) {
    uint32_t as_u32 = read_u32_le(buf);
    float as_f32;
    /* The only way to convert from int to float bit-per-bit is to do a memcpy. */
    memcpy(&as_f32, &as_u32, sizeof(as_f32));
    return as_f32;
}

void write_f32_le(uint8_t* buf, float value) {
    uint32_t as_u32;
    memcpy(&as_u32, &value, sizeof(as_u32));
    write_u32_le(buf, as_u32);
}

bool parse_start_switch(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::StartSwitch);
    return msg.buf[0] != 0;
}

CAN_message_t create_start_switch(bool value) {
    CAN_message_t new_message = empty_can_message(MessageId::StartSwitch);
    new_message.buf[0] = value;
    return new_message;
}

uint16_t parse_throttle_one_position(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::ThrottleOnePosition);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_one_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleOnePosition);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_throttle_two_position(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::ThrottleTwoPosition);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_two_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleTwoPosition);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_throttle_brake_pressure(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::BrakePressure);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_brake_pressure(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::BrakePressure);
    write_u16_le(new_message.buf, value);
    return new_message;
}

RvcMessage parse_rvc(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::Rvc);
    // Make sure the rcv type is one of the six valid types.
    uint8_t rvc_type = msg.buf[0];
    SAFETY_ASSERT(rvc_type < 6);

    float value = read_f32_le(&msg.buf[1]);

    return (RvcMessage) { (RvcType)rvc_type, value };
}

CAN_message_t create_rvc(RvcMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::Rvc);
    new_message.buf[0] = (uint8_t)value.type;
    write_f32_le(&new_message.buf[1], value.value);
    return new_message;
}

TireRpmMessage parse_tire_rpm(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::TireRpm);
    // Make sure the tire position is one of the valid positions.
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position < 4);

    float value = read_f32_le(&msg.buf[1]);

    return (TireRpmMessage) { (TirePosition)tire_position, value };
}

CAN_message_t create_tire_rpm(TireRpmMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::TireRpm);
    new_message.buf[0] = (uint8_t)value.position;
    write_f32_le(&new_message.buf[1], value.value);
    return new_message;
}

TireTemperatureMessage parse_tire_temperature(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::TireTemperature);
    // Make sure the tire position is one of the valid positions.
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position < 4);

    uint16_t value = read_u16_le(&msg.buf[1]);

    return (TireTemperatureMessage) { (TirePosition)tire_position, value};
}

CAN_message_t create_tire_temperature(TireTemperatureMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::TireTemperature);
    new_message.buf[0] = (uint8_t)value.position;
    write_u16_le(&new_message.buf[1], value.value);
    return new_message;
}

LapMessage parse_lap(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::Lap);
    // Make sure the lap message is one of the valid types.
    uint8_t lap_type = msg.buf[0];
    SAFETY_ASSERT(lap_type < 5);

    return (LapMessage)lap_type;
}

CAN_message_t create_lap(LapMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::Lap);
    new_message.buf[0] = (uint8_t)value;
    return new_message;
}
