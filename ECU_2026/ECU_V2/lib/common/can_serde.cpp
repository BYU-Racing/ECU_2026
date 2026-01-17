#include "can_serde.hpp"

#include <cstring>

#include "assert.hpp"

/* We do a lot of bit mucking below, so we'll use these quite a bit. */
#define BIT_READ(value, bit) ((bool)(((value) >> (bit)) & 0x01))
#define BIT_SET(value, bit) ((value) |= (1UL << (bit)))

CAN_message_t empty_can_message(MessageId id, uint8_t len) {
    CAN_message_t result = {0};
    result.id = (uint8_t)id;
    result.len = len;
    return result;
}

/* u = unsigned int, 16 = 16 bits, le = little endian. */
uint16_t read_u16_le(uint8_t* buf) {
    return (static_cast<uint16_t>(buf[1]) << 8) | buf[0];
}

void write_u16_le(uint8_t* buf, uint16_t value) {
    buf[0] = value & 0xFF;
    buf[1] = value >> 8;
}

/* i = signed int, 16 = 16 bits, le = little endian. */
int16_t read_i16_le(uint8_t* buf) {
    return static_cast<int16_t>(read_u16_le(buf));
}

void write_i16_le(uint8_t* buf, int16_t value) {
    write_u16_le(buf, static_cast<uint16_t>(value));
}

uint32_t read_u32_le(const uint8_t* buf) {
    return  static_cast<uint32_t>(buf[0])        |
           (static_cast<uint32_t>(buf[1]) <<  8) |
           (static_cast<uint32_t>(buf[2]) << 16) |
           (static_cast<uint32_t>(buf[3]) << 24);
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
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::StartSwitch);
    return msg.buf[0] != 0;
}

CAN_message_t create_start_switch(bool value) {
    CAN_message_t new_message = empty_can_message(MessageId::StartSwitch, 2);
    new_message.buf[0] = value;
    return new_message;
}

uint16_t parse_throttle_one_position(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::ThrottleOnePosition && msg.len == 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_one_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleOnePosition, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_throttle_two_position(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::ThrottleTwoPosition && msg.len == 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_two_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleTwoPosition, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_throttle_brake_pressure(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::BrakePressure && msg.len == 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_brake_pressure(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::BrakePressure, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

RvcMessage parse_rvc(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::Rvc && msg.len == 5);
    // Make sure the rcv type is one of the six valid types.
    uint8_t rvc_type = msg.buf[0];
    SAFETY_ASSERT(rvc_type < 6);

    float value = read_f32_le(&msg.buf[1]);

    return (RvcMessage) { (RvcType)rvc_type, value };
}

CAN_message_t create_rvc(RvcMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::Rvc, 5);
    new_message.buf[0] = (uint8_t)value.type;
    write_f32_le(&new_message.buf[1], value.value);
    return new_message;
}

TireRpmMessage parse_tire_rpm(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::TireRpm && msg.len == 5);
    // Make sure the tire position is one of the valid positions.
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position < 4);

    float value = read_f32_le(&msg.buf[1]);

    return (TireRpmMessage) { (TirePosition)tire_position, value };
}

CAN_message_t create_tire_rpm(TireRpmMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::TireRpm, 5);
    new_message.buf[0] = (uint8_t)value.position;
    write_f32_le(&new_message.buf[1], value.value);
    return new_message;
}

TireTemperatureMessage parse_tire_temperature(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::TireTemperature && msg.len == 7);
    // Make sure the tire position is one of the valid positions.
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position < 4);

    int16_t inner = read_i16_le(&msg.buf[1]);
    int16_t outer = read_i16_le(&msg.buf[3]);
    int16_t core = read_i16_le(&msg.buf[5]);

    return (TireTemperatureMessage) { (TirePosition)tire_position, inner, outer, core };
}

CAN_message_t create_tire_temperature(TireTemperatureMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::TireTemperature, 7);
    new_message.buf[0] = (uint8_t)value.position;

    write_i16_le(&new_message.buf[1], value.inner);
    write_i16_le(&new_message.buf[3], value.outer);
    write_i16_le(&new_message.buf[5], value.core);

    return new_message;
}

LapMessage parse_lap(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::Lap);
    // Make sure the lap message is one of the valid types.
    uint8_t lap_type = msg.buf[0];
    SAFETY_ASSERT(lap_type < 5);

    return (LapMessage)lap_type;
}

CAN_message_t create_lap(LapMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::Lap, 1);
    new_message.buf[0] = (uint8_t)value;
    return new_message;
}


/* Motor messages */
MotorTemperaturesOne parse_motor_temperatures_one(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::TemperaturesOne && msg.len == 1);
    return (MotorTemperaturesOne) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorTemperaturesTwo parse_motor_temperatures_two(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::TemperaturesTwo);
    return (MotorTemperaturesTwo) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorTemperaturesThree parse_motor_temperatures_three(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::TemperaturesThree);
    return (MotorTemperaturesThree) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorAnalogInputVoltages parse_motor_analog_input_voltages(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::AnalogInputVoltages);
    return (MotorAnalogInputVoltages) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorDigitalInputStatus parse_motor_digital_input_status(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::DigitalInputStatus);
    return (MotorDigitalInputStatus) {
        msg.buf[0] != 0,
        msg.buf[1] != 0,
        msg.buf[2] != 0,
        msg.buf[3] != 0,
        msg.buf[4] != 0,
        msg.buf[5] != 0,
        msg.buf[6] != 0,
        msg.buf[7] != 0,
    };
}

MotorPositionInfo parse_motor_position_info(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::PositionInfo);
    return (MotorPositionInfo) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorCurrentInfo parse_motor_current_info(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::CurrentInfo);
    return (MotorCurrentInfo) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorVoltageInfo parse_motor_voltage_info(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::VoltageInfo);
    return (MotorVoltageInfo) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorFluxInfo parse_motor_flux_info(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::FluxInfo);
    return (MotorFluxInfo) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorInternalVoltages parse_motor_internal_voltages(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::InternalVoltages);
    return (MotorInternalVoltages) {
        read_i16_le(&msg.buf[0]),
        read_i16_le(&msg.buf[2]),
        read_i16_le(&msg.buf[4]),
        read_i16_le(&msg.buf[6]),
    };
}

MotorInternalStates parse_motor_internal_states(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::InternalStates);

    // Make sure buf[0] is safe to cast to VsmState.
    SAFETY_ASSERT(msg.buf[0] <= 7 || (msg.buf[0] >= 14 && msg.buf[0] <= 15));
    VsmState vsm_state = (VsmState)msg.buf[0];
    uint8_t pwm_frequency = msg.buf[1];

    // Make sure buf[2] is safe to cast to InverterState.
    SAFETY_ASSERT(msg.buf[2] <= 12);
    InverterState inverter_state = (InverterState)msg.buf[2];

    // Make sure buf[3] is within bounds.
    SAFETY_ASSERT(msg.buf[3] < 0b01000000);
    RelayState relay_state = (RelayState) {
        BIT_READ(msg.buf[3], 0),
        BIT_READ(msg.buf[3], 1),
        BIT_READ(msg.buf[3], 2),
        BIT_READ(msg.buf[3], 3),
        BIT_READ(msg.buf[3], 4),
        BIT_READ(msg.buf[3], 5),
    };

    InverterRunMode inverter_run_mode;
    if (BIT_READ(msg.buf[4], 0)) {
        inverter_run_mode = InverterRunMode::SpeedMode;
    } else {
        inverter_run_mode = InverterRunMode::TorqueMode;
    }

    bool self_sensing_assist_enable = BIT_READ(msg.buf[4], 1);

    
}
