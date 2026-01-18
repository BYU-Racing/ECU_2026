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
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::ThrottleOnePosition && msg.len >= 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_one_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleOnePosition, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_throttle_two_position(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::ThrottleTwoPosition && msg.len >= 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_two_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleTwoPosition, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_brake_pressure(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::BrakePressure && msg.len >= 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_brake_pressure(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::BrakePressure, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

RvcMessage parse_rvc(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == (uint8_t)MessageId::Rvc && msg.len == 5);
    // Make sure the rvc type is one of the six valid types.
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
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::TemperaturesOne) && msg.len == 8);

    MotorTemperaturesOne temps;
    temps.phase_a_temp     = read_i16_le(&msg.buf[0]);
    temps.phase_b_temp     = read_i16_le(&msg.buf[2]);
    temps.phase_c_temp     = read_i16_le(&msg.buf[4]);
    temps.gate_driver_temp = read_i16_le(&msg.buf[6]);

    return temps;
}

MotorTemperaturesTwo parse_motor_temperatures_two(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::TemperaturesTwo) && msg.len == 8);

    MotorTemperaturesTwo temps;
    temps.control_board_temp = read_i16_le(&msg.buf[0]);
    temps.rtd1               = read_i16_le(&msg.buf[2]);
    temps.rtd2               = read_i16_le(&msg.buf[4]);
    temps.rtd3               = read_i16_le(&msg.buf[6]);

    return temps;
}

MotorTemperaturesThree parse_motor_temperatures_three(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::TemperaturesThree) && msg.len == 8);

    MotorTemperaturesThree temps;
    temps.coolant_temp   = read_i16_le(&msg.buf[0]);
    temps.hot_spot_temp  = read_i16_le(&msg.buf[2]);
    temps.motor_temp     = read_i16_le(&msg.buf[4]);
    temps.torque_shudder = read_i16_le(&msg.buf[6]);

    return temps;
}

MotorAnalogInputVoltages parse_motor_analog_input_voltages(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::AnalogInputVoltages) && msg.len == 8);

    MotorAnalogInputVoltages voltages;
    voltages.analog_input_one   = read_i16_le(&msg.buf[0]);
    voltages.analog_input_two   = read_i16_le(&msg.buf[2]);
    voltages.analog_input_three = read_i16_le(&msg.buf[4]);
    voltages.analog_input_four  = read_i16_le(&msg.buf[6]);

    return voltages;
}

MotorDigitalInputStatus parse_motor_digital_input_status(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::DigitalInputStatus) && msg.len == 8);

    MotorDigitalInputStatus status;
    status.forward_switch       =   msg.buf[0] != 0;
    status.reverse_switch       =   msg.buf[1] != 0;
    status.brake_switch         =   msg.buf[2] != 0;
    status.regen_disable_switch =   msg.buf[3] != 0;
    status.ignition_switch      =   msg.buf[4] != 0;
    status.start_switch         =   msg.buf[5] != 0;
    status.valet_mode           =   msg.buf[6] != 0;
    status.digital_input_eight  =   msg.buf[7] != 0;

    return status;
}

MotorPositionInfo parse_motor_position_info(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::PositionInfo) && msg.len == 8);

    MotorPositionInfo info;
    info.motor_angle            = read_i16_le(&msg.buf[0]);
    info.motor_speed            = read_i16_le(&msg.buf[2]);
    info.electrical_frequency   = read_i16_le(&msg.buf[4]);
    info.delta_resolver_angle   = read_i16_le(&msg.buf[6]);

    return info;
}

MotorCurrentInfo parse_motor_current_info(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::CurrentInfo) && msg.len == 8);

    MotorCurrentInfo info;
    info.phase_a_current = read_i16_le(&msg.buf[0]);
    info.phase_b_current = read_i16_le(&msg.buf[2]);
    info.phase_c_current = read_i16_le(&msg.buf[4]);
    info.dc_bus_current  = read_i16_le(&msg.buf[6]);

    return info;
}

MotorVoltageInfo parse_motor_voltage_info(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::VoltageInfo) && msg.len == 8);

    MotorVoltageInfo info;
    info.dc_bus_voltage = read_i16_le(&msg.buf[0]);
    info.output_voltage = read_i16_le(&msg.buf[2]);
    info.vab_voltage    = read_i16_le(&msg.buf[4]);
    info.vbc_voltage    = read_i16_le(&msg.buf[6]);

    return info;
}

MotorFluxInfo parse_motor_flux_info(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::FluxInfo) && msg.len == 8);

    MotorFluxInfo info;
    info.flux_command  = read_i16_le(&msg.buf[0]);
    info.flux_feedback = read_i16_le(&msg.buf[2]);
    info.id_feedback   = read_i16_le(&msg.buf[4]);
    info.iq_feedback   = read_i16_le(&msg.buf[6]);

    return info;
}

MotorInternalVoltages parse_motor_internal_voltages(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::InternalVoltages) && msg.len == 8);

    MotorInternalVoltages voltages;
    voltages.reference_1_5_v = read_i16_le(&msg.buf[0]);
    voltages.reference_2_5_v = read_i16_le(&msg.buf[2]);
    voltages.reference_5_0_v = read_i16_le(&msg.buf[4]);
    voltages.reference_12_v  = read_i16_le(&msg.buf[6]);

    return voltages;
}

MotorInternalStates parse_motor_internal_states(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::InternalStates));

    /* Make sure buf[0] is safe to cast to VsmState. */
    SAFETY_ASSERT(msg.buf[0] <= 7 || (msg.buf[0] >= 14 && msg.buf[0] <= 15));
    VsmState vsm_state = static_cast<VsmState>(msg.buf[0]);
    uint8_t pwm_frequency = msg.buf[1];

    /* Make sure buf[2] is safe to cast to InverterState. */
    SAFETY_ASSERT(msg.buf[2] <= 12);
    InverterState inverter_state = static_cast<InverterState>(msg.buf[2]);

    RelayState relay_state;
    relay_state.relay_one   = BIT_READ(msg.buf[3], 0);
    relay_state.relay_two   = BIT_READ(msg.buf[3], 1);
    relay_state.relay_three = BIT_READ(msg.buf[3], 2);
    relay_state.relay_four  = BIT_READ(msg.buf[3], 3);
    relay_state.relay_five  = BIT_READ(msg.buf[3], 4);
    relay_state.relay_six   = BIT_READ(msg.buf[3], 5);

    InverterRunMode inverter_run_mode;
    if (BIT_READ(msg.buf[4], 0)) {
        inverter_run_mode = InverterRunMode::SpeedMode;
    } else {
        inverter_run_mode = InverterRunMode::TorqueMode;
    }

    bool self_sensing_assist_enable = BIT_READ(msg.buf[4], 1);

    // FIXME properly parse this
}

MotorControlCommand parse_motor_control_command(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::ControlCommand) && msg.len == 8);

    MotorControlCommand result;
    result.torque = read_i16_le(&msg.buf[0]);
    result.speed = read_i16_le(&msg.buf[2]);

    /* Convert the direction value into an enum so it's easier to use. */
    if (msg.buf[4] != 0) {
        result.direction = MotorDirection::Forward;
    } else {
        result.direction = MotorDirection::Reverse;
    }

    result.enable_inverter = BIT_READ(msg.buf[5], 0);
    result.inverter_discharge = BIT_READ(msg.buf[5], 1);
    result.override_speed = BIT_READ(msg.buf[5], 2);
    result.torque_limit = read_i16_le(&msg.buf[6]);

    return result;
}

