#include "can_serde.hpp"

#include <cstring>

#include "assert.hpp"

/* We do a lot of bit mucking below, so we'll use these quite a bit. */
#define BIT_READ(value, bit) ((bool)(((value) >> (bit)) & 0x01))
#define BIT_SET_TO(value, bit, set_to) if (set_to) { (value) |= (1UL << (bit)); }

CAN_message_t empty_can_message(MessageId id, uint8_t len) {
    CAN_message_t result = {0};
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises. */
    result.id = static_cast<uint8_t>(id);
    result.len = len;
    return result;
}

/* u = unsigned int, 16 = 16 bits, le = little endian. */
uint16_t read_u16_le(uint8_t* buf) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises. */
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
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises. */
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
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::StartSwitch) && msg.len >= 1);
    return msg.buf[0] != 0;
}

CAN_message_t create_start_switch(bool value) {
    CAN_message_t new_message = empty_can_message(MessageId::StartSwitch, 2);
    new_message.buf[0] = value;
    return new_message;
}

uint16_t parse_throttle_one_position(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::ThrottleOnePosition) && msg.len >= 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_one_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleOnePosition, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_throttle_two_position(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::ThrottleTwoPosition) && msg.len >= 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_two_position(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::ThrottleTwoPosition, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

uint16_t parse_brake_pressure(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::BrakePressure) && msg.len >= 2);
    return read_u16_le(msg.buf);
}

CAN_message_t create_throttle_brake_pressure(uint16_t value) {
    CAN_message_t new_message = empty_can_message(MessageId::BrakePressure, 2);
    write_u16_le(new_message.buf, value);
    return new_message;
}

RvcMessage parse_rvc(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::Rvc) && msg.len >= 5);
    // Make sure the rvc type is one of the six valid types.
    uint8_t rvc_type = msg.buf[0];
    SAFETY_ASSERT(rvc_type <= 5);

    float value = read_f32_le(&msg.buf[1]);

    RvcMessage result;
    result.type = static_cast<RvcType>(rvc_type);
    result.value = value;

    return result;
}

CAN_message_t create_rvc(RvcMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::Rvc, 5);
    new_message.buf[0] = (uint8_t)value.type;
    write_f32_le(&new_message.buf[1], value.value);
    return new_message;
}

TireRpmMessage parse_tire_rpm(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::TireRpm) && msg.len == 5);
    // Make sure the tire position is one of the valid positions.
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position <= 3);

    float value = read_f32_le(&msg.buf[1]);

    TireRpmMessage result;
    result.position = static_cast<TirePosition>(tire_position);
    result.value = value;

    return result;
}

CAN_message_t create_tire_rpm(TireRpmMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::TireRpm, 5);
    new_message.buf[0] = (uint8_t)value.position;
    write_f32_le(&new_message.buf[1], value.value);
    return new_message;
}

TireTemperatureMessage parse_tire_temperature(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::TireTemperature) && msg.len == 7);
    /* Make sure the tire position is one of the valid positions. */
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position <= 3);

    TireTemperatureMessage temps;
    temps.position = static_cast<TirePosition>(tire_position);
    temps.inner = read_i16_le(&msg.buf[1]);
    temps.outer = read_i16_le(&msg.buf[3]);
    temps.core  = read_i16_le(&msg.buf[5]);

    return temps;
}

CAN_message_t create_tire_temperature(TireTemperatureMessage value) {
    CAN_message_t new_message = empty_can_message(MessageId::TireTemperature, 7);
    new_message.buf[0] = static_cast<uint8_t>(value.position);

    write_i16_le(&new_message.buf[1], value.inner);
    write_i16_le(&new_message.buf[3], value.outer);
    write_i16_le(&new_message.buf[5], value.core);

    return new_message;
}

LapMessage parse_lap(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::Lap) && msg.len >= 1);
    /* Make sure the lap message is one of the valid types. */
    uint8_t lap_type = msg.buf[0];
    SAFETY_ASSERT(lap_type < 5);

    return static_cast<LapMessage>(lap_type);
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

    MotorInternalStates result;

    /* Make sure buf[0] is safe to cast to VsmState. */
    SAFETY_ASSERT(msg.buf[0] <= 7 || (msg.buf[0] >= 14 && msg.buf[0] <= 15));
    result.vsm_state = static_cast<VsmState>(msg.buf[0]);

    result.pwm_frequency = msg.buf[1];

    /* Make sure buf[2] is safe to cast to InverterState. */
    SAFETY_ASSERT(msg.buf[2] <= 12);
    result.inverter_state = static_cast<InverterState>(msg.buf[2]);

    RelayState relay_state;
    relay_state.relay_one   = BIT_READ(msg.buf[3], 0);
    relay_state.relay_two   = BIT_READ(msg.buf[3], 1);
    relay_state.relay_three = BIT_READ(msg.buf[3], 2);
    relay_state.relay_four  = BIT_READ(msg.buf[3], 3);
    relay_state.relay_five  = BIT_READ(msg.buf[3], 4);
    relay_state.relay_six   = BIT_READ(msg.buf[3], 5);
    /* Ignore bits 6 and 7. */
    result.relay_state = relay_state;

    if (BIT_READ(msg.buf[4], 0)) {
        result.inverter_run_mode = InverterRunMode::SpeedMode;
    } else {
        result.inverter_run_mode = InverterRunMode::TorqueMode;
    }

    result.self_sensing_assist_enable = BIT_READ(msg.buf[4], 1);

    uint8_t inverter_active_discharge_state = msg.buf[4] >> 5;
    /* FIXME this assert may be too strong, as the docs state that
     * "All other states are reserved for future use." May be better
     * to provide a default if it's out of range. */
    SAFETY_ASSERT(inverter_active_discharge_state <= 4);
    result.inverter_active_discharge_state =
        static_cast<InverterActiveDischargeState>(inverter_active_discharge_state);

    if (BIT_READ(msg.buf[5], 0)) {
        result.inverter_command_mode = InverterCommandMode::VsmMode;
    } else {
        result.inverter_command_mode = InverterCommandMode::CanMode;
    }

    result.rolling_counter_value = msg.buf[5] >> 4;

    result.inverter_enable_state = BIT_READ(msg.buf[6], 0);

    if (BIT_READ(msg.buf[6], 1)) {
        result.burst_model_mode = BurstModelMode::HighSpeed;
    } else {
        result.burst_model_mode = BurstModelMode::Stall;
    }

    result.start_mode_active = BIT_READ(msg.buf[6], 6);

    result.inverter_enable_lockout = BIT_READ(msg.buf[6], 7);

    if (BIT_READ(msg.buf[7], 0)) {
        result.direction_command = MotorDirection::Forward;
    } else {
        result.direction_command = MotorDirection::Reverse;
    }

    result.bms_active = BIT_READ(msg.buf[7], 1);

    result.bms_limiting_torque = BIT_READ(msg.buf[7], 2);

    result.limit_max_speed = BIT_READ(msg.buf[7], 3);

    result.limit_hot_spot = BIT_READ(msg.buf[7], 4);

    result.low_speed_limiting = BIT_READ(msg.buf[7], 5);

    result.coolant_temperature_limiting = BIT_READ(msg.buf[7], 6);

    result.limit_stall_burst_model = BIT_READ(msg.buf[7], 1);

    return result;
}

MotorFaultCodes parse_motor_fault_codes(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::FaultCodes) && msg.len == 8);

    MotorFaultCodes result;

    result.hardware_gate_or_desaturation_fault_first = BIT_READ(msg.buf[0], 0);
    result.hardware_overcurrent_fault           = BIT_READ(msg.buf[0], 1);
    result.accelerator_shorted                  = BIT_READ(msg.buf[0], 2);
    result.accelerator_open                     = BIT_READ(msg.buf[0], 3);
    result.current_sensor_low                   = BIT_READ(msg.buf[0], 4);
    result.current_sensor_high                  = BIT_READ(msg.buf[0], 5);
    result.module_temperature_low               = BIT_READ(msg.buf[0], 6);
    result.module_temperature_high              = BIT_READ(msg.buf[0], 7);

    result.control_pcb_temperature_low          = BIT_READ(msg.buf[1], 0);
    result.control_pcb_temperature_high         = BIT_READ(msg.buf[1], 1);
    result.gate_drive_pcb_temperature_low       = BIT_READ(msg.buf[1], 2);
    result.gate_drive_pcb_temperature_high      = BIT_READ(msg.buf[1], 3);
    result.sense_voltage_low_5v                 = BIT_READ(msg.buf[1], 4);
    result.sense_voltage_high_5v                = BIT_READ(msg.buf[1], 5);
    result.sense_voltage_low_12v                = BIT_READ(msg.buf[1], 6);
    result.sense_voltage_high_12v               = BIT_READ(msg.buf[1], 7);

    result.sense_voltage_low_2_5v               = BIT_READ(msg.buf[2], 0);
    result.sense_voltage_high_2_5v              = BIT_READ(msg.buf[2], 1);
    result.sense_voltage_low_1_5v               = BIT_READ(msg.buf[2], 2);
    result.sense_voltage_high_1_5v              = BIT_READ(msg.buf[2], 3);
    result.dc_bus_voltage_high                  = BIT_READ(msg.buf[2], 4);
    result.dc_bus_voltage_low                   = BIT_READ(msg.buf[2], 5);
    result.pre_charge_timeout                   = BIT_READ(msg.buf[2], 6);
    result.pre_charge_voltage_failure           = BIT_READ(msg.buf[2], 7);

    result.eeprom_checksum_invalid              = BIT_READ(msg.buf[3], 0);
    result.eeprom_data_out_of_range             = BIT_READ(msg.buf[3], 1);
    result.eeprom_update_required               = BIT_READ(msg.buf[3], 2);
    result.hardware_dc_bus_over_voltage_during_initialization = BIT_READ(msg.buf[3], 3);
    result.gate_drive_initialization            = BIT_READ(msg.buf[3], 4);
    result.reserved_bit_29                      = BIT_READ(msg.buf[3], 5);
    result.brake_shorted                        = BIT_READ(msg.buf[3], 6);
    result.brake_open                           = BIT_READ(msg.buf[3], 7);

    result.motor_over_speed_fault               = BIT_READ(msg.buf[4], 0);
    result.over_current_fault                   = BIT_READ(msg.buf[4], 1);
    result.over_voltage_fault                   = BIT_READ(msg.buf[4], 2);
    result.inverter_over_temperature_fault      = BIT_READ(msg.buf[4], 3);
    result.accelerator_input_shorted_fault      = BIT_READ(msg.buf[4], 4);
    result.accelerator_input_open_fault         = BIT_READ(msg.buf[4], 5);
    result.direction_command_fault              = BIT_READ(msg.buf[4], 6);
    result.inverter_response_timeout_fault      = BIT_READ(msg.buf[4], 7);

    result.hardware_gate_or_desaturation_fault_second  = BIT_READ(msg.buf[5], 0);
    result.hardware_over_current_fault          = BIT_READ(msg.buf[5], 1);
    result.under_voltage_fault                  = BIT_READ(msg.buf[5], 2);
    result.can_command_message_lost_fault       = BIT_READ(msg.buf[5], 3);
    result.motor_over_temperature_fault         = BIT_READ(msg.buf[5], 4);
    result.reserved_bit_45                      = BIT_READ(msg.buf[5], 5);
    result.reserved_bit_46                      = BIT_READ(msg.buf[5], 6);
    result.reserved_bit_47                      = BIT_READ(msg.buf[5], 7);

    result.brake_input_shorted_fault            = BIT_READ(msg.buf[6], 0);
    result.brake_input_open_fault               = BIT_READ(msg.buf[6], 1);
    result.module_a_over_temperature_fault      = BIT_READ(msg.buf[6], 2);
    result.module_b_over_temperature_fault      = BIT_READ(msg.buf[6], 3);
    result.module_c_over_temperature_fault      = BIT_READ(msg.buf[6], 4);
    result.pcb_over_temperature_fault           = BIT_READ(msg.buf[6], 5);
    result.gate_drive_board_one_over_temperature_fault = BIT_READ(msg.buf[6], 6);
    result.gate_drive_board_two_over_temperature_fault = BIT_READ(msg.buf[6], 7);

    result.gate_drive_board_three_over_temperature_fault = BIT_READ(msg.buf[7], 0);
    result.current_sensor_fault                          = BIT_READ(msg.buf[7], 1);
    result.gate_driver_over_voltage                      = BIT_READ(msg.buf[7], 2);
    result.hardware_dc_bus_over_voltage_fault_old        = BIT_READ(msg.buf[7], 3);
    result.hardware_dc_bus_over_voltage_fault_new        = BIT_READ(msg.buf[7], 4);
    result.reserved_bit_61                               = BIT_READ(msg.buf[7], 5);
    result.resolver_not_connected                        = BIT_READ(msg.buf[7], 6);
    result.reserved_bit_63                               = BIT_READ(msg.buf[7], 7);

    return result;
}

MotorTorqueAndTimerInfo parse_motor_torque_and_timer_info(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::TorqueAndTimerInfo) && msg.len == 8);

    MotorTorqueAndTimerInfo result;
    result.commanded_torque = read_i16_le(&msg.buf[0]);
    result.torque_feedback = read_i16_le(&msg.buf[2]);
    result.power_on_timer = read_u32_le(&msg.buf[4]);

    return result;
}

MotorModIndexAndFlux parse_motor_mod_index_and_flux(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::ModIndexAndFlux) && msg.len == 8);

    MotorModIndexAndFlux result;
    result.modulation_index = read_i16_le(&msg.buf[0]);
    result.flux_weakening_output = read_i16_le(&msg.buf[2]);
    result.id_command = read_i16_le(&msg.buf[4]);
    result.iq_command = read_i16_le(&msg.buf[6]);

    return result;
}

int16_t parse_motor_torque_capability(CAN_message_t msg) {
    /* Why use `static_cast`? You can think of it as a normal cast, but with fewer surprises.
     * We use the cast to extract the CAN message id from its name. */
    SAFETY_ASSERT(msg.id == static_cast<uint8_t>(MessageId::TorqueCapability) && msg.len >= 2);

    return read_i16_le(&msg.buf[0]);
}

CAN_message_t create_motor_control_command(MotorControlCommand value) {
    CAN_message_t new_message = empty_can_message(MessageId::ControlCommand, 8);

    write_i16_le(&new_message.buf[0], value.torque);
    write_i16_le(&new_message.buf[2], value.speed);

    new_message.buf[4] = static_cast<uint8_t>(value.direction);

    BIT_SET_TO(new_message.buf[5], 0, value.enable_inverter);
    BIT_SET_TO(new_message.buf[5], 1, value.inverter_discharge);
    BIT_SET_TO(new_message.buf[5], 2, value.override_speed);

    write_i16_le(&new_message.buf[6], value.torque_limit);

    return new_message;
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

