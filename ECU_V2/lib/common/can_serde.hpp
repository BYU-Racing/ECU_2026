/*
 * This module adds support for serialization/deserialization (serde) of can messages.
 */
#pragma once

#include "CAN_message_t.hpp"

/* All the CAN messages with their associated IDs. */
enum class MessageId: uint32_t {
    /* Sensor messages. */
    StartSwitch = 0,
    ThrottleOnePosition = 1,
    ThrottleTwoPosition = 2,
    BrakePressure = 3,
    /* Acceleration and Rotation. */
    Rvc = 4,
    TireRpm = 5,
    /* Tire temperature (inner, outer, core). */
    TireTemperature = 6,
    BmsPercent = 7,
    BmsTemperature = 8,
    Gps = 9,
    Lap = 10,

    /* Motor messages, see internal docs for details. */
    /* Temperatures of Phase A, B, C, and Gate Driver Board. */
    TemperaturesOne = 160,
    /* Control Board Temperature and RTD values. */
    TemperaturesTwo = 161,
    /* Coolant, Hot Spot, Motor Temperature, and Torque Shudder. */
    TemperaturesThree = 162,
    /* Analog input voltages for the motor. */
    AnalogInputVoltages = 163,
    /* Status of digital inputs (ignition, brake, reverse, etc). */
    DigitalInputStatus = 164,
    /* Motor angle, speed, and electrical frequency. */
    PositionInfo = 165,
    /* Phase currents and DC Bus current. */
    CurrentInfo = 166,
    /* DC Bus voltage, output voltage, and measured phase voltages. */
    VoltageInfo = 167,
    /* Commanded and feedback flux, D-axis and Q-axis currents. */
    FluxInfo = 168,
    /* Internal reference voltages (1.5V, 2.5V, 5V, and system voltage). */
    InternalVoltages = 169,
    /* Internal states like VSM, Inverter, and Relay status. */
    InternalStates = 170,
    /* POST and Run Fault Codes. */
    FaultCodes = 171,
    /* Commanded torque, torque feedback, and power-on timer. */
    TorqueAndTimerInfo = 172,
    /* Modulation Index, Flux Weakening Output, D-axis and Q-axis currents. */
    ModIndexAndFlux = 173,
    /* For factory use only - not for CAN use. */
    FirmwareInfo = 174,
    /* Downloadable only - not for CAN use. */
    DiagnosticData = 175,
    /* Torque Command, Torque Feedback, Motor Speed, DC Bus Voltage. */
    HighSpeed = 176,
    TorqueCapability = 177,
    /* The Command Message is used to transmit data to the controller. This message
     * is sent from a user-supplied external controller to the motor controller. The
     * Control Message (0x0C0) is used to operate the controller via the CAN interface. */
    ControlCommand = 192,
    ParameterCommand = 193,
    ParameterResponse = 194,

    /* Project messages. */
    /* Request a health check for all sensors. No data in the buffers. */
    HealthCheck = 200,
    /* Health Check response from DC1. */
    HealthCheckResponseDc1 = 201,
    /* Health Check response from DC2. */
    HealthCheckResponseDc2 = 202,
    /* Health Check response from DC3. */
    HealthCheckResponseDc3 = 203,
    /* Critical Failure has occured. */
    Fault = 204,
    /* Car functional drive state. */
    DriveState = 205,
    /* Set the ECU driving mode - used for varied torque output. */
    DriveMode = 206,
    /* Set the minimum throttle value. */
    ThrottleMin = 207,
    /* Set the maximum throttle value. */
    ThrottleMax = 208,
};

enum class RvcType: uint8_t {
    XAccel = 0,
    YAccel = 1,
    ZAccel = 2,
    XRot   = 3,
    YRot   = 4,
    ZRot   = 5,
};

struct RvcMessage {
    RvcType type;
    float value;
};

enum class TirePosition: uint8_t {
    FrontLeft  = 0,
    FrontRight = 1,
    RearLeft   = 2,
    RearRight  = 3,
};

struct TireRpmMessage {
    TirePosition position;
    float value;
};

struct TireTemperatureMessage {
    TirePosition position;
    int16_t inner;  /* 1 = 0.1C */
    int16_t outer;  /* 1 = 0.1C */
    int16_t core;   /* 1 = 0.1C */
};

enum class LapMessage {
    TrackFormed = 0,
    SectorOneTime = 1,
    SectorTwoTime = 2,
    SectorThreeTime = 3,
    LapTime = 4,
};

struct MotorTemperaturesOne {
    int16_t phase_a_temp;      /* 1 = 0.1C */
    int16_t phase_b_temp;      /* 1 = 0.1C */
    int16_t phase_c_temp;      /* 1 = 0.1C */
    int16_t gate_driver_temp;  /* 1 = 0.1C */
};

struct MotorTemperaturesTwo {
    int16_t control_board_temp;    /* 1 = 0.1C */
    int16_t rtd1;                  /* 1 = 0.1C */
    int16_t rtd2;                  /* 1 = 0.1C */
    int16_t rtd3;                  /* 1 = 0.1C */
};

struct MotorTemperaturesThree {
    int16_t coolant_temp;      /* 1 = 0.1C */
    int16_t hot_spot_temp;     /* 1 = 0.1C */
    int16_t motor_temp;        /* 1 = 0.1C */
    int16_t torque_shudder;    /* 1 = 0.1Nm */
};

struct MotorAnalogInputVoltages {
    int16_t analog_input_one;      /* 1 = 0.01V */
    int16_t analog_input_two;      /* 1 = 0.01V */
    int16_t analog_input_three;    /* 1 = 0.01V */
    int16_t analog_input_four;     /* 1 = 0.01V */
};

struct MotorDigitalInputStatus {
    bool forward_switch;
    bool reverse_switch;
    bool brake_switch;
    bool regen_disable_switch;
    bool ignition_switch;
    bool start_switch;
    bool valet_mode;
    bool digital_input_eight;
};

struct MotorPositionInfo {
    int16_t motor_angle;            /* 1 = 0.1° */
    int16_t motor_speed;            /* RPM */
    int16_t electrical_frequency;   /* 1 = 0.1Hz */
    int16_t delta_resolver_angle;   /* 1 = 0.1Hz */
};

struct MotorCurrentInfo {
    int16_t phase_a_current;    /* 1 = 0.1A */
    int16_t phase_b_current;    /* 1 = 0.1A */
    int16_t phase_c_current;    /* 1 = 0.1A */
    int16_t dc_bus_current;     /* 1 = 0.1A */
};

struct MotorVoltageInfo {
    int16_t dc_bus_voltage; /* 1 = 0.1V */
    int16_t output_voltage; /* 1 = 0.1V */
    int16_t vab_voltage;    /* 1 = 0.1V */
    int16_t vbc_voltage;    /* 1 = 0.1V */
};

struct MotorFluxInfo {
    int16_t flux_command;   /* mWb */
    int16_t flux_feedback;  /* mWb */
    int16_t id_feedback;    /* 1 = 0.1A */
    int16_t iq_feedback;    /* 1 = 0.1A */
};

struct MotorInternalVoltages {
    int16_t reference_1_5_v;    /* Reference 1.5V. 1 = 0.01V */
    int16_t reference_2_5_v;    /* Reference 2.5V. 1 = 0.01V */
    int16_t reference_5_0_v;    /* Reference 5.0V. 1 = 0.01V */
    int16_t reference_12_v;     /* Reference 12V. 1 = 0.01V */
};

enum class VsmState: uint8_t {
    VsmStart = 0,
    PreChargeInit = 1,
    PreChargeActive = 2,
    PreChargeComplete = 3,
    VsmWait = 4,
    VsmReady = 5,
    MotorRunning = 6,
    BlinkFaultCode = 7,
    ShutdownInProcess = 14,
    RecyclePower = 15,
};

enum class InverterState: uint8_t {
    PowerOn = 0,
    Stop = 1,
    OpenLoop = 2,
    ClosedLoop = 3,
    Wait = 4,
    _Internal1 = 5,
    _Internal2 = 6,
    _Internal3 = 7,
    IdleRun = 8,
    IdleStop = 9,
    _Internal4 = 10,
    _Internal5 = 11,
    _Internal6 = 12,
};

struct RelayState {
    bool relay_one;
    bool relay_two;
    bool relay_three;
    bool relay_four;
    bool relay_five;
    bool relay_six;
};

enum class InverterRunMode {
    TorqueMode = 0,
    SpeedMode = 1,
};

enum class InverterActiveDischargeState {
    DischargeDisabled = 0,
    DischargedEnabledAndWaiting = 1,
    PerformingSpeedCheck = 2,
    DischargeActivelyOccuring = 3,
    DischargeCompleted = 4,
};

enum class InverterCommandMode {
    CanMode = 0,
    VsmMode = 1,
};

enum class BurstModelMode {
    Stall = 0,
    HighSpeed = 1,
};

enum class MotorDirection {
    Reverse = 0,
    Forward = 1,
};

struct MotorInternalStates {
    VsmState vsm_state;
    uint8_t pwm_frequency; /* kHz */
    InverterState inverter_state;
    RelayState relay_state;
    InverterRunMode inverter_run_mode;
    bool self_sensing_assist_enable;
    InverterActiveDischargeState inverter_active_discharge_state;
    InverterCommandMode inverter_command_mode;
    uint8_t rolling_counter_value;
    bool inverter_enable_state; /* false = not enabled, true = enabled. */
    BurstModelMode burst_model_mode;
    bool start_mode_active; /* false = start signal not activated, true = activated. */
    bool inverter_enable_lockout; /* false = inverter can be enabled, true = can't be enabled. */
    MotorDirection direction_command;
    bool bms_active; /* false = BMS message not being received, true is being received. */
    bool bms_limiting_torque; /* false = torque not being limited by BMS, true = limited. */

    /* This value is currently available only in Gen 5/CM inverters and Gen 3 version 2042+:
     * false = no torque limiting is occurring. true = torque limiting is occurring due to
     * the motor speed exceeding the maximum motor speed.*/
    bool limit_max_speed;

    /* This value/function is currently available only in Gen 5/CM inverters:
     * false = Inverter hot spot temperature is below the limit. true = Inverter is limiting
     * current due to regulate the maximum hot spot temperature. */
    bool limit_hot_spot;

    /* This value is currently available only in Gen 5/CM inverters and Gen 3 version 2042+:
     * false = low speed current limiting is not occurring. true = low speed current
     * limiting is applied. */
    bool low_speed_limiting;

    /* This value is currently available only in Gen 5/CM inverters. The bit indicates that the
     * maximum motor current is being limited due to coolant temperature. */
    bool coolant_temperature_limiting;

    /* For Gen 5 / CM: false = Not Limiting, true = Limiting. Indicates if Stall Burst Model is
     * limiting the current. For select inverters only. See Stall Burst manual. */
    bool limit_stall_burst_model;
};

struct MotorFaultCodes {
    bool hardware_gate_or_desaturation_fault_first; /* For whatever reason the docs have two versions of this. */
    bool hardware_overcurrent_fault;
    bool accelerator_shorted;
    bool accelerator_open;
    bool current_sensor_low;
    bool current_sensor_high;
    bool module_temperature_low;
    bool module_temperature_high;

    bool control_pcb_temperature_low;
    bool control_pcb_temperature_high;
    bool gate_drive_pcb_temperature_low;
    bool gate_drive_pcb_temperature_high;
    bool sense_voltage_low_5v;
    bool sense_voltage_high_5v;
    bool sense_voltage_low_12v;
    bool sense_voltage_high_12v;

    bool sense_voltage_low_2_5v;
    bool sense_voltage_high_2_5v;
    bool sense_voltage_low_1_5v;
    bool sense_voltage_high_1_5v;
    bool dc_bus_voltage_high;
    bool dc_bus_voltage_low;
    bool pre_charge_timeout;
    bool pre_charge_voltage_failure;

    bool eeprom_checksum_invalid;
    bool eeprom_data_out_of_range;
    bool eeprom_update_required;
    bool hardware_dc_bus_over_voltage_during_initialization;
    bool gate_drive_initialization; /* Gen 3 has this value reserved, only this on Gen 5. */
    bool reserved_bit_29;
    bool brake_shorted;
    bool brake_open;

    bool motor_over_speed_fault;
    bool over_current_fault;
    bool over_voltage_fault;
    bool inverter_over_temperature_fault;
    bool accelerator_input_shorted_fault;
    bool accelerator_input_open_fault;
    bool direction_command_fault;
    bool inverter_response_timeout_fault;

    bool hardware_gate_or_desaturation_fault_second; /* For whatever reason the docs have two versions of this. */
    bool hardware_over_current_fault;
    bool under_voltage_fault;
    bool can_command_message_lost_fault;
    bool motor_over_temperature_fault;
    bool reserved_bit_45;
    bool reserved_bit_46;
    bool reserved_bit_47;

    bool brake_input_shorted_fault;
    bool brake_input_open_fault;
    bool module_a_over_temperature_fault;
    bool module_b_over_temperature_fault;
    bool module_c_over_temperature_fault;
    bool pcb_over_temperature_fault;
    bool gate_drive_board_one_over_temperature_fault;
    bool gate_drive_board_two_over_temperature_fault;

    bool gate_drive_board_three_over_temperature_fault;
    bool current_sensor_fault;
    bool gate_driver_over_voltage; /* Gen 3 has this value reserved, only this on Gen 5. */

    /* Gen _5_ has this value reserved, only this on Gen _3_. Note this was available on an older
     * version but is no longer availabe on the newer version. */
    bool hardware_dc_bus_over_voltage_fault_old;
    bool hardware_dc_bus_over_voltage_fault_new; /* Gen 3 has this value reserved, only this on Gen 5. */
    bool reserved_bit_61;
    bool resolver_not_connected;
    bool reserved_bit_63;
};

struct MotorTorqueAndTimerInfo {
    int16_t commanded_torque;
    int16_t torque_feedback;

    /* This timer is updated every 3 msec. This timer will roll-over in approximately 5
     * months. The timer will reset to 0 at power on. Monitoring this can be useful to show
     * when a reset of the processor has occurred.
     * 1 = 3ms = 0.003s */
    uint32_t power_on_timer;
};

/* Full title is Modulation Index & Flux Weakening Output Information */
struct MotorModIndexAndFlux {
    int16_t modulation_index; /* Assumed to be signed. */
    int16_t flux_weakening_output; /* This is the current output of the flux regulator. 1 = 0.1A */
    int16_t id_command; /* The commanded D-axis current. 1 = 0.1A */
    int16_t iq_command; /* The commanded Q-axis current. 1 = 0.1A */
};

struct MotorControlCommand {
    int16_t torque; /* 1 = 0.1Nm. */
    int16_t speed;  /* RPM */
    MotorDirection direction;
    bool enable_inverter;
    bool inverter_discharge; /* false = disable discharge, true = enable. */

    /* false = Do not over-ride mode. true = If controller is in torque mode then
     * controller will change to speed mode. This is a mode over-ride bit that will
     * change the mode from torque to speed only. It does not change the mode from
     * speed to torque. See manual Using Speed Mode for more information. */
    bool override_speed;

    /* If set to 0, the default torque limits sets in the EEPROM parameters are used. If
     * set to a positive number then the Motor and Regen Torque limits are set to the
     * torque value sent.*/
    int16_t torque_limit;
};

CAN_message_t empty_can_message(MessageId id, uint8_t len);
uint16_t read_u16_le(uint8_t* buf);
void write_u16_le(uint8_t* buf, uint16_t value);
uint32_t read_u32_le(const uint8_t* buf);
void write_u32_le(uint8_t* buf, uint32_t value);
float read_f32_le(uint8_t* buf);
void write_f32_le(uint8_t* buf, float value);

bool parse_start_switch(CAN_message_t msg);
CAN_message_t create_start_switch(bool value);
uint16_t parse_throttle_one_position(CAN_message_t msg);
CAN_message_t create_throttle_one_position(uint16_t value);
uint16_t parse_throttle_two_position(CAN_message_t msg);
CAN_message_t create_throttle_two_position(uint16_t value);
uint16_t parse_brake_pressure(CAN_message_t msg);
CAN_message_t create_brake_pressure(uint16_t value);
RvcMessage parse_rvc(CAN_message_t msg);
CAN_message_t create_rvc(RvcMessage value);
TireRpmMessage parse_tire_rpm(CAN_message_t msg);
CAN_message_t create_tire_rpm(TireRpmMessage value);
TireTemperatureMessage parse_tire_temperature(CAN_message_t msg);
CAN_message_t create_tire_temperature(TireTemperatureMessage value);
LapMessage parse_lap(CAN_message_t msg);
CAN_message_t create_lap(LapMessage value);

MotorTemperaturesOne parse_motor_temperatures_one(CAN_message_t msg);
MotorTemperaturesTwo parse_motor_temperatures_two(CAN_message_t msg);
MotorTemperaturesThree parse_motor_temperatures_three(CAN_message_t msg);
MotorAnalogInputVoltages parse_motor_analog_input_voltages(CAN_message_t msg);
MotorDigitalInputStatus parse_motor_digital_input_status(CAN_message_t msg);
MotorPositionInfo parse_motor_position_info(CAN_message_t msg);
MotorCurrentInfo parse_motor_current_info(CAN_message_t msg);
MotorVoltageInfo parse_motor_voltage_info(CAN_message_t msg);
MotorFluxInfo parse_motor_flux_info(CAN_message_t msg);
MotorInternalVoltages parse_motor_internal_voltages(CAN_message_t msg);
MotorInternalStates parse_motor_internal_states(CAN_message_t msg);
MotorFaultCodes parse_motor_fault_codes(CAN_message_t msg);
MotorTorqueAndTimerInfo parse_motor_torque_and_timer_info(CAN_message_t msg);
MotorModIndexAndFlux parse_motor_mod_index_and_flux(CAN_message_t msg);
int16_t parse_motor_torque_capability(CAN_message_t msg);
CAN_message_t create_motor_control_command(MotorControlCommand value);
MotorControlCommand parse_motor_control_command(CAN_message_t msg);
