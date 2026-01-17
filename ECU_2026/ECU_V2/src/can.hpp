#include <cstdint>

#include "FlexCAN_T4.h"

/* All the CAN messages with their IDs. */
enum class MessageId : uint8_t {
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
    ModulationIndex = 173,
    /* For factory use only - not for CAN use. */
    FirmwareInfo = 174,
    /* Downloadable only - not for CAN use. */
    DiagnosticData = 175,
    /* Torque Command, Torque Feedback, Motor Speed, DC Bus Voltage. */
    HighSpeed = 176,
    TorqueCapability = 177,
    /* Response to ParameterCommand. */
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

enum class DirectionCommand {
    Forward, Reverse, Stopped,
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
    DirectionCommand direction_command;
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

CAN_message_t empty_can_message(MessageId id);
uint16_t read_u16_le(uint8_t* buf);
void write_u16_le(uint8_t* buf, uint16_t value);
uint32_t read_u32_le(uint8_t* buf);
void write_u32_le(uint8_t* buf, uint32_t value);
float read_f32_le(uint8_t* buf);
void write_f32_le(uint8_t* buf, float value);

bool parse_start_switch(CAN_message_t msg);
CAN_message_t create_start_switch(bool value);
uint16_t parse_throttle_one_position(CAN_message_t msg);
CAN_message_t create_throttle_one_position(uint16_t value);
uint16_t parse_throttle_two_position(CAN_message_t msg);
CAN_message_t create_throttle_two_position(uint16_t value);
uint16_t parse_throttle_brake_pressure(CAN_message_t msg);
CAN_message_t create_throttle_brake_pressure(uint16_t value);
RvcMessage parse_rvc(CAN_message_t msg);
CAN_message_t create_rvc(RvcMessage value);
TireRpmMessage parse_tire_rpm(CAN_message_t msg);
CAN_message_t create_tire_rpm(TireRpmMessage value);
TireTemperatureMessage parse_tire_temperature(CAN_message_t msg);
CAN_message_t create_tire_temperature(TireTemperatureMessage value);
LapMessage parse_lap(CAN_message_t msg);
CAN_message_t create_lap(LapMessage value);

/* You can ignore everything in-between template<>, this is just so we can work with the FlexCAN library. */
template<CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize, FLEXCAN_TXQUEUE_TABLE _txSize>
class CanController {
    private:
        /* This is the can bus we'll be using for sending and receiving messages. */
        FlexCAN_T4<_bus, _rxSize, _txSize> can;

    public:
        void init(FlexCAN_T4<_bus, _rxSize, _txSize> can_to_use, uint32_t baud_rate) {
        can = can_to_use;

        can.begin();
        can.setBaudRate(baud_rate);
    }

    CAN_message_t poll_message() {
        /* Read the message into a temporary variable, then return the message
            * on the stack so it's easier to work with. */
        CAN_message_t msg;
        can.read(&msg);
        return msg;
    }

    void send_start_switch(bool value) {
        can.write(create_start_switch(value));
    }

    void send_throttle_one_position(uint16_t value) {
        can.write(create_throttle_one_position(value));
    }

    void send_throttle_two_position(uint16_t value) {
        can.write(create_throttle_two_position(value));
    }

    void send_throttle_break_pressure(uint16_t value) {
        can.write(create_throttle_brake_pressure(value));
    }

    void send_rvc(RvcMessage value) {
        can.write(create_rvc(value));
    }

    void send_tire_rpm(TireRpmMessage value) {
        can.write(create_tire_rpm(value));
    }

    void send_tire_temperature(TireTemperatureMessage value) {
        can.write(create_tire_temperature(value));
    }

    void send_lap(LapMessage value) {
        can.write(create_lap(value));
    }
};
