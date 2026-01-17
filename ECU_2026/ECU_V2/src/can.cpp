#include <cstdint>
#include <bit>

#include <FlexCAN_T4.h>

#include "util.hpp"

/* All the CAN messages with their IDs. */

enum MessageId : uint8_t {
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
    MotorPositionInfo = 165,
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
};

/* u = unsigned int, 16 = 16 bits, le = little endian */
static uint16_t read_u16_le(uint8_t* buf) {
    return buf[0] << 8 | buf[1];
}

static uint32_t read_u32_le(uint8_t* buf) {
    return buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
}

static float read_f32_le(uint8_t* buf) {
    uint32_t as_u32 = read_u32_le(buf);
    float as_f32;
    // The only way to convert from int to float per-bit is to do a memcpy.
    memcpy(&as_f32, &as_u32, sizeof(as_f32));
    return as_f32;
}

bool parse_start_switch(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::StartSwitch);
    return msg.buf[0] != 0;
}

uint16_t parse_throttle_one_position(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::ThrottleOnePosition);
    return read_u16_le(msg.buf);
}

uint16_t parse_throttle_two_position(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::ThrottleTwoPosition);
    return read_u16_le(msg.buf);
}

uint16_t parse_throttle_brake_pressure(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::BrakePressure);
    return read_u16_le(msg.buf);
}


enum RvcType: uint8_t {
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

RvcMessage parse_rvc(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::Rvc);
    // Make sure the rcv type is one of the six valid types.
    uint8_t rvc_type = msg.buf[0];
    SAFETY_ASSERT(rvc_type < 6);

    float value = read_f32_le(&msg.buf[1]);

    return (RvcMessage) { (RvcType)rvc_type, value };
}


enum TirePosition: uint8_t {
    FrontLeft  = 0,
    FrontRight = 1,
    RearLeft   = 2,
    RearRight  = 3,
};

struct TireRpmMessage {
    TirePosition position;
    float value;
};

TireRpmMessage parse_tire_rpm(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::TireRpm);
    // Make sure the tire position is one of the valid positions.
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position < 4);

    float value = read_f32_le(&msg.buf[1]);

    return (TireRpmMessage) { (TirePosition)tire_position, value };
}


struct TireTemperatureMessage {
    TirePosition position;
    uint16_t value;
};

TireTemperatureMessage parse_tire_temperature(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::TireTemperature);
    // Make sure the tire position is one of the valid positions.
    uint8_t tire_position = msg.buf[0];
    SAFETY_ASSERT(tire_position < 4);

    uint16_t value = read_u16_le(&msg.buf[1]);

    return (TireTemperatureMessage) { (TirePosition)tire_position, value};
}


enum LapMessage {
    TrackFormed = 0,
    SectorOneTime = 1,
    SectorTwoTime = 2,
    SectorThreeTime = 3,
    LapTime = 4,
};

LapMessage parse_lap_message(CAN_message_t msg) {
    SAFETY_ASSERT(msg.id == MessageId::Lap);
    // Make sure the lap message is one of the valid types.
    uint8_t lap_type = msg.buf[0];
    SAFETY_ASSERT(lap_type < 5);

    return (LapMessage)lap_type;
}
