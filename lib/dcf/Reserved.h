#ifndef RESERVED_H
#define RESERVED_H

#include <cstdint>

/** CAN Message IDs that are reserved */
enum ReservedIDs : uint32_t
{
    // Custom Sensor Messages

    StartSwitchId, Throttle1PositionId, Throttle2PositionId, BrakePressureId, RVCId, TireRPMId, TireTemperatureId, BMSPercentageId,
    BMSTemperatureId, SteeringWheelAngleId,
    // Motor Messages

    Temperatures1Id=0x0A0, Temperatures2Id, Temperatures3Id, AnalogInputVoltagesId, DigitalInputStatusId, MotorPositionInfoId,
    CurrentInfoId, VoltageInfoId, FluxInfoId, InternalVoltagesId, InternalStatesId, FaultCodesId, TorqueAndTimerInfoId,
    ModulationIndexId, FirmwareInformationId, DiagnosticDataId, HighSpeedId, TorqueCapabilityId,
    // Motor Commands/Response Messages

    ControlCommandId=0x0C0, ParameterCommandId, ParameterResponseId,
    // Health Check Commands/Response Messages

    HealthCheckId=200, DCFId, DCRId, DCTId,
    // Other Commands/Response Messages
    FaultId, DriveStateId, DriveModeId, ThrottleMinId, ThrottleMaxId,

    // ID for default initializations.
    INVALIDId=0xFFFFFFFF,
};

enum FaultSourcesIDs : uint8_t
{
    ThrottleMismatchId,
    Throttle1ZeroId,
    Throttle2ZeroId,
    BrakeZeroId,
    StartFaultId,
};

enum DriveModesIDs : uint8_t
{
    FullId,
    EnduranceId,
    SkidPadId,
    PartyId,
    LimpId,
};

enum RVCSubIDs : uint8_t
{
    X_AccelId,
    Y_AccelId,
    Z_AccelId,
    RollId,
    PitchId,
    YawId,
};

enum TireSubIDs : uint8_t
{
    FrontLeftId,
    FrontRightId,
    RearLeftId,
    RearRightId,
};

#endif //RESERVED_H
