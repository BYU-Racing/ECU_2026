#include <cmath>
#include <optional>

#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "assert.hpp"
#include "can_serde.hpp"

using namespace std;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> MotorCAN;

int collect_throttle(CAN_message_t rmsg);
bool collect_switch(CAN_message_t rmsg);
int brake_shutoff(int brake_val);

int throttle_map(int throttle1, int throttle2);
int32_t map(int32_t input, int32_t min, int32_t max);

int state_check(int torque, int brake_val, bool switch_status);

void handle_activation(int brake_val, bool switch_status);

void send_message(int system_status, int torque);

void user_update(int throttle1, int throttle2, int brake_val, bool switch_status, int torque, int system_status);

void panic_handler(const char* file, int line, const char* msg) {
    /* Shut everything down. */
    CAN_message_t shutdown_message = empty_can_message(MessageId::ControlCommand, 8);
    /* `empty_can_message` is guaranteed to generate a message full of zeroes,
     * and since this is panic wind down code, it's best to keep it as simple as
     * possible, so we don't use the normal message creation function. */
    MotorCAN.write(shutdown_message);

    /* Loop forever so we never do anything after the panic. */
    while (true) {
        Serial.printf("Assertion failed! Line %d in file %s with message %s\n", line, file, msg);
        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        delay(500);
    }
}

// CONSTANTS
const int SERIAL_RATE = 115200;
const int CAN_RATE = 250000;
const int MIN_THROTTLE = 0;
const int MAX_THROTTLE = 100; // In CASCADIA format (x10 Nm)
const int BRAKE_THRESHOLD = 100;
const int TORQUE_FLOOR = 10;

const int THROTTLE1_MIN = 0;
const int THROTTLE1_MAX = 160;
const int THROTTLE2_MIN = 0;
const int THROTTLE2_MAX = 70;

/* Maximum allowed difference between the two throttles in %. */
const uint16_t THROTTLE_DISAGREE = 10;

const int MOTOR_COMMAND_ID = 192;
const int switch_ID = 0;
const int THROTTLE1_ID = 1;
const int THROTTLE2_ID = 2;
const int BRAKE_ID = 3;
const int BRAKE_MIN = 10;

const int MAX_BRAKE_VALUE = 1023;
const int BRAKE_SAFETY_THRESHOLD = MAX_BRAKE_VALUE * .1;  // 10 percent of the max brake value
const int TORQUE_SAFETY_THRESHOLD = 10;  // 10 percent of max torque


// INPUTS
int throttle1 = 0;
int throttle2 = 0;
int brake_val = 0;
int switch_status = false;

// FUNCTION GENERATED
int system_status;

bool throttle1UPDATE;
bool throttle2UPDATE;

bool brake_fault = false;
bool throttle_fault = false;

// MOTOR ACTIVATED
const int ACTIVATE_BRAKE_THRESHOLD = 20;  // percent
const unsigned long DELAY_MS = 2000;  // 2 seconds
bool requested = false;
bool motor_enabled = false;
unsigned long start_time = 0;

void setup() {
  // First things first, register the panic handler. If something goes
    // wrong during setup, we'll wind everything down.
  register_panic_handler(panic_handler);
  Serial.begin(SERIAL_RATE);

  MotorCAN.begin();
  MotorCAN.setBaudRate(CAN_RATE);

  Serial.println("============================================");
  Serial.println("==========Motor CAN initialized=============");
  Serial.println("============================================");

}

int16_t calculated_torque = 0;

void loop() {
    delay(15);

    bool start_switch_on = false;

    CAN_message_t rmsg;
    if (MotorCAN.read(rmsg)) { 
      /* If we received a CAN message, update the corresponding value. */
      switch (static_cast<MessageId>(rmsg.id)) {
          case MessageId::StartSwitch:
            start_switch_on = parse_start_switch(rmsg);
            break;
          case MessageId::ThrottleOnePosition:
            throttle1_pos = parse_throttle_one_position(rmsg);
            break;
          case MessageId::ThrottleTwoPosition:
            throttle2_pos = parse_throttle_two_position(rmsg);
            break;
          case MessageId::BrakePressure:
            brake_val = parse_brake_pressure(rmsg);
            break;
          default:
            break;
      }
    }

    if (throttle1_pos.has_value() && throttle2_pos.has_value()) {
      calculated_torque = throttle_map(*throttle1_pos, *throttle2_pos);
      throttle1_pos = std::nullopt;
      throttle2_pos = std::nullopt;
    }

    handle_activation(brake_val, switch_status);

    if (brake_shutoff(brake_val)) {
      brake_fault = true;
    }

    if (brake_fault) {
      torque = 0;
      system_status = 0;
      send_message(system_status, torque);
      return;
    }

    SAFETY_ASSERT(!brake_fault);
    SAFETY_ASSERT(!throttle_fault);


    system_status = state_check(torque, brake_val, switch_status);

    send_message(system_status, torque);

    user_update(throttle1, throttle2, brake_val, switch_status, torque, system_status);
}

int brake_shutoff(int brake_val) {
    if (brake_val < BRAKE_MIN) {
        return true;
    }
    return false;
}

int16_t throttle_map(uint16_t throttle1, uint16_t throttle2) {
    int32_t throttleA = map(static_cast<int32_t>(throttle1), THROTTLE1_MIN, THROTTLE1_MAX);
    int32_t throttleB = map(static_cast<int32_t>(throttle2), THROTTLE2_MIN, THROTTLE2_MAX);

    SAFETY_ASSERT(abs(throttleA - throttleB) < THROTTLE_DISAGREE);

    int32_t average = (throttleA + throttleB) / 2;

    /* Make sure the average can fit into the new size (int16_t). */
    SAFETY_ASSERT(average >= INT16_MIN && average <= INT16_MAX);
    int16_t torque_percentage = static_cast<int16_t>(average);

    int16_t torque_mapped = map(torque_percentage, MIN_THROTTLE, MAX_THROTTLE);
    SAFETY_ASSERT(torque_mapped >= 0);

    return torque_mapped;
}

void handle_activation(int brake_val, bool switch_status) {
  if (motor_enabled) return;

  if ((brake_val >= ACTIVATE_BRAKE_THRESHOLD) && switch_status) {
    if (!requested) {
      requested = true;
      start_time = millis();
      Serial.println("MOTOR ENABLE REQUESTED");
    }

    if (millis() - start_time >= DELAY_MS) {
      motor_enabled = true;
      requested = false;
      Serial.println("MOTOR ENABLE");
    }
  }

  else {
    requested = false;
  }
}

/* FIXME this is not overflow safe. */
int32_t map(int32_t input, int32_t min, int32_t max) {
  if (input < min) {
    input = min;
  } if (input > max) {
    input = max;
  }

  if (max == min) {
    return 0;
  }

  return ((input - min) * 100) / (max - min);
}

int state_check(int torque, int brake_val, bool switch_status) {
  if (brake_val > BRAKE_SAFETY_THRESHOLD && torque_percentage > TORQUE_SAFETY_THRESHOLD) {
    Serial.println("SAFETY SHUTDOWN: Brake and Throttle applied simultaneously");
    return 0;
  }
  
  if (brake_fault || throttle_fault) {
    return 0;
  }

  if (!motor_enabled) {
    return 0;
  }

  if (!switch_status) {
    return 0;
  }

  if (brake_val >= BRAKE_THRESHOLD) {
    return 0;
  } 
  
  else if (switch_status) {
    return 2;
  } 
  
  else if (torque <= TORQUE_FLOOR) {
    return 1;
  } 
  
  else {
    return 0;
  }
}

void send_message(int system_status, int torque) {
  if (system_status == 0) { // Keep alive mode
    MotorControlCommand cmd;
    cmd.torque = 0;
    cmd.speed = 0;
    cmd.direction = MotorDirection::Forward;
    cmd.enable_inverter = false;
    cmd.inverter_discharge = false;
    cmd.override_speed = false;
    cmd.torque_limit = 0;
    MotorCAN.write(create_motor_control_command(cmd));
    Serial.println("DISABLING INVERTER");
  } else if (system_status == 1) { // Inables the motor the prepares it to drive
    MotorControlCommand cmd;
    cmd.torque = 0;
    cmd.speed = 0;
    cmd.direction = MotorDirection::Forward;
    cmd.enable_inverter = true;
    cmd.inverter_discharge = false;
    cmd.override_speed = false;
    cmd.torque_limit = 0;
    MotorCAN.write(create_motor_control_command(cmd));
    Serial.println("ENABLING INVERTER");
  } else if (system_status == 2) { // Sends a valid torque command based on torque input value
    MotorControlCommand cmd;
    cmd.torque = torque;
    cmd.speed = 0;
    cmd.direction = MotorDirection::Forward;
    cmd.enable_inverter = true;
    cmd.inverter_discharge = false;
    cmd.override_speed = false;
    cmd.torque_limit = 0;
    MotorCAN.write(create_motor_control_command(cmd));
    Serial.println("SENDING TORQUE COMMAND");
  } else { // If anything else gets passed in, sends the keep alive but disable message
    MotorControlCommand cmd;
    cmd.torque = 0;
    cmd.speed = 0;
    cmd.direction = MotorDirection::Forward;
    cmd.enable_inverter = false;
    cmd.inverter_discharge = false;
    cmd.override_speed = false;
    cmd.torque_limit = 0;
    MotorCAN.write(create_motor_control_command(cmd));
  }
}

void user_update(int throttle1,
                 int throttle2,
                 int brake_val,
                 bool switch_status,
                 int torque,
                 int system_status)
{
    Serial.println();
    Serial.println("========== SYSTEM STATUS ==========");

    Serial.print("Throttle 1   : ");
    Serial.println(throttle1);

    Serial.print("Throttle 2   : ");
    Serial.println(throttle2);

    Serial.print("Brake        : ");
    Serial.println(brake_val);

    Serial.print("Switch       : ");
    Serial.println(switch_status);

    Serial.print("Torque Cmd   : ");
    Serial.println(torque);

    Serial.print("System State : ");
    Serial.println(system_status);

    Serial.println("==================================");
}
