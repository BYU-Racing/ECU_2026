#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> MotorCAN;
CAN_message_t rmsg;


int collect_throttle();
int collect_brake();
bool collect_button();

int throttle_map(int throttle1, int throttle2);
int map(int input, int min, int max);

int state_check(int torque, int brake_val, bool button_status);

void send_message(int system_status, int torque);

void user_update(int throttle1, int throttle2, int brake_val, bool button_status, int torque, int system_status);


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

const int MOTOR_COMMAND_ID = 192;
const int BUTTON_ID = 0;
const int THROTTLE1_ID = 1;
const int THROTTLE2_ID = 2;
const int BRAKE_ID = 3;
const int BRAKE_MIN = 10;


// INPUTS
int throttle1 = 0;
int throttle2 = 0;
int brake_val = 0;
int button_status = false;

int throttleA = 0;
int throttleB = 0;

// FUNCTION GENERATED
int torque;
int torque_percentage;
int system_status;

bool throttle1UPDATE;
bool throttle2UPDATE;

void setup() {
  Serial.begin(SERIAL_RATE);

  MotorCAN.begin();
  MotorCAN.setBaudRate(CAN_RATE);

  Serial.println("============================================");
  Serial.println("==========Motor CAN initialized=============");
  Serial.println("============================================");

}

void loop() {
  
  if (MotorCAN.read(rmsg)) { // If there is a CAN message recieved update the corresponding value
    switch (rmsg.id) {
      case BUTTON_ID:
        button_status = collect_button();
        break;
      case THROTTLE1_ID:
        torque = collect_throttle();
        break;
      case THROTTLE2_ID:
        torque = collect_throttle();
        break;
      case BRAKE_ID:
        brake_val = collect_brake();
        break;
      default:
        break;
    }
  }

  system_status = state_check(torque, brake_val, button_status);

  send_message(system_status, torque);

  user_update(throttle1, throttle2, brake_val, button_status, torque, system_status);
}

bool collect_button() {
  return rmsg.buf[0] != 0;
}

int collect_throttle() {
    if(rmsg.id == THROTTLE1_ID) {
        throttle1 = rmsg.buf[0];
        throttle1UPDATE = true;
    } else if (rmsg.id == THROTTLE2_ID) {
        throttle2 = rmsg.buf[0];
        throttle2UPDATE = true;
    }
    // if(!throttle1UPDATE || !throttle2UPDATE) { // exits if both haven't been updated
    //     return 0;
    // }

    if(throttle1UPDATE && throttle2UPDATE) {
    torque = throttle_map(throttle1, throttle2);
    throttle1UPDATE = false;
    throttle2UPDATE = false;
  }
  
  return torque;
}

int collect_brake() {
  brake_val = rmsg.buf[0];
  
  return brake_val;
}

int throttle_map(int throttle1, int throttle2) {
  
  throttleA = map(throttle1, THROTTLE1_MIN, THROTTLE1_MAX);
  throttleB = map(throttle2, THROTTLE2_MIN, THROTTLE2_MAX);
  
  torque_percentage = (throttleA + throttleB) / 2;

  torque = map(torque_percentage, MIN_THROTTLE, MAX_THROTTLE);

  if(torque < 0) {
      torque = 0;
  }

  return torque;
}

int map(int input, int min, int max) {
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

int state_check(int torque, int brake_val, bool button_status) {
  if (brake_val >= BRAKE_THRESHOLD) {
    return 0;
  } else if (button_status) {
    return 2;
  } else if (torque <= TORQUE_FLOOR) {
    return 1;
  } else {
    return 0;
  }
}

void send_message(int system_status, int torque) {
  if (system_status == 0) { // Keep alive mode
    rmsg.len=8;
    rmsg.buf[0]=0;
    rmsg.buf[1]=0;
    rmsg.buf[2]=0;
    rmsg.buf[3]=0;
    rmsg.buf[4]=0;
    rmsg.buf[5]=0;
    rmsg.buf[6]=0;
    rmsg.buf[7]=0;
    rmsg.id=MOTOR_COMMAND_ID;
    MotorCAN.write(rmsg);
    Serial.println("DISABLING INVERTER");
  } else if (system_status == 1) { // Inables the motor the prepares it to drive
    rmsg.len=8;
    rmsg.buf[0]=0;
    rmsg.buf[1]=0;
    rmsg.buf[2]=0;
    rmsg.buf[3]=0;
    rmsg.buf[4]=0;
    rmsg.buf[5]=1; // Ensures the motor controller is active
    rmsg.buf[6]=0;
    rmsg.buf[7]=0;
    rmsg.id=MOTOR_COMMAND_ID;
    MotorCAN.write(rmsg);
    Serial.println("ENABLING INVERTER");
  } else if (system_status == 2) { // Sends a valid torque command based on torque input value
    rmsg.len=8;
    rmsg.buf[0]=torque % 256;
    rmsg.buf[1]=torque / 256;
    rmsg.buf[2]=0;
    rmsg.buf[3]=0;
    rmsg.buf[4]=0;
    rmsg.buf[5]=1; // Ensures the motor controller is still active
    rmsg.buf[6]=0;
    rmsg.buf[7]=0;
    rmsg.id=MOTOR_COMMAND_ID;
    MotorCAN.write(rmsg);
    Serial.println("SENDING TORQUE COMMAND");
  } else { // If anything else gets passed in, sends the keep alive but disable message
    rmsg.len=8;
    rmsg.buf[0]=0;
    rmsg.buf[1]=0;
    rmsg.buf[2]=0;
    rmsg.buf[3]=0;
    rmsg.buf[4]=0;
    rmsg.buf[5]=0;
    rmsg.buf[6]=0;
    rmsg.buf[7]=0;
    rmsg.id=MOTOR_COMMAND_ID;
    MotorCAN.write(rmsg);
  }
}

void user_update(int throttle1,
                 int throttle2,
                 int brake_val,
                 bool button_status,
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

    Serial.print("Button       : ");
    Serial.println(button_status);

    Serial.print("Torque Cmd   : ");
    Serial.println(torque);

    Serial.print("System State : ");
    Serial.println(system_status);

    Serial.println("==================================");
}

