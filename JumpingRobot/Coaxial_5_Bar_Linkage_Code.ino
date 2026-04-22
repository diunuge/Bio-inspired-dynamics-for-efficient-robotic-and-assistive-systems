#include "ODriveCAN.h"
#include <Arduino.h>
#include <Ramp.h>

// Ramp Objects
rampFloat rampX, rampY;

// code used to send a message (msg) to a specific ODrive
#define sendMessage                                                            \
  switch (nodeID) {                                                            \
  case 0:                                                                      \
    odrvA.send(msg);                                                           \
    break;                                                                     \
  case 1:                                                                      \
    odrvB.send(msg);                                                           \
    break;                                                                     \
  }
// sets the velocity of a specific ODrive while homing
#define setHomingVelocity                                                      \
  switch (nodeID) {                                                            \
  case 0:                                                                      \
    odrvA.setVelocity(vel);                                                    \
    break;                                                                     \
  case 1:                                                                      \
    odrvB.setVelocity(vel);                                                    \
    break;                                                                     \
  }
// defined states
#define IDLE ODriveAxisState::AXIS_STATE_IDLE
#define CLC ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL
#define CAN_BAUDRATE 1000000 // CAN bus baudrate

//--Setup for Teensy 4.1--
#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface
#ifdef IS_TEENSY_BUILTIN
#include "ODriveFlexCAN.hpp"
#include <FlexCAN_T4.h>
struct ODriveStatus; // hack to prevent teensy compile error
#endif               // IS_TEENSY_BUILTIN
#ifdef IS_TEENSY_BUILTIN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}
#endif // IS_TEENSY_BUILTIN

// Instantiate ODrive objects
ODriveCAN odrvA(wrap_can_intf(can_intf),
                0); // Standard CAN message ID (set CAN Node ID)
ODriveCAN odrvB(wrap_can_intf(can_intf),
                1); // Standard CAN message ID (set CAN Node ID)
ODriveCAN *odrives[] = {
    &odrvA, &odrvB}; // Make sure all ODriveCAN instances are accounted for here
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrvA_user_data;
ODriveUserData odrvB_user_data;

// actuation variables
float motorPos[2];          // position of the motor [counts]
float motorVel[2];          // velocity of the motor [counts/sec]
float reduction[2];         // gear reduction
float Iq[2];                // torque producing current [A]
float homingIq[2];          // positive current limit for homing [A]
float targetActuatorPos[2]; // target position of the actuator [counts]
float FSD = 120; //(Full Scale Deflection) actuator range of motion [degrees]

// Inverse Kinematic Variables
float c = 90;         // input parameter [mm]
float d = 110.0;      // input parameter [mm]
float thetaA, thetaB; // output motor positions [counts]
float homePos =
    40.509389228 /
    360.0; // middle position of the actuator (found in CAD) [counts]
static const int NUM_REPEATS = 1;

// distance sensor variables
const int sensorPin = A4;
const uint32_t DIST_INTERVAL_MS = 40;

uint32_t lastDistMs = 0;
int dist_adc = 0;
float dist_cm = 0.0f;

void setup() {
  Serial.begin(115200);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrvA.onFeedback(onFeedback, &odrvA_user_data);
  odrvA.onStatus(onHeartbeat, &odrvA_user_data);

  odrvB.onFeedback(onFeedback, &odrvB_user_data);
  odrvB.onStatus(onHeartbeat, &odrvB_user_data);

  // Configure and initialize the CAN bus interface
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true)
      ; // spin indefinitely
  }

  for (int i = 0; i < 2; i++) {
    setState(i, 0);  // set state to IDLE
    setAbsPos(i, 0); // set the absolute position to 0 at startup (prior to this
                     // the absolute position is nan)
    setLimits(i, 30, 20); //(velocity, current)
    setGains(i, 60, 0.167,
             0.32); //(position gain, veloocity gain, velocity integrator gain)
    homingIq[i] = 7;
  }
  homingSequence();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
    case 'c': // closed loop control
      for (int i = 0; i < 2; i++) {
        setState(i, 1);
      }
      break;

    case 'i': // idle
      for (int i = 0; i < 2; i++) {
        setState(i, 0);
      }
      break;

    case 'h': // homing
      homingSequence();
      break;

    case 'n': // bad line
      badLine(NUM_REPEATS);
      break;

    case 'm': // line
      line(NUM_REPEATS);
      break;

    case 'l': // bad square
      badSquare(NUM_REPEATS);
      break;

    case 's': // square
      square(NUM_REPEATS);
      break;

    case 'k': // square with cross
      squareCross(NUM_REPEATS);
      break;

    case 't': // triangle
      triangle(NUM_REPEATS);
      break;

    case 'u': // circle
      circle(NUM_REPEATS);
      break;

    case 'd': // diamond
      diamond(NUM_REPEATS);
      break;

    case 'q': // sinusoidal wave
      sineWave(NUM_REPEATS);
      break;

    case 'y': // square, circle, diamond sequence
      squareToDiamond(NUM_REPEATS);
      break;

    case 'j': // jump
      jump(NUM_REPEATS);
      break;

    case 'f': // jump frequency
      jumpFrequency();
      break;

    case 'r': // up down
      upDown(NUM_REPEATS);
      break;
    }
  }
}