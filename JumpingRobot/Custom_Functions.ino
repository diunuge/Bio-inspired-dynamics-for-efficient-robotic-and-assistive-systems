// set the state of the odrives (state == 0, IDLE) (state == 1, CLOSED LOOP
// CONTROL)

// Distance sensor variables
extern const int sensorPin;
extern const uint32_t DIST_INTERVAL_MS;

extern uint32_t lastDistMs;
extern int dist_adc;
extern float dist_cm;

void setState(int nodeID, int state) {
  int actualState;
  do {
    switch (nodeID) {
    case 0:
      odrvA.clearErrors();
      break;
    case 1:
      odrvB.clearErrors();
      break;
    }
    switch (nodeID) {
    case 0:
      odrvA.setState(state ? CLC : IDLE);
      break;
    case 1:
      odrvB.setState(state ? CLC : IDLE);
      break;
    }
    switch (nodeID) {
    case 0:
      actualState = odrvA_user_data.last_heartbeat.Axis_State;
      break;
    case 1:
      actualState = odrvB_user_data.last_heartbeat.Axis_State;
      break;
    }
  } while (actualState != (state ? CLC : IDLE));
}
// set the gains: position gain, velocity gain, and velocity integrator gain
void setGains(int nodeID, float posGain, float velGain,
              float velIntegratorGain) {
  if (1) { // position gain
    const Set_Pos_Gain_msg_t msg = {
        .Pos_Gain = posGain,
    };
    sendMessage
  }
  delay(5);
  if (1) { // velocity gains
    const Set_Vel_Gains_msg_t msg = {
        .Vel_Gain = velGain,
        .Vel_Integrator_Gain = velIntegratorGain,
    };
    sendMessage
  }
  delay(5);
}
// set limits: velocity and current
void setLimits(int nodeID, float velLimit, float currentLimit) {
  const Set_Limits_msg_t msg = {
      .Velocity_Limit = velLimit,
      .Current_Limit = currentLimit,
  };
  sendMessage delay(5);
}
// set trapezoidal trajectory limits: velocity, acceleration, and deceleration,
// and inertia value
void setTrapTraj(int nodeID, float velLimit, float accelLimit, float decelLimit,
                 float inertia) {
  if (1) { // velocity limit
    const Set_Traj_Vel_Limit_msg_t msg = {
        .Traj_Vel_Limit = velLimit,
    };
    sendMessage
  }
  delay(5);
  if (1) { // accel limits
    const Set_Traj_Accel_Limits_msg_t msg = {
        .Traj_Accel_Limit = accelLimit,
        .Traj_Decel_Limit = decelLimit,
    };
    sendMessage
  }
  delay(5);
  if (1) { // inertia
    const Set_Traj_Inertia_msg_t msg = {
        .Traj_Inertia = inertia,
    };
    sendMessage
  }
  delay(5);
}
// set the absolute position of the encoder
void setAbsPos(int nodeID, float pos) {
  const Set_Absolute_Position_msg_t msg = {
      .Position = pos,
  };
  sendMessage delay(5);
}
// defines global position and velocity variables
void getEncoderValues() {
  pumpEvents(can_intf); // handles incoming feedback CAN messages
  if (odrvA_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrvA_user_data.last_feedback;
    odrvA_user_data.received_feedback = false;
    motorPos[0] = feedback.Pos_Estimate;
    motorVel[0] = feedback.Vel_Estimate;
  }
  if (odrvB_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrvB_user_data.last_feedback;
    odrvB_user_data.received_feedback = false;
    motorPos[1] = feedback.Pos_Estimate;
    motorVel[1] = feedback.Vel_Estimate;
  }
}
float motorTurnsToActuatorDeg(int nodeID, float motorTurns) {
  return reduction[nodeID] == 0.0f ? 0.0f
                                   : (motorTurns * 360.0f) / reduction[nodeID];
}
float motorTurnsPerSecToActuatorDegPerSec(int nodeID, float motorTurnsPerSec) {
  return reduction[nodeID] == 0.0f ? 0.0f
                                   : (motorTurnsPerSec * 360.0f) /
                                         reduction[nodeID];
}
void beginJumpAngleLog(int rep) {
  jumpLogActive = true;
  jumpLogRep = rep;
  jumpLogCount = 0;
  jumpLogOverflowed = false;
  jumpLogStartMs = millis();
  lastJumpLogMs = 0;
}
void sampleJumpAngleLog() {
  if (!jumpLogActive) {
    return;
  }

  uint32_t now = millis();
  if (jumpLogCount > 0 && now - lastJumpLogMs < JUMP_LOG_INTERVAL_MS) {
    return;
  }

  if (jumpLogCount >= JUMP_LOG_CAPACITY) {
    jumpLogOverflowed = true;
    return;
  }

  JumpAngleSample &sample = jumpLog[jumpLogCount++];
  sample.t_ms = now - jumpLogStartMs;
  sample.rep = jumpLogRep;
  sample.motor0_deg = motorTurnsToActuatorDeg(0, motorPos[0]);
  sample.motor1_deg = motorTurnsToActuatorDeg(1, motorPos[1]);
  sample.motor0_vel_dps = motorTurnsPerSecToActuatorDegPerSec(0, motorVel[0]);
  sample.motor1_vel_dps = motorTurnsPerSecToActuatorDegPerSec(1, motorVel[1]);
  sample.target0_deg = targetActuatorPos[0] * 360.0f;
  sample.target1_deg = targetActuatorPos[1] * 360.0f;
  lastJumpLogMs = now;
}
void endJumpAngleLog() {
  if (!jumpLogActive) {
    return;
  }

  jumpLogActive = false;

  Serial.print("#ANGLE_BEGIN ");
  Serial.print(jumpLogRep);
  Serial.print(" ");
  Serial.println(jumpLogCount);

  for (uint16_t i = 0; i < jumpLogCount; i++) {
    const JumpAngleSample &sample = jumpLog[i];
    Serial.print("#ANGLE ");
    Serial.print(sample.rep);
    Serial.print(" ");
    Serial.print(sample.t_ms);
    Serial.print(" ");
    Serial.print(sample.motor0_deg, 4);
    Serial.print(" ");
    Serial.print(sample.motor1_deg, 4);
    Serial.print(" ");
    Serial.print(sample.motor0_vel_dps, 4);
    Serial.print(" ");
    Serial.print(sample.motor1_vel_dps, 4);
    Serial.print(" ");
    Serial.print(sample.target0_deg, 4);
    Serial.print(" ");
    Serial.println(sample.target1_deg, 4);
  }

  if (jumpLogOverflowed) {
    Serial.print("#ANGLE_OVERFLOW ");
    Serial.print(jumpLogRep);
    Serial.print(" ");
    Serial.println(jumpLogCount);
  }

  Serial.print("#ANGLE_END ");
  Serial.println(jumpLogRep);
}
// get torque current
void getIq() {
  Get_Iq_msg_t data;
  if (!odrvA.request(data, 1)) {
    Serial.println("Iq request failed!");
    while (true)
      ; // spin indefinitely
  }
  Iq[0] = data.Iq_Measured;

  if (!odrvB.request(data, 1)) {
    Serial.println("Iq request failed!");
    while (true)
      ; // spin indefinitely
  }
  Iq[1] = data.Iq_Measured;
}
// homes the actuator by moving to one endstops
void oneStopHoming(int nodeID, int dir, float setReduction, float setMidPos) {
  float pos;                        // position of the endstop
  float midPos;                     // position of the middle of the actuator
  int vel = 2 * dir;                // velocity
  reduction[nodeID] = setReduction; // predefined reduction ratio

  // set control mode to velocity control
  switch (nodeID) {
  case 0:
    odrvA.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL,
                            INPUT_MODE_PASSTHROUGH);
    break;
  case 1:
    odrvB.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL,
                            INPUT_MODE_PASSTHROUGH);
    break;
  }
  delay(5);
  // spin to endstop
  do {
    setHomingVelocity delay(1);
    getIq();
  } while (dir == 1 ? Iq[nodeID] > -1 * homingIq[nodeID]
                    : Iq[nodeID] < homingIq[nodeID]);
  getEncoderValues();
  pos = motorPos[nodeID];
  vel = 0;
  setHomingVelocity

      midPos = (pos * dir - reduction[nodeID] * (FSD / 2) / 360) /
               reduction[nodeID]; // define middle position

  // set input mode to trap traj
  switch (nodeID) {
  case 0:
    odrvA.setControllerMode(CONTROL_MODE_POSITION_CONTROL,
                            INPUT_MODE_TRAP_TRAJ);
    break;
  case 1:
    odrvB.setControllerMode(CONTROL_MODE_POSITION_CONTROL,
                            INPUT_MODE_TRAP_TRAJ);
    break;
  }
  delay(5);

  setTrapTraj(nodeID, 5, 5, 5, 0);
  for (int i = 0; i < 2; i++) {
    targetActuatorPos[i] = i == nodeID ? midPos : 4004;
  }
  trapTraj(); // move to middle position
  delay(10);
  setAbsPos(nodeID,
            setMidPos *
                reduction[nodeID]); // set the position of the middle position
}
// homes the actuator by moving to two endstops
void twoStopHoming(int nodeID, float setMidPos) {
  float pos1, pos2; // position of the first and second endstop
  int vel = 2;      // velocity
  float midPos;     // positon of the middle of the actuator

  // set control mode to velocity control
  switch (nodeID) {
  case 0:
    odrvA.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL,
                            INPUT_MODE_PASSTHROUGH);
    break;
  case 1:
    odrvB.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL,
                            INPUT_MODE_PASSTHROUGH);
    break;
  }
  delay(5);

  // spin to first endstop
  do {
    setHomingVelocity delay(1);
    getIq();
  } while (Iq[nodeID] > homingIq[nodeID] * -1);
  getEncoderValues();
  pos1 = motorPos[nodeID];

  // spin to second endstop
  vel = -2;
  do {
    setHomingVelocity delay(1);
    getIq();
  } while (Iq[nodeID] < homingIq[nodeID]);
  getEncoderValues();
  pos2 = motorPos[nodeID];
  vel = 0;
  setHomingVelocity

      reduction[nodeID] = (pos1 - pos2) * 360 / FSD;
  midPos = (pos2 + (pos1 - pos2) / 2) / reduction[nodeID];
  // set input mode to trap traj
  switch (nodeID) {
  case 0:
    odrvA.setControllerMode(CONTROL_MODE_POSITION_CONTROL,
                            INPUT_MODE_TRAP_TRAJ);
    break;
  case 1:
    odrvB.setControllerMode(CONTROL_MODE_POSITION_CONTROL,
                            INPUT_MODE_TRAP_TRAJ);
    break;
  }
  delay(5);

  setTrapTraj(nodeID, 5, 5, 5, 0);
  for (int i = 0; i < 2; i++) {
    targetActuatorPos[i] = i == nodeID ? midPos : 4004;
  }
  trapTraj(); // move to the middle position (absolute zero)
  delay(10);
  setAbsPos(nodeID,
            setMidPos * reduction[nodeID]); // set the absolute position of the
                                            // middle position to 0

  // Serial.print((String) "Calculated Reduction " + nodeID + ": ");
  // Serial.println(reduction[nodeID], 6);
}
// homing sequence for the system
void homingSequence() {
  setState(0, 0);
  setState(1, 1);
  twoStopHoming(1, homePos); // (nodeID, setMidPos)
  setState(0, 1);
  setState(1, 0);
  oneStopHoming(0, 1, reduction[1],
                homePos); // (nodeID, dir, setReduction, setMidPos)
  setState(1, 1);
  moveEndEffectorSimple(0, 160); // move to home position (0, 160)
}
// move actuator to a defined position using trapezoidal movement
void trapTraj() {
  float targetMotorPos; // targetActuatorPos * reduction
  int check;            // variable used to determine whether to exit the loop
  do {
    delay(2);
    getEncoderValues();
    check = 0;
    for (int i = 0; i < 2; i++) {
      if (targetActuatorPos[i] != 4004) {
        targetMotorPos = targetActuatorPos[i] * reduction[i];
        switch (i) {
        case 0:
          odrvA.setPosition(targetMotorPos);
          break;
        case 1:
          odrvB.setPosition(targetMotorPos);
          break;
        }
      }
      // add 1 to check if the motor is not being moved or if error is less than
      // 0.01 counts
      check += targetActuatorPos[i] == 4004
                   ? 1
                   : (abs(motorPos[i] - targetMotorPos) < 0.01 ? 1 : 0);
    }
  } while (check != 2); // exit loop when check is equal to 2
}
// use the inverse kinematic equations to calculate the output motor positions
void getInverseKinematics(float x, float y) {
  thetaA =
      (acos((sq(x) + sq(y) + sq(c) - sq(d)) / (2 * c * sqrt(sq(x) + sq(y)))) +
       atan(x / y)) *
      180 / PI / 360.0;
  thetaB =
      (acos((sq(x) + sq(y) + sq(c) - sq(d)) / (2 * c * sqrt(sq(x) + sq(y)))) -
       atan(x / y)) *
      180 / PI / 360.0;
}
// move the end effector without any interpolation
void moveEndEffectorSimple(float x, float y) {
  getInverseKinematics(x, y);
  targetActuatorPos[0] = 2 * homePos - thetaA;
  targetActuatorPos[1] = thetaB;
  trapTraj();
  // set ramp values
  rampX.go(x);
  rampY.go(y);
}
// move the end effector with linear interpolation
void moveEndEffectorInterpolate(float x, float y, int type, float time) {
  // set target values
  switch (type) {
  case 0: // LINEAR
    rampX.go(x, time, LINEAR);
    rampY.go(y, time, LINEAR);
    break;
  case 1: // SINUSOIDAL IN OUT
    rampX.go(x, time, SINUSOIDAL_IN);
    rampY.go(y, time, SINUSOIDAL_OUT);
    break;
  case 2: // SINUSOIDAL OUT IN
    rampX.go(x, time, SINUSOIDAL_OUT);
    rampY.go(y, time, SINUSOIDAL_IN);
    break;
  case 3: // LINEAR - SINUSOIDAL_INOUT
    rampX.go(x, time, LINEAR);
    rampY.go(y, time, SINUSOIDAL_INOUT);
    break;
  case 4: // EXPONENTIAL IN
    rampX.go(x, time, EXPONENTIAL_IN);
    rampY.go(y, time, EXPONENTIAL_IN);
    break;
  case 5: // EXPONENTIAL OUT
    rampX.go(x, time, EXPONENTIAL_OUT);
    rampY.go(y, time, EXPONENTIAL_OUT);
    break;
  }
  while (rampX.isRunning() || rampY.isRunning()) {
    x = rampX.update();
    y = rampY.update();
    getInverseKinematics(x, y);
    targetActuatorPos[0] = 2 * homePos - thetaA;
    targetActuatorPos[1] = thetaB;
    odrvA.setPosition(targetActuatorPos[0] * reduction[0]);
    odrvB.setPosition(targetActuatorPos[1] * reduction[1]);
    getEncoderValues();
    sampleJumpAngleLog();
    distanceTick(); // Check distance sensor during movement
    delay(2);
  }
  getEncoderValues();
  sampleJumpAngleLog();
}
// type p or P into the serial monitor to continue
void pressPlay() {
  // flush any leftover '\n' or '\r' from the previous command
  delay(10);
  while (Serial.available()) {
    Serial.read();
  }
  // now wait for an explicit 'p' or 'P'
  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'p' || cmd == 'P') {
        break;
      }
    }
  }
}
// draw a bad line
void badLine(int num) {
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 5, 5, 0);
  }
  moveEndEffectorSimple(30, 170);
  pressPlay();
  moveEndEffectorSimple(-30, 110); // bad line
}
// draw a line
void line(int num) {
  int time = 700;
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }
  moveEndEffectorSimple(30, 170);
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }
  pressPlay();
  moveEndEffectorInterpolate(-30, 110, 0, time); // good line
}
// draw a bad square
void badSquare(int num) {
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 5, 5, 0);
  }
  moveEndEffectorSimple(25, 110);
  pressPlay();
  for (int i = 0; i < num; i++) {
    moveEndEffectorSimple(25, 160);
    moveEndEffectorSimple(-25, 160);
    moveEndEffectorSimple(-25, 110);
    moveEndEffectorSimple(25, 110);
    // Cross
    /*
    moveEndEffectorSimple(-25, 160);
    moveEndEffectorSimple(-25, 110);
    moveEndEffectorSimple(25, 160);
    moveEndEffectorSimple(25, 110);
    */
  }
}
// draw a square of side length 50mm
void square(int num) {
  int time = 600;
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }
  moveEndEffectorSimple(25, 110);
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }
  pressPlay();
  for (int i = 0; i < num; i++) {
    moveEndEffectorInterpolate(25, 160, 0, time);
    moveEndEffectorInterpolate(-25, 160, 0, time);
    moveEndEffectorInterpolate(-25, 110, 0, time);
    moveEndEffectorInterpolate(25, 110, 0, time);
  }
}
// draw a square of side length 50mm with a cross
//  draw a square of side length 50mm with a cross
void squareCross(int num) {
  int time = 600;

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }

  // start point: bottom-right corner
  moveEndEffectorSimple(25, 110);

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }

  pressPlay();

  for (int i = 0; i < num; i++) {
    // ---- square perimeter ----
    moveEndEffectorInterpolate(25, 160, 0, time);  // right side up
    moveEndEffectorInterpolate(-25, 160, 0, time); // top side left
    moveEndEffectorInterpolate(-25, 110, 0, time); // left side down
    moveEndEffectorInterpolate(25, 110, 0, time);  // bottom side right

    // ---- cross inside the square ----
    moveEndEffectorInterpolate(-25, 160, 0,
                               time); // diagonal 1: bottom-right -> top-left
    moveEndEffectorInterpolate(-25, 110, 0,
                               time); // left side down to bottom-left
    moveEndEffectorInterpolate(25, 160, 0,
                               time); // diagonal 2: bottom-left -> top-right
    moveEndEffectorInterpolate(25, 110, 0,
                               time); // right side down to start point
  }
}
// draw a circle of radius 50mm
//  draw a circle of radius 25mm centered at (0, 135)
void circle(int num) {
  int time = 600;

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }

  // start point: rightmost point of circle
  moveEndEffectorSimple(25, 135);

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }

  pressPlay();

  for (int i = 0; i < num; i++) {
    moveEndEffectorInterpolate(0, 110, 1, time);   // quadrant 1
    moveEndEffectorInterpolate(-25, 135, 2, time); // quadrant 2
    moveEndEffectorInterpolate(0, 160, 1, time);   // quadrant 3
    moveEndEffectorInterpolate(25, 135, 2, time);  // quadrant 4, back to start
  }
}
// draw a diamond with vertices (0,110), (-25,135), (0,160), (25,135)
void diamond(int num) {
  int time = 600;

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }

  // start point: right vertex
  moveEndEffectorSimple(25, 135);

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }

  pressPlay();

  for (int i = 0; i < num; i++) {
    moveEndEffectorInterpolate(0, 110, 0, time);   // right -> bottom
    moveEndEffectorInterpolate(-25, 135, 0, time); // bottom -> left
    moveEndEffectorInterpolate(0, 160, 0, time);   // left -> top
    moveEndEffectorInterpolate(25, 135, 0, time); // top -> right, back to start
  }
}
// draw a triangle that fits in a 50mm square
void triangle(int num) {
  int time = 600;

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }

  // start point: bottom center
  moveEndEffectorSimple(0, 110);

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }

  pressPlay();

  for (int i = 0; i < num; i++) {
    moveEndEffectorInterpolate(-25, 160, 0, time); // bottom -> top-left
    moveEndEffectorInterpolate(25, 160, 0, time);  // top-left -> top-right
    moveEndEffectorInterpolate(
        0, 110, 0, time); // top-right -> bottom center, back to start
  }
}
// draw a sine wave with a period of 50mm
void sineWave(int num) {
  int time = 600;

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }

  // start point: left crest
  moveEndEffectorSimple(-50, 160);

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }

  pressPlay();

  for (int i = 0; i < num; i++) {
    moveEndEffectorInterpolate(-25, 135, 3, time);
    moveEndEffectorInterpolate(0, 160, 3, time);
    moveEndEffectorInterpolate(25, 135, 3, time);
    moveEndEffectorInterpolate(50, 160, 3, time);
    moveEndEffectorInterpolate(25, 185, 3, time);
    moveEndEffectorInterpolate(0, 160, 3, time);
    moveEndEffectorInterpolate(-25, 185, 3, time);
    moveEndEffectorInterpolate(-50, 160, 3, time); // back to start
  }
}
// draw a square circle diamond and triangle sequence
void squareToDiamond(int num) {
  int time = 600;
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }
  moveEndEffectorSimple(25, 110);

  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 150, 150, 0);
  }

  pressPlay();

  for (int k = 0; k < num; k++) {
    // square
    moveEndEffectorInterpolate(25, 160, 0, time);
    moveEndEffectorInterpolate(-25, 160, 0, time);
    moveEndEffectorInterpolate(-25, 110, 0, time);
    moveEndEffectorInterpolate(25, 110, 0, time);

    moveEndEffectorInterpolate(25, 135, 0, time);

    // circle
    moveEndEffectorInterpolate(0, 110, 1, time);   // 2nd quadrant
    moveEndEffectorInterpolate(-25, 135, 2, time); // 1st quadrant
    moveEndEffectorInterpolate(0, 160, 1, time);   // 4th quadrant
    moveEndEffectorInterpolate(25, 135, 2, time);  // 3rd quadrant

    // diamond
    moveEndEffectorInterpolate(0, 110, 0, time);
    moveEndEffectorInterpolate(-25, 135, 0, time);
    moveEndEffectorInterpolate(0, 160, 0, time);
    moveEndEffectorInterpolate(50, 160, 0, time);
  }
}
// jump the 5 bar linkage
void jump(int num) {
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 5, 5, 5, 0);
  }
  moveEndEffectorSimple(0, 160); // move to start position
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 30, 200, 200, 0); // accel/decel reduced: 500 → 200
  }
  pressPlay(); // wait for 'p'
  for (int i = 0; i < num; i++) {
    beginJumpAngleLog(i + 1);
    moveEndEffectorInterpolate(0, 115, 4, 100); // compress: 100→115mm (softer)
    // Non-blocking delay: call distanceTick() repeatedly
    uint32_t compress_start = millis();
    while (millis() - compress_start < 60) {
      getEncoderValues();
      sampleJumpAngleLog();
      distanceTick();
    }
    moveEndEffectorInterpolate(0, 160, 4, 100); // extend to top
    endJumpAngleLog();
  }
}
// jump speed sweep with phase markers for Python data collection
// Markers: #START pct, #DOWN rep pct, #HOLD rep pct, #UP rep pct, #REP_END rep
// pct, #DONE pct
void jumpFrequency() {
  for (int i = 0; i < 2; i++) {
    setTrapTraj(i, 60, 600, 600,
                0); // Increased speed and acceleration (e.g., 60, 600)
  }

  int constant_delay =
      50; // Constant delay time 50ms (you can change it)

  // Run 10 times, increasing speed by 20% each time
  for (int step = 0; step < 10; step++) {
    float speed_factor = 1.0 + (0.2 * step); // 1.0, 1.2, 1.4 ... 2.8
    int push_time =
        (int)(100.0 /
              speed_factor); // Decrease push time (ms) proportionally to speed
    float speed_mps =
        0.045 / (push_time / 1000.0); // Speed (m/s). Displacement from 160mm to 115mm is 45mm or 0.045m

    // ── Speed level start marker ──────────────────────────────────────
    Serial.print("#START ");
    Serial.println(speed_mps, 2);

    for (int j = 1; j <= 3; j++) {
      // ── Compress phase ───────────────────────────────────
      Serial.print("#DOWN ");
      Serial.print(j);
      Serial.print(" ");
      Serial.println(speed_mps, 2);
      beginJumpAngleLog(j);
      moveEndEffectorInterpolate(0, 115, 4, 100); // Compress speed is tested constantly

      // ── Hold phase ───────────────────────────────────────
      Serial.print("#HOLD ");
      Serial.print(j);
      Serial.print(" ");
      Serial.println(speed_mps, 2);
      // Non-blocking delay: call distanceTick() repeatedly
      uint32_t hold_start = millis();
      while (millis() - hold_start < constant_delay) {
        getEncoderValues();
        sampleJumpAngleLog();
        distanceTick();
      }

      // ── Extend phase (push phase) ─────────────────────────
      // SPEED CHANGES HERE (push_time)
      Serial.print("#UP ");
      Serial.print(j);
      Serial.print(" ");
      Serial.println(speed_mps, 2);
      moveEndEffectorInterpolate(0, 160, 4, push_time);

      // ── Rep end marker ───────────────────────────────────
      Serial.print("#REP_END ");
      Serial.print(j);
      Serial.print(" ");
      Serial.println(speed_mps, 2);
      endJumpAngleLog();

      // Wait 2 seconds (2000ms) between jumps (non-blocking)
      uint32_t wait_start = millis();
      while (millis() - wait_start < 2000) {
        distanceTick();
      }
    }

    // ── Speed level done marker ───────────────────────────────────────
    Serial.print("#DONE ");
    Serial.println(speed_mps, 2);
  }
  Serial.println("#SWEEP_COMPLETE");
}
// move the end Effector up and down
void upDown(int num) {
  for (int i = 0; i < num; i++) {
    moveEndEffectorInterpolate(0, 80, 4, 1000);
    moveEndEffectorInterpolate(0, 180, 4, 1000);
  }
}

// read the distance sensor value every DIST_INTERVAL_MS milliseconds and print to serial
void distanceTick() {
  uint32_t now = millis();

  if (now - lastDistMs >= DIST_INTERVAL_MS) {
    lastDistMs = now;

    dist_adc = analogRead(sensorPin);
    dist_cm = 10826.7f / (dist_adc + 69.3f) - 3.89f;

    Serial.print("Distance (cm): ");
    Serial.println(dist_cm);
  }
}
