#include "math.h"

//simulation constants
float pi = 3.14159;
float timeStep = 0.002; //500Hz
float timeSinceStart = 0;
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 20Hz without using additional timers (time period(ms) =1000/frequency(Hz))

float arcsinHelper;
float integrationHelper[6];

//Rocket parameters
float T_C = 0.1; //meters
float m = 0.51; //kg
float I = 0.05; //idk
float thrust = 10; //newtons

//Rotation
float rocketRotation[3] = {10,0,0}; //position, velocity, acceleration

//Linear
float rocketMovement_Z[3] = {0,0,0}; //position, velocity, acceleration
float rocketMovement_X[3] = {0,0,0}; //position, velocity, acceleration

//Response
float accel_sp = 0;

float motorAngle_sp = 0; //degrees
float motorAngle_true = 0; //degrees
float motorAngle_trueGlobal; //degrees

float Kp = 0.35; //weight, aim for 3 deg sp on 5 deg error
float Kd = 0.60; //weight

//////////////////////////////////////////////////////////////////////////////////////

void valueShift() {
  
  I = 0.00566 - (0.0001842 * timeSinceStart);

  m = 0.5094 - (0.005789 * timeSinceStart);

  T_C = 0.105 + (0.005263 * timeSinceStart);
  
}

//////////////////////////////////////////////////////////////////////////////////////

void rocketDynamics() {

  //actuator delay *KEEP TRACK OF RATES MODELED IN THIS SECTION -- NO PARAMETER VARS IN*
  if (motorAngle_sp > motorAngle_true + 0.15) {
    motorAngle_true = motorAngle_true + 0.15;
  }
  else if (motorAngle_sp < motorAngle_true - 0.15) {
    motorAngle_true = motorAngle_true - 0.15;
  }
  else {
    motorAngle_true = motorAngle_sp;
  }

  //rotational movement
  rocketRotation[2] = ((thrust * T_C * sin(motorAngle_true) * 180 / pi) / I);

  rocketRotation[1] = (rocketRotation[2] + integrationHelper[0]) * 0.5 * timeStep + rocketRotation[1];
  integrationHelper[0] = rocketRotation[2];

  rocketRotation[0] = (rocketRotation[1] + integrationHelper[1]) * 0.5 * timeStep + rocketRotation[0];
  integrationHelper[1] = rocketRotation[1];

  //lateral movement
  rocketMovement_Z[2] = (cos((rocketRotation[0] - motorAngle_true) * pi / 180) * thrust / m) - 9.80665;
  rocketMovement_X[2] = (sin((rocketRotation[0] - motorAngle_true) * pi / 180) * thrust / m);

  rocketMovement_Z[1] = (rocketMovement_Z[2] + integrationHelper[2]) * 0.5 * timeStep + rocketMovement_Z[1];
  integrationHelper[2] = rocketMovement_Z[2];

  rocketMovement_Z[0] = (rocketMovement_Z[1] + integrationHelper[3]) * 0.5 * timeStep + rocketMovement_Z[0];
  integrationHelper[3] = rocketMovement_Z[1];

  rocketMovement_X[1] = (rocketMovement_X[2] + integrationHelper[4]) * 0.5 * timeStep + rocketMovement_X[1];
  integrationHelper[4] = rocketMovement_X[2];

  rocketMovement_X[0] = (rocketMovement_X[1] + integrationHelper[5]) * 0.5 * timeStep + rocketMovement_X[0];
  integrationHelper[5] = rocketMovement_X[1];

  if (rocketMovement_Z[0] <= 0) {
    rocketMovement_Z[0] = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////

void responseDynamics() {

  //PD loop configuration
  accel_sp = -(Kp * rocketRotation[0]) - (Kd * rocketRotation[1]);

  //torque to motor angle
  arcsinHelper = accel_sp * I / (thrust * T_C);

  if (arcsinHelper > 1) {
    arcsinHelper = 0.9;
  }
  else if (arcsinHelper < -1) {
    arcsinHelper = -0.9;
  }
  else {
    arcsinHelper = arcsinHelper;
  }
  
  motorAngle_sp = (180 / pi) * asin(arcsinHelper);

  //actuation limit modeling

  if (motorAngle_sp > 3) {
    motorAngle_sp = 3;
  }
  else if (motorAngle_sp < -3) {
    motorAngle_sp = -3;
  }
  else {
    motorAngle_sp = motorAngle_sp;
  }

}

//////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin(115200);

  randomSeed(analogRead(0));
  
}

void loop() {
  motorThrustCurve();
  rocketDynamics();

  if (((timeSinceStart * 1000) - lastStreamTime) >= streamPeriod)
  {
    responseDynamics();
    lastStreamTime = timeSinceStart * 1000;
  }
  
  timeSinceStart = timeSinceStart + timeStep;

  
  Serial.print(timeSinceStart);
  Serial.print("    ");
  Serial.print(rocketRotation[0]);
  Serial.print("    ");
  Serial.print(motorAngle_true);
  Serial.print("                  ");
  Serial.print(rocketMovement_Z[0]);
  Serial.print("    ");
  Serial.print(rocketMovement_X[0]);
  Serial.print("                  ");
  Serial.print(thrust);
  Serial.println();

  while(timeSinceStart > 55) {
    Serial.println();
    Serial.print("Sim ended.");
    delay(100000);
  }
  
}

void motorThrustCurve() {
  if (0 <= timeSinceStart) {
    thrust = 5;
  }
  if (0.1 <= timeSinceStart) {
    thrust = 9;
  }
  if (0.2 <= timeSinceStart) {
    thrust = 12;
  }
  if (0.3 <= timeSinceStart) {
    thrust = 6;
  }
  if (0.4 <= timeSinceStart) {
    thrust = 4.7;
  }
  if (0.5 <= timeSinceStart) {
    thrust = 4.8;
  }
  if (1.6 <= timeSinceStart) {
    thrust = 5;
  }
  if (2 <= timeSinceStart) {
    thrust = 0.5;
  }
}
