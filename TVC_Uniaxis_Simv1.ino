#include <Servo.h>
Servo servo;

//will add more variables in the future for multi-axis control; any commented out variable or function is for later use; be sure to convert radians to degrees

//simulation constants

float pi = 3.14159;
float timeStep = 0.001; //100Hz

//simulation dynamics

float timeSinceStart = 0;

//TVC dynamics

float motorTorque; //for torque-based control later on
float motorForce;
float motorThrust = 15;
float motorSetpointAngle = 0;
float motorAngle = 0;
float motorAngleTarget;
float motorSetpointDifferenceAngle;
float motorAngleRate = 60;
float servoInput;

//rocket properties dynamics

float vehicleMass = 0.4745;
float vehicleDistanceCMtoTVC = 0.0889;
float vehicleMMOI = 0.010681;
float Kp = 5;
float Kd = 5;

//rocket angle dynamics

//float vehicleAngleSetpoint;
float vehicleAngleAccel = 0;
float vehicleAngleRate = 0;
float vehicleAnglePosition = 10;
float previousVehicleAnglePosition = 0;
float previousVehicleAngleRate = 0;
float previousVehicleAngleAccel = 0;
//float vehicleAngleSetpointDifferenceAccel;
//float vehicleAngleSetpointDifferenceRate;
//float vehicleAngleSetpointDifferencePosition;

void setup() {

  Serial.begin(9600);
  servo.attach(9);

  randomSeed(8193204);

  Serial.println();
  Serial.println("Simulation Constants");
  Serial.println();
  Serial.print("Simulation Time Step:  ");
  Serial.println(timeStep);
  Serial.print("Starting Angle:  ");
  Serial.println(vehicleAnglePosition);
  Serial.println();


  delay(5000);
}

void loop() {

  //run sim here, fly with flight states in setup

  outputServos();
  responseDynamics();
  rotationDynamics();
  simOutput(); //happens last

  timeSinceStart = timeSinceStart + timeStep;
  delay(1);

}

void vehicleDynamics() {

  //model with polynomial curves

}

void rotationDynamics() {

  //equations of motion

  motorTorque = motorThrust * vehicleDistanceCMtoTVC * sin(motorAngle * pi / 180);
  vehicleAngleAccel = motorTorque / vehicleMMOI;

  //integration and differentiation; not for angular acceleration

  vehicleAngleRate = (vehicleAngleAccel + previousVehicleAngleAccel) * 0.5 * timeStep + vehicleAngleRate;
  vehicleAnglePosition = (vehicleAngleRate + previousVehicleAngleRate) * 0.5 * timeStep + vehicleAnglePosition;

  //integration setup for next loop

  previousVehicleAnglePosition = vehicleAnglePosition;
  previousVehicleAngleRate = vehicleAngleRate;
  previousVehicleAngleAccel = vehicleAngleAccel;

}

void responseDynamics() {

  //PD loop configuration and motor delay

  motorAngleTarget = -(Kp * vehicleAnglePosition) - (Kd * vehicleAngleRate);

  //actuation limit modeling

  if (motorAngleTarget > 14) {
    motorAngleTarget = 14;
  }
  else if (motorAngleTarget < -22.5) {
    motorAngleTarget = -22.5;
  }
  else {
    motorAngleTarget = motorAngleTarget;
  }

  //actuator delay *KEEP TRACK OF RATES MODELED IN THIS SECTION -- NO PARAMETER VARS IN*

  if (motorAngleTarget > motorAngle + 0.04) {
    motorAngle = motorAngle + 0.04;
  }
  else if (motorAngleTarget < motorAngle - 0.04) {
    motorAngle = motorAngle - 0.04;
  }
  else {
    motorAngle = motorAngleTarget;
  }
}

void simOutput() {

  //write variables to serial

  Serial.print(timeSinceStart);
  Serial.print("  MotorPos:  ");
  Serial.print(motorAngle);
  Serial.print("  Pos:  ");
  Serial.print(vehicleAnglePosition);
  Serial.print("  Rate:  ");
  Serial.print(vehicleAngleRate);
  Serial.print("  Accel:  ");
  Serial.print(vehicleAngleAccel);
  Serial.print("  Thrust (N):  ");
  Serial.println(motorThrust);

}

void errorNoise() {

}

void outputServos() {
  servoInput = (0.0171 * motorAngle * motorAngle) - (2.4744 * motorAngle) + 94;
  servo.write(servoInput);
}

void motorThrustCurve() {
  if (0 <= timeSinceStart) {
    motorThrust = 3.75;
  }
  if (0.1 <= timeSinceStart) {
    motorThrust = 13.75;
  }
  if (0.2 <= timeSinceStart) {
    motorThrust = 23.75;
  }
  if (0.3 <= timeSinceStart) {
    motorThrust = 18.75;
  }
  if (0.4 <= timeSinceStart) {
    motorThrust = 11;
  }
  if (0.5 <= timeSinceStart) {
    motorThrust = 10;
  }
  if (0.6 <= timeSinceStart) {
    motorThrust = 9;
  }
  if (0.7 <= timeSinceStart) {
    motorThrust = 8;
  }
  if (1.6 <= timeSinceStart) {
    motorThrust = 2;
  }
  if (1.65 <= timeSinceStart) {
    motorThrust = 8;
  }
}
