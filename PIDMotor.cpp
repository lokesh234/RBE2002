/*
* PIDMotor.cpp
*
* Created on: Oct 16, 2018
* Author: hephaestus
*/
#include "PIDMotor.h"
#include <Arduino.h>
PIDMotor::PIDMotor() :
myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT) {
}
PIDMotor::~PIDMotor() {
// TODO Auto-generated destructor stub
}
void PIDMotor::pidinit() {

myPID.SetMode(MANUAL);
myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(-PID_OUTPUT_COMPUTE_RANGE,
PID_OUTPUT_COMPUTE_RANGE);
myPID.SetTunings(Kp, Ki, Kd, P_ON_E);
myPID.SetSampleTime(5);
}
void PIDMotor::pidIMUcontrol(long long int orientation, int P){
//data = sensor->bufferINTERNAL;
float zPos = eulers[0]; //Z Euler angle
In = zPos;
Out = orientation;
error = P * (Out - In);
}
void PIDMotor::updateEulers(imu::Vector<3> newEuler) {
eulers = newEuler;
}
float PIDMotor::getIMUorientation(){
//sensor->
//data = sensor->bufferINTERNAL;
float zPos = eulers[0]; //Z Euler angle
//IMUPos = data[9]; //Z
IMUPos = zPos;
return IMUPos;
}
void PIDMotor::loop() {
if(turning) {
float Zorientation = this->getIMUorientation();
Input = (double) Zorientation * 30.55;
}else{
Input = (float) getPosition(); //change to angle if we are turning
}
bool thisErrPositive = Input > 0;
if (thisErrPositive != lastErrPositive) {
// strobe mode to trigger zeroing of the Integral buffer
// In case of passing zero clear the integral sum
Output = 0;
myPID.SetMode(MANUAL);
myPID.SetMode(AUTOMATIC);
}
lastErrPositive = thisErrPositive;
if (myPID.Compute()) {
int out = map(Output, -PID_OUTPUT_COMPUTE_RANGE,
PID_OUTPUT_COMPUTE_RANGE, getOutputMin(),
getOutputMax());
this->pidIMUcontrol(orientation, P);
setOutput(out + error);
}
double diff = Output - lastOutput;
if(diff > -1 && diff < 1) {
if((getSetPoint() - getPosition()) > -35 && (getSetPoint() - getPosition()) < 35) {
//we've settled;
if(!settled) {
settled = true;
turning = false;
settledTime = millis();
Serial.println("Settled!");
}
}
}else{
//Serial.println("Motor1 Encoder count: "+String((int) Motor1.getPosition()));
//Serial.println("Motor2 Encoder count: "+String((int)
Motor2.getPosition()));
if(!settled) {
Serial.println("Output: "+String(Output));
}
lastOutput = Output;
}
}
void PIDMotor::overrideCurrentPosition(int64_t val) {
overrideCurrentPositionHardware(val);
setSetpoint(val);
myPID.SetTunings(Kp, Ki, Kd, P_ON_E);
// strobe mode to trigger zeroing of the Integral buffer
Output = 0;
myPID.SetMode(MANUAL);
myPID.SetMode(AUTOMATIC);
}
void PIDMotor::setSetpoint(int64_t val) {
Setpoint = (float) val;
settled = false;
}
float PIDMotor::getSetPoint(){
return Setpoint;
}
void PIDMotor::SetTunings(double Kp, double Ki, double Kd) {
this->Kp = Kp;
this->Ki = Ki;
this->Kd = Kd;
overrideCurrentPosition(getPosition());
}
// Returns Vel in degress/second
double PIDMotor::calcVel(){
//current positions
double curPos=getPosition();
//current time
curTime=millis();
//time change in ms from last call
timeInterval=curTime-prevTime;
//encoder ticks since last call
movement=curPos-prevPos;
//encoder ticks to degrees
movement= movement *ticksToDeg;
//timeInterval in seconds
timeInterval=timeInterval/1000;
//Velocity in degrees per milliseconds
Vel=movement/timeInterval;
//sets curent vals to previous
prevPos=curPos;
prevTime=curTime;
return Vel;
}
