/*
* ExampleRobot.cpp
* Remember, Remember the 5th of november
* Created on: Nov 5, 2018
* Author: hephaestus
*/
#include "ExampleRobot.h"
#define MOTOR1_SERVO_PIN 15
#define MOTOR1_ENCODER_A 32
#define MOTOR1_ENCODER_B 33
#define MOTOR2_SERVO_PIN 2
#define MOTOR2_ENCODER_A 35
#define MOTOR2_ENCODER_B 34
#define RELAY_TRIGGER_PIN 12
void ExampleRobot::loop() {
if (esp_timer_get_time() - lastPrint > 500
|| esp_timer_get_time() < lastPrint // check for the wrap over case
) {
switch (state) {
case Startup:
setup();
Serial.println(a);
state = WaitForConnect;
break;
case WaitForConnect:
#if defined(USE_WIFI)
if (manager.getState() == Connected)
#endif
state = readGame; // begin the main operation loop
break;
case readGame:
runGameControl();
state = readIMU;
break;
case readIMU:
#if defined(USE_IMU)
sensor->loop();
// if (PRINTROBOTDATA)
// sensor->print();
#endif
state = readIR;
break;
case readIR:
#if defined(USE_IR_CAM)
serverIR->loop();
if (PRINTROBOTDATA)
serverIR->print();
#endif
break;
}
printAll(); // Print some values in a slow loop
lastPrint = esp_timer_get_time(); // ensure 0.5 ms spacing *between* reads for Wifi to
transact
}

// If this is run before the sensor reads, the I2C will fail because the time it takes to send the UDP
causes a timeout
fastLoop(); // Run PID and wifi after State machine on all states
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
ExampleRobot::ExampleRobot(String * mn) {
pidList[0] = &motor1;
pidList[1] = &motor2;
wristPtr = NULL;
state = Startup;
ultrasonicCvalue = 0;
#if defined( USE_WIFI)
#if defined(USE_IMU)
sensor = NULL;
#endif
#if defined(USE_IR_CAM)
serverIR = NULL;
#endif
#endif
name = mn;
}
ExampleRobot::~ExampleRobot() {
// TODO Auto-generated destructor stub
}
void ExampleRobot::setupPIDServers(){
// coms.attach(new PIDConfigureSimplePacketComsServer(numberOfPID,pidList));
// coms.attach(new GetPIDData(numberOfPID,pidList));
// coms.attach(new GetPIDConfigureSimplePacketComsServer(numberOfPID,pidList));
}
void ExampleRobot::setup() {
if (state != Startup)
return;
state = WaitForConnect;
#if defined(USE_WIFI)
manager.setup();
#else
#endif
Serial.begin(115200);
delay(100);
//motor1.attach(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCA, MOTOR1_ENCB);
//motor2.attach(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCA, MOTOR2_ENCB);
Serial.println("Starting Motors");

Motor1.attach(MOTOR1_SERVO_PIN, MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
Motor2.attach(MOTOR2_SERVO_PIN, MOTOR2_ENCODER_A, MOTOR2_ENCODER_B);
TurretRotationMotor.attach(13, 750, 2250);
TurretAngleMotor.attach(4, 750, 2250);
fanController.attach(12, 1000, 2000);
pinMode(RELAY_TRIGGER_PIN, OUTPUT);
digitalWrite(RELAY_TRIGGER_PIN, LOW);
// // Create sensors and servers
#if defined(USE_IMU)
sensor = new GetIMU();
/* Initialise the sensor */
if (!bno.begin()) {
//Motor1.attach(motorpin,encoderApin,encoderBpin);
//Motor1.setOutput(0);
//Serial.println(a);
/* There was a problem detecting the BNO055 ... check your connections */
Serial.print(
"Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
delay(1000);
while (1)
;
}
delay(1000);
bno.setExtCrystalUse(true);
sensor->startSensor(&bno);
#endif
#if defined(USE_IR_CAM)
myDFRobotIRPosition.begin();
/*Serial.println(myDFRobotIRPosition.readX(0));
Serial.println(myDFRobotIRPosition.readY(0));
myDFRobotIRPosition.readX(0);
myDFRobotIRPosition.readY(0);*/
#endif
#if defined(USE_WIFI)
// Attach coms
#if defined(USE_IMU)
coms.attach(sensor);
#endif
#if defined(USE_IR_CAM)
serverIR = new IRCamSimplePacketComsServer(&myDFRobotIRPosition);
coms.attach(serverIR);
#endif
coms.attach(new NameCheckerServer(name));
setupPIDServers();
#endif
#if defined(USE_GAME_CONTOL)
control.begin();
control.readData(); // Read inputs and update maps
#endif
}
void ExampleRobot::turnRobot(double degree){
Motor1.turning = true;
Motor2.turning = true;
Motor1.setSetpoint(degree);
Motor2.setSetpoint(degree);
}
void ExampleRobot::writeAngleServo(int angle) {
if(angle != currentAngleServoValue) {
//new move, need to reset settle time;
angleServoWriteTime = millis();
currentAngleServoValue = angle;
}
TurretAngleMotor.write(currentAngleServoValue);
}
void ExampleRobot::doTurnThenGoToState(int nextState) {

nextBuildingState = nextState;
makingTurnMotorsSet = false;
checkBuildingState = MAKING_TURN;
}
bool ExampleRobot::isAngleServoSettled() {
if(millis() > angleServoWriteTime + 2000) {
return true;
}
return false;
}
void ExampleRobot::fastLoop() {
if (state == Startup) // Do not run before startp
return;
#if defined(USE_WIFI)
manager.loop();
if (manager.getState() == Connected)
coms.server();
else {
return;
}
#endif
//wristPtr->loop();
TurretRotationMotor.write(0);
//eulers = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
writeAngleServo(currentAngleServoValue);
TurretRotationMotor.write(currentRotationServoValue);
/*
switch(Current_Debug_State) {
case 0:
//Motor1.setSetpoint(3200);
//Motor2.setSetpoint(-3200);
TurretRotationMotor.write(90); //100 == right
TurretAngleMotor.write(90); //~85--> down, ~100 --> up
//Motor2.
Current_Debug_State = 1;
break;
case 1:
//Serial.println("Motor1 Encoder count: "+String((int) Motor1.getPosition()));
//Serial.println("Motor2 Encoder count: "+String((int) Motor2.getPosition()));
break;
default:
break;
}
int x = camera->readX(0);
}*/
// int leftTest = Left.read(INC);
// Serial.println("Left Distance: "+String((int) leftTest));
// int rightTest = Right.read(INC);
// Serial.println("Right Distance: "+String((int) rightTest));
Motor1.loop();
Motor2.loop();
if(checkingBuilding) {
switch(checkBuildingState) {
case CHECKING_STREET_SIDE:
//look down, check cam.
//look up, check cam.
//Note: Maybe can use this state for every side...
if(lookDirection) {
currentRotationServoValue = 15;
}else{
currentRotationServoValue = 150;
}
if(isAngleServoSettled()) {
if(!serverIR->flameFound()) {
if(floorsChecked >= 2) {
checkBuildingState++;
checkingFaceMotorsSet = false;
facesChecked++;
floorsChecked = 0;
}
if(currentAngleServoValue == 85) {
writeAngleServo(125);
}else{
writeAngleServo(85);
}
floorsChecked++;
}else{
//jump to flame state!
}
}
break;
case ADVANCING_PAST_SIDE_1:
if(!checkingFaceMotorsSet) {
Motor1.overrideCurrentPosition(0);
Motor2.overrideCurrentPosition(0);
Motor1.setSetpoint(-4500);
Motor2.setSetpoint(4500);
checkingFaceMotorsSet = true;
floorsChecked = 0;
}
if(Motor1.isSettled() && Motor2.isSettled()) {
int rightDistance = Right.read(INC);
if(rightDistance <= 10) {
checkingFaceMotorsSet = false;//keep driving
}else{
//turn
if(facesChecked>=4) {
//exit building check state.
doTurnThenGoToState(NO_FLAME_FOUND);
}else{
doTurnThenGoToState(CHECKING_SECOND_SIDE);

}
}
}
break;
case CHECKING_SECOND_SIDE:
if(!checkingFaceMotorsSet) {
Motor1.overrideCurrentPosition(0);
Motor2.overrideCurrentPosition(0);
Motor1.setSetpoint(-3500);
Motor2.setSetpoint(3500);
checkingFaceMotorsSet = true;
floorsChecked = 0;
}
if(Motor1.isSettled() && Motor2.isSettled()) {
//ping to see if building still there
int rightDistance = Right.read(INC);
if(rightDistance <= 10) {
//check for flame
if(isAngleServoSettled()) {
if(!serverIR->flameFound()) {

if(floorsChecked >=2) {
floorsChecked = 0;
checkingFaceMotorsSet = false;
//checkBuildingState++;
//instead jump to advance state?
facesChecked++;
checkBuildingState =
ADVANCING_PAST_SIDE_1;
}
if(currentAngleServoValue == 85) {
floorsChecked++;
writeAngleServo(125);
}else{
floorsChecked++;
writeAngleServo(85);
}
}else{
//flame found, jump to state!
checkBuildingState =
PUTTING_OUT_FLAME;
}
}
}else{
//probably already passed the building...
//advance to turning state...
}
}
break;
case ADVANCING_PAST_SIDE_2:
break;
case CHECKING_THIRD_SIDE:
break;
case ADVANCING_PAST_SIDE_3:
break;
case CHECKING_FOURTH_SIDE:
//look down, check cam.
//look up, check cam.
break;
case RETURNING_TO_STREET:

//when done, set
checkingBuilding = false;
break;
case MAKING_TURN:
if(!makingTurnMotorsSet) {
Motor1.overrideCurrentPosition(0);
Motor2.overrideCurrentPosition(0);
Motor1.setSetpoint(2400);
Motor2.setSetpoint(2400);
makingTurnMotorsSet = true;
}
if(Motor1.isSettled() && Motor2.isSettled()) {
checkBuildingState = nextBuildingState;
makingTurnMotorsSet = false; //for next time
checkingFaceMotorsSet = false;
}
break;
case PUTTING_OUT_FLAME:
digitalWrite(RELAY_TRIGGER_PIN, HIGH);
if(!serverIR->flameFound()) {

//digitalWrite(RELAY_TRIGGER_PIN, LOW);
checkBuildingState = 100;
}
break;
case NO_FLAME_FOUND:
//jump back to driving state machine.
checkingBuilding = false;
checkBuildingState = CHECKING_STREET_SIDE;
facesChecked = 0;
break;
default:
Serial.println("Default state reached, flame was extinguished.");
break;
}
}else{
//**** NOTE: Need to make sure to set lookDirection correctly if
//we ping a building
switch(drivingState) {
case DRIVING_FORWARD_ON_OAK:
if(driveMore) {
Motor1.overrideCurrentPosition(0);
Motor2.overrideCurrentPosition(0);

Motor1.setSetpoint(-6500);
Motor2.setSetpoint(6500);
timeMillis = millis();
motorsTravelling = true;
//stateCheck++;
driveMore = false;
}
if(Motor1.isSettled() && Motor2.isSettled()) {
//check for building
int rightDistance = Right.read(INC);
if(rightDistance <= 10) {
facesChecked = 0;
checkingBuilding = true;
driveMore = true; //for next time.
//this resets the servo settle value
//so that both servos have time to travel
if(currentAngleServoValue == 85) {
writeAngleServo(105);
}else{
writeAngleServo(85);
}

checkBuildingState = CHECKING_STREET_SIDE;
}else{
driveMore = true;
//if this is the 3rd time this happened, advance to next
portion/turn into 3rd Ave.
}
}
break;
/*
case DRIVING_FORWARD_ON_THIRD:
break;
case DRIVING_FORWARD_ON_BEECH:
break;
case DRIVING_FORWARD_ON_FIRST:
break;
case DRIVING_FORWARD_ON_MAPLE:
break;*/
}
}
}
bool ExampleRobot::flameFound() {
myDFRobotIRPosition.requestPosition();
for(int i = 0; i < 4; i++) {
int x = (int)(myDFRobotIRPosition.readX(i));
int y = (int)(myDFRobotIRPosition.readY(i));
/* if(x != 1023 && y != 1023) {
Serial.println("Flame Found!");
Serial.println("X: "+String(x)+", Y: "+String(y));
return true;
}*/
}
return false;
}
void ExampleRobot::runGameControl() {

#if defined(USE_GAME_CONTOL)
control.readData(); // Read inputs and update maps
float Servo1Val = mapf((float) control.values[1], 0.0, 255.0, -15.0, 15.0);
float Servo3Val = mapf((float) control.values[0], 0.0, 255.0, -60.0, 60.0); // z button
int panVal = map(control.values[2], 0, 255, 35, 148);
int jawVal = map(control.values[5] > 0 ? 0 : // Upper button pressed
(control.values[18] > 0 ? 255 : // Lower button pressed
128) //neither pressed
, 0, 255, 80, 140);
int tiltVal = map(control.values[3], 0, 255, 24, 120); // z button
panEyes.write(panVal);
tiltEyes.write(tiltVal);
jaw.write(jawVal);
wristPtr->setTarget(Servo1Val, Servo3Val);
#endif
}
void ExampleRobot::printAll() {
if (PRINTROBOTDATA) {
// Serial.println(
// " Pan = " + String(panVal) + " tilt = " + String(tiltVal));
// Serial.println(
// " Angle of A = " + String(wristPtr->getA()) + " Angle of B = "
// + String(wristPtr->getB()));
// Serial.println(
// " Tick of L = " + String((int32_t) motor1.getPosition())
// + " Angle of R = "
// + String((int32_t) motor2.getPosition()));
// for (int i = 0; i < WII_VALUES_ARRAY_SIZE; i++) {
// Serial.println(
// "\tVal " + String(i) + " = "
// + String((uint8_t) control.values[i]));
// }
//
}
}
#endif /* SRC_CONFIG_H_ */
