/*
* ExampleRobot.h
*
* Created on: Nov 5, 2018
* Author: hephaestus
*/
#ifndef SRC_EXAMPLEROBOT_H_
#define SRC_EXAMPLEROBOT_H_
#if defined(Arduino_h)
#include <Arduino.h>
#endif
#include "config.h"
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include "PIDMotor.h"
#include "ServoEncoderPIDMotor.h"
#include "HBridgeEncoderPIDMotor.h"
#include <Wire.h>
#include <WiiChuck.h>
#include "GearWrist.h"
#include <Preferences.h>
#include <WiFi.h>
#include <SimplePacketComs.h>
#include <Esp32SimplePacketComs.h>
#include <wifi/WifiManager.h>
#include <server/NameCheckerServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Ultrasonic.h>
#include <LiquidCrystal.h>
#include <BNO055SimplePacketComs.h>
#include <DFRobotIRPosition.h>
#include "coms/IRCamSimplePacketComsServer.h"
#include "coms/GetPIDConfigureSimplePacketComsServer.h"
#include "coms/GetPIDData.h"
#include "coms/PIDConfigureSimplePacketComsServer.h"
#include "math.h"
//states for checking a building....
//Note: Will have to set a global variable to denote
//if the detected building is a special case. I.E., if there are only
//3 sides
#define CHECKING_STREET_SIDE 0
#define ADVANCING_PAST_SIDE_1 1 //includes turning?
#define CHECKING_SECOND_SIDE 2
#define ADVANCING_PAST_SIDE_2 3
#define CHECKING_THIRD_SIDE 4
#define ADVANCING_PAST_SIDE_3 5
#define CHECKING_FOURTH_SIDE 6
#define RETURNING_TO_STREET 7 //after complete return to driving state machine
#define MAKING_TURN 8
#define PUTTING_OUT_FLAME 9
#define NO_FLAME_FOUND 10
//states for driving state machine:
//Note: Will drive forward, checking each area where a building "should" be.
#define DRIVING_FORWARD_ON_OAK 0
#define DRIVING_FORWARD_ON_THIRD 1 //if we see a building on our left, it is
special case/edge
#define DRIVING_FORWARD_ON_BEECH 2
#define DRIVING_FORWARD_ON_FIRST 3
#define DRIVING_FORWARD_ON_MAPLE 4 //a building on our right is special case.
enum state_t {
Startup,
WaitForConnect,
readGame,
readIMU,
readIR,
// Add more states here and be sure to add them to the cycle
};
enum stateOrFire_t {
startbetaposition,
movetooakfirstbuilding,
sonicoakfirstbuilding,
sonicoakfirstbuildingtrue,
IRCamoakfirstbuildingfirstfloor,
IRCamoakfirstbuildinggroundfloor,
sonicoakfirstbuildingfalse,
movetooaksecondbuilding,
sonicoaksecondbuilding,
sonicoaksecondbuildingtrue,
IRCamoaksecondbuildingfirstfloor,
IRCamoaksecondbuildinggroundfloor,
sonicoaksecondbuildingfalse,
movetooakthirdbuilding,
sonicoakthirdbuilding,
sonicoakthirdbuildingtrue,
IRCamoakthirdbuildingfirstfloor,
IRCamoakthirdbuildinggroundfloor,
sonicoakthirdbuildingfalse,
movetobeechfirstbuilding,
sonicbeechfirstbuilding,
sonicbeechfirstbuildingtrue,
IRCambeechfirstbuildingfirstfloor,
IRCambeechfirstbuildinggroundfloor,
sonicbeechfirstdbuildingfalse,
movetobeechsecondbuilding,
sonicbeechsecondbuilding,
sonicbeechsecondbuildingtrue,
IRCambeechsecondbuildingfirstfloor,
IRCambeechsecondbuildinggroundfloor,
sonicbeechsecondbuildingfalse,
movetobeechthirdbuilding,
sonicbeechthirdbuilding,
sonicbeechthirdbuildingtrue,
IRCambeechthirdbuildingfirstfloor,
IRCambeechthirdbuildinggroundfloor,
sonicbeechthirdbuildingfalse,
movetomaplefirstbuilding,
sonicmaplefirstbuilding,
sonicmaplefirstbuildingtrue,
IRCammaplefirstbuildingfirstfloor,
IRCammaplefirstbuildinggroundfloor,
sonicmaplefirstbuildingfalse,
movetomaplesecondbuilding,
sonicmaplesecondbuilding,
sonicmaplesecondbuildingtrue,
IRCammaplesecondbuildingfirstfloor,
IRCammaplesecondbuildinggroundfloor,
sonicmaplesecondbuildingfalse,
movetomaplethirdbuilding,
sonicmaplethirdbuilding,
sonicmaplethirdbuildingtrue,
IRCammaplethirdbuildingfirstfloor,
IRCammaplethirdbuildinggroundfloor,
sonicmaplethirdbuildingfalse,
thereturn
};
#define numberOfPID 2
class ExampleRobot {
private:
HBridgeEncoderPIDMotor motor1; // PID controlled motor object
HBridgeEncoderPIDMotor motor2; // PID controlled motor object
GearWrist * wristPtr; // An object to mux/demux the 2 motors using the bevel gear differential.
// Servo objects
RBE 2002 – C1022
Servo tiltEyes;
Servo jaw;
Servo panEyes;
float bufferCache[8];
// A value to check if enough time has elapsed to tun the sensors and prints
int64_t lastPrint = 0;
// Change this to set your team name
String * name;//
// List of PID objects to use with PID server
PIDMotor * pidList[numberOfPID];// = { &motor1.myPID, &motor2.myPID };
DFRobotIRPosition * camera;
#if defined(USE_GAME_CONTOL)
//Wii game pad
Accessory control;
#endif
#if defined(USE_IMU)
// Simple packet coms server for IMU
GetIMU * sensor;
// The IMU object
Adafruit_BNO055 bno;
#endif
#if defined(USE_WIFI)
// SImple packet coms implementation useing WiFi
UDPSimplePacket coms;
RBE 2002 – C1023
// WIfi stack managment state machine
WifiManager manager;
#endif
#if defined(USE_IR_CAM)
// IR camera
DFRobotIRPosition myDFRobotIRPosition;
IRCamSimplePacketComsServer * serverIR;
#endif
// RUn the game control logic
void runGameControl();
// Print the values of the robot
void printAll();
// The fast loop actions
// This should be run every loop and is internally gated for fast opperation
void fastLoop();
// Internal setup function. set up all objects
void setup();
//attach the PID servers
void setupPIDServers();
// State machine stategetIMU
state_t state=Startup;
public:
const int motorpin = 16;
const int encoderApin = 32;
RBE 2002 – C1024
const int encoderBpin = 35;
int ultrasonicAvalue;
int ultrasonicBvalue;
int ultrasonicCvalue;
int Badboi;
int Current_Debug_State = 0;
Servo TurretRotationMotor;
Servo TurretAngleMotor;
ServoEncoderPIDMotor Motor1;
ServoEncoderPIDMotor Motor2;
int a = Motor1.getPosition();
int b = Motor2.getPosition();
ExampleRobot(String * name);
virtual ~ExampleRobot();
// Pulse the loop function from the main thread
bool checkingBuilding = false;
int currentCheckState = 0;
int checkBuildingState = 0;
int drivingState = 0;
int stateCheck = 0; //amount of building checks performed so far during each state
bool motorsTravelling = false;
//working --> 27, 14
Ultrasonic Left = Ultrasonic(27, 14);
Ultrasonic Right = Ultrasonic(26, 25);
long timeMillis = 0;
void turnRobot(double degree);
bool lookDirection = 0; //0 --< LEFT, 1 --> RIGHT
void loop();
imu::Vector<3> eulers;
int buildingStateCheck = 0;
long angleServoWriteTime = 0;
bool flameFound();
bool checkingFaceMotorsSet = false;
void writeAngleServo(int angle);
int floorsChecked = 0;
int currentAngleServoValue = 90;
int currentRotationServoValue = 90;
//bool isAngleServoSettled = false;
bool isAngleServoSettled();
bool driveMore = true;
void doTurnThenGoToState(int state);
int nextBuildingState = 0;
bool makingTurnMotorsSet = false;
Servo fanController;
int facesChecked = 0;
};
#endif /* SRC_EXAMPLEROBOT_H_*/