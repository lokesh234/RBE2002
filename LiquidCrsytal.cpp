/*
* lcdstate.cpp
*
* Created on: Dec 11, 2018
* Author: lgangaramaney
*/
#include "LCDStateMachine.h"
void LCDStateMachine::getlcdstate(){
if(lcdstate == 1){
RobotLCD.print("200 Oak St.");
}if(lcdstate == 2){
RobotLCD.print("500 2nd Ave.");
}if(lcdstate == 3){
RobotLCD.print("100 Beech St.");
}if(lcdstate == 4){
RobotLCD.print("600 1st Ave.");
}if(lcdstate == 5){
RobotLCD.print("400 Oak St.");
}if(lcdstate == 6){
RobotLCD.print("500 3rd Ave");
}if(lcdstate == 7){
RobotLCD.print("300 Beech St.");
}if(lcdstate == 8){
RobotLCD.print("600 2nd Ave.");
}if(lcdstate == 9){
RobotLCD.print("600 3rd Ave.");
}if(lcdstate == 10 ){
RobotLCD.print("600 Oak St.");
}if(lcdstate == 11){
RobotLCD.print("500 Beech St.");
}if(lcdstate == 12){
RobotLCD.print("400 Beech St.");
}if(lcdstate == 13){
RobotLCD.print("400 2nd Ave.");
}if(lcdstate == 14){
RobotLCD.print("300 Maple St.");
}if(lcdstate == 15){
RobotLCD.print("300 3rd Ave.");
}if(lcdstate == 16){
RobotLCD.print("200 Beech St.");
}if(lcdstate == 17){
RobotLCD.print("400 1st Ave.");
}if(lcdstate == 18){
RobotLCD.print("100 Maple St.");
}if(lcdstate == 19){
RobotLCD.print("300 2nd Ave.");
}if(lcdstate == 20){
RobotLCD.print("200 Maple St.");
}if(lcdstate == 21){
RobotLCD.print("200 1st Ave.");
}if(lcdstate == 22){
RobotLCD.print("100 2nd Ave.");
}if(lcdstate == 23){
RobotLCD.print("400 Maple St.");
}if(lcdstate == 24){
RobotLCD.print("200 2nd Ave.");
}if(lcdstate == 25){
RobotLCD.print("100 3rd Ave.");
}if(lcdstate == 26){
RobotLCD.print("600 Maple St.");
//look left?
}if(lcdstate == 27){
RobotLCD.print("200 3rd Ave.");
}if(lcdstate == 28){
RobotLCD.print("100 3rd Ave.");
//add last building in here
}
}
