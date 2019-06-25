LCDStateMachine.h
/*
* LCDStateMachine.h
*
* Created on: Dec 11, 2018
* Author: lgangaramaney
*/
#include <string.h>
#include <LiquidCrystal.h>
#ifndef SRC_LCDSTATEMACHINE_H_
#define SRC_LCDSTATEMACHINE_H_
class LCDStateMachine{
public:
int lcdstate = 1;
void getlcdstate();
LiquidCrystal RobotLCD;
};
#endif /* SRC_LCDSTATEMACHINE_H_ */
