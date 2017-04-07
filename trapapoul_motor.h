/*
 * trapapoul_motor.h
 *
 *  Created on: 22 juil. 2016
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_MOTOR_H_
#define TRAPAPOUL_MOTOR_H_


#include <Arduino.h>
#include "ERRcodes.h"
#include "trapapoul_config.h"


//parametres de moteur
#define QUAD0 1
#define QUAD1 3
#define QUAD2 2
#define QUAD3 0
#define PRERUN 0
#define INRUN 1
#define POSTRUN 2
#define OVERSHOOT 3
#define TIMEOUT 50
#define LOST 7
#define BROKEN 8
#define EXIT 9


volatile bool quadratureA,quadratureB;
volatile int motorPosition=0;
volatile unsigned long motorLastMoved;
volatile uint8_t machineState;
volatile uint32_t motorStopped=0;

void motorInit ();
void motorGoTo (int);
void motorTurn (int);
void motorForward();
void motorBackward();
void motorStop();
void resetPosition();

#define QUADRATUREON \
				PCIFR  |= bit (digitalPinToPCICRbit(pinQuadratureB));/*enable interrupt for the group*/\
				PCICR  |= bit (digitalPinToPCICRbit(pinQuadratureB));/*clear outstanding interrupt*/

#define QUADRATUREOFF PCIFR  &= ~(bit (digitalPinToPCICRbit(pinQuadratureB)))/*disable interrupt for the group*/


//interrupt routine for pin change of port B
ISR (PCINT1_vect){ // handle pin change interrupt for A0 to A7 here
	bool newQuadratureA;
	bool newQuadratureB;

	newQuadratureA=(digitalRead(pinQuadratureA));
	newQuadratureB=(digitalRead(pinQuadratureB));
	if(newQuadratureA != quadratureA){
		if(newQuadratureB!=quadratureB){
			machineState=LOST;
			motorStopped=millis();
			quadratureA=newQuadratureA;
			quadratureB=newQuadratureB;
			return;
		}
		if (newQuadratureB){
			if (newQuadratureA)
				motorPosition+=1;
			else
				motorPosition-=1;
		}else {
			if (newQuadratureA)
				motorPosition-=1;
			else
				motorPosition+=1;
		}
	}
	if(newQuadratureB != quadratureB){
		if (newQuadratureB){
			if (newQuadratureA)
				motorPosition-=1;
			else
				motorPosition+=1;
		}else {
			if (newQuadratureA)
				motorPosition+=1;
			else
				motorPosition-=1;
		}
	}
	quadratureA=newQuadratureA;
	quadratureB=newQuadratureB;
	motorLastMoved=millis();

}

void activateVpp(){
	pinMode(pinVppEn,OUTPUT);
	digitalWrite(pinVppEn,1);
}

void deactivateVpp(){
	digitalWrite(pinVppEn,0);
	delay(10);
	pinMode(pinVppEn,INPUT);
}

void activateMotor ()
{
	activateVpp();
	delay(1000);//just in case, can be quicken after tests
	PCIFR  |= bit (digitalPinToPCICRbit(pinQuadratureB));//enable interrupt for the group
	PCICR  |= bit (digitalPinToPCICRbit(pinQuadratureB));//clear outstanding interrupt
}

void deactivateMotor()
{
	delay(1000);//just to be sure
	PCIFR  &= ~(bit (digitalPinToPCICRbit(pinQuadratureB)));//disable interrupt for the group
	deactivateVpp();
}

void motorGoTo (int targetPosition)
{
	bool motorDirection;

	machineState=INRUN;
	motorDirection = (targetPosition>motorPosition);
	motorLastMoved=millis();
	if(motorDirection){//start motor
		digitalWrite(pinMotorBackward,LOW);
		delay(100);
		digitalWrite(pinMotorForward,HIGH);
	}else{
		digitalWrite(pinMotorForward,LOW);
		delay(100);
		digitalWrite(pinMotorBackward,HIGH);
	}
	while (1){
		if(machineState==LOST){
			motorStop();
			if ((millis()-motorStopped)>POST_RUN_TIME)
			break;
		}
		noInterrupts();
		if(motorPosition==targetPosition){
			machineState=POSTRUN;
			interrupts();
			break;
		}
		if ((millis()-motorLastMoved)>(MOTOR_TIME_OUT)){
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);
			machineState=TIMEOUT;
			interrupts();
			break;
		}
		if(motorDirection != (targetPosition>motorPosition)){
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);
			machineState=OVERSHOOT;
			interrupts();
			break;
		}
		interrupts();

	}

	digitalWrite(pinMotorForward,LOW);
	digitalWrite(pinMotorBackward,LOW);

	delay(100);
	if(machineState==TIMEOUT){
	}
	if(machineState==LOST){
	}
}

void motorTurn (int16_t motorDistance)
{
	int16_t target=motorDistance+motorPosition;
	motorGoTo(target);
}

void motorInit (){

	pinMode(pinMotorForward,OUTPUT);
	pinMode(pinMotorBackward,OUTPUT);
	pinMode(pinQuadratureA,INPUT);
	pinMode(pinQuadratureB,INPUT);
	//init quadrature
	quadratureA=digitalRead(pinQuadratureA);
	quadratureB=digitalRead(pinQuadratureB);
	//récupération de l'ancienne position en EEPROM à ajouter ici
    *digitalPinToPCMSK(pinQuadratureA) |= bit (digitalPinToPCMSKbit(pinQuadratureA));  // enable pin
    *digitalPinToPCMSK(pinQuadratureB) |= bit (digitalPinToPCMSKbit(pinQuadratureB));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pinQuadratureB)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pinQuadratureB)); // enable interrupt for the group

}

void motorForward(){
	digitalWrite(pinMotorBackward,LOW);
	delay(100);
	digitalWrite(pinMotorForward,HIGH);
}

void motorBackward(){
	digitalWrite(pinMotorForward,LOW);
	delay(100);
	digitalWrite(pinMotorBackward,HIGH);
}

void motorStop(){
	digitalWrite(pinMotorForward,LOW);
	digitalWrite(pinMotorBackward,LOW);
}

void resetPosition(){
	motorPosition=0;
}

void manualMoveMotor(){
uint8_t manualMoveMachineState=0;	//1=frd 2=bkw
//uint32_t millisMarker=0;
	manualMoveMachineState=0;
	clearButtons();
	while(1){
		if (manualMoveMachineState==0){
			if(buttonState[BPUP]==0){
				manualMoveMachineState=1;
				motorForward();
			}
			if(buttonState[BPDW]==0){
				manualMoveMachineState=2;
				motorBackward();
			}
		}
		if (manualMoveMachineState==1){
			if(buttonState[BPUP]==1){
				manualMoveMachineState=0;
				motorStop();
			}
			if(buttonState[BPDW]==0){
				manualMoveMachineState=0;
				motorStop();
			}
		}
		if (manualMoveMachineState==2){
			if(buttonState[BPUP]==0){
				manualMoveMachineState=0;
				motorStop();
			}
			if(buttonState[BPDW]==1){
				manualMoveMachineState=0;
				motorStop();
			}
		}
		if(buttonPushed[BPOK]){
			motorStop();
			clearButtons();
			return;
		}
	}
}

#endif /* TRAPAPOUL_MOTOR_H_ */
