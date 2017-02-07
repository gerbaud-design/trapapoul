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
#include "logSD.h"


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

//interrupt routine for pin change of port B
ISR (PCINT0_vect){ // handle pin change interrupt for D8 to D13 here
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

void activateMotor ()
{

	pinMode(pinVppEn,OUTPUT);
	digitalWrite(pinVppEn,1);
	delay(1000);//just in case, can be quicken after tests
	quadratureOn;
}

void deactivateMotor()
{
	delay(1000);//just to be sure
	quadratureOff;
	pinMode(pinVppEn,INPUT);
}

void motorGoTo (int targetPosition)
{
	bool motorDirection;

	Serial.print("P:");
	Serial.print(motorPosition);
	Serial.print("  ");
	Serial.print(quadratureA);
	Serial.println(quadratureB);
	delay(100);
	Serial.end();
	delay(100);
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
		//	Serial.print("P:");
		//	Serial.print(motorPosition);
		//	Serial.print("  ");
		//	Serial.print(quadratureA);
		//	Serial.println(quadratureB);
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);
			//Serial.print(ERR_POSLOSS);
			//Serial.print(IN_MOTGOTO);
			//pushLog(ERR_POSLOSS);
			//pushLog(IN_MOTGOTO);
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

	Serial.begin(9600);
	delay(100);
	if(machineState==TIMEOUT){
		Serial.println(ERR_TIMEOUT);
		Serial.println(IN_MOTGOTO);
		pushLog(ERR_TIMEOUT);
		pushLog(IN_MOTGOTO);
		pushLog(printTime());
		pushLog("broken");
		pushLog("\n");
		Serial.println(F("broken"));
	}
	if(machineState==LOST){
		Serial.print(ERR_POSLOSS);
		Serial.print(IN_MOTGOTO);
		pushLog(ERR_POSLOSS);
		pushLog(IN_MOTGOTO);
	}
	Serial.print("S:");
	Serial.println(machineState);
	Serial.print(" P:");
	Serial.print(motorPosition);
	Serial.print("  ");
	Serial.print(quadratureA);
	Serial.println(quadratureB);
}


void motorTurn (int16_t motorDistance)
{
	int16_t target=motorDistance+motorPosition;
	motorGoTo(target);
}


void motorInit (){

	pinMode(pinMotorForward,OUTPUT);
	digitalWrite(pinMotorForward,0);
	pinMode(pinMotorBackward,OUTPUT);
	digitalWrite(pinMotorBackward,0);
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
	while(1){
		if (manualMoveMachineState==0){
			if(digitalRead(pinBPUP)==0){
				manualMoveMachineState=1;
				motorForward();
			}
			if(digitalRead(pinBPDW)==0){
				manualMoveMachineState=2;
				motorBackward();
			}
		}
		if (manualMoveMachineState==1){
			if(digitalRead(pinBPUP)==1){
				manualMoveMachineState=0;
				motorStop();
			}
			if(digitalRead(pinBPDW)==0){
				manualMoveMachineState=0;
				motorStop();
			}
		}
		if (manualMoveMachineState==2){
			if(digitalRead(pinBPUP)==0){
				manualMoveMachineState=0;
				motorStop();
			}
			if(digitalRead(pinBPDW)==1){
				manualMoveMachineState=0;
				motorStop();
			}
		}
		if(digitalRead(pinBPOK)==0){
			motorStop();
			clearButtons();
			return;
		}
	}
}	/*
BEGIN:
	switch(waitButton()){
	case BPUP:
		motorForward();
		while(buttonState[BPUP]){
			lcd.setCursor(13,1);
			lcd.print(motorPosition);
		}
		motorStop();
		goto BEGIN;
	case BPDW:
		motorBackward();
		while(buttonState[BPDW]){
			lcd.setCursor(13,1);
			lcd.print(motorPosition);
		}
		motorStop();
		goto BEGIN;
	case TIMEOUT:
		goto BEGIN;
	case BPOK:
		return;
	}
}*/




#endif /* TRAPAPOUL_MOTOR_H_ */
