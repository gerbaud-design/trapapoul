/*
 * protopoulie.cpp
 *
 *  Created on: 22 avr. 2016
 *      Author: guiguilours
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <DS1337RTC.h>
#include <LiquidCrystal_I2C.h>
#include "ERRcodes.h"
#include <Wire.h>
#include <Time.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include "departementGPS.h"
#include <AVR/pgmspace.h>


//pin config
#define pinQuadratureA 9
#define pinQuadratureB 8
#define pinMotorForward 6
#define pinMotorBackward 5
#define pinAlarm 2
#define pinChargeOFF 4
#define pinBuzzer 7
#define pinBPUP 1
#define pinBPDW 0
#define pinBPOK 3



//SDconfig : set up variables using the SD utility library functions:
const uint8_t SDchipSelect = 10;
const String logFileName = "moulog.txt";
//String LogText;//log format : type;subtype;timestamp;value;comment

//RTCconfig
volatile bool timeToGo=0;
tmElements_t timeElements;
bool isTimeValid(tmElements_t*);
bool isDateValid(tmElements_t*);
extern DS1337RTC RTC;

//timer1config
volatile bool blink=0;
volatile uint8_t blinkCount=0;

//lcd_I2C config
LiquidCrystal_I2C lcd(0x27,16,2);
#define CHECK_CHAR 0
#define CHECK2_CHAR 1
#define CHECK3_CHAR 2
#define BAT1_CHAR 3
#define BAT2_CHAR 4
#define BAT3_CHAR 5
#define DEG_CHAR 6
void lcdClearLine(){lcd.print(F("                "));}

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

//boutons config
#define BPUP 0
#define BPDW 1
#define BPOK 2
#define DEBOUNCE 400 //ms
#define BUTTON_TIMEOUT 10000 //ms
volatile unsigned long lastPush[3];
volatile bool buttonState[3];
volatile bool buttonPushed[3];
void clearButtons();
uint8_t waitButton();

//menu variables
volatile uint16_t menuPointer=0;
void userInterface();
void enterNumber(uint8_t *val,uint8_t min,uint8_t max,
		uint8_t col, uint8_t lin, uint8_t digit/*, bool print0*/);
void enterTime(tmElements_t*);
#define BLINK_HALF_PERIOD 500


volatile bool quadratureA,quadratureB;
volatile int motorPosition=0;
volatile unsigned long motorLastMoved;
volatile uint8_t machineState;
const uint8_t motorOverturn=10;
#define POST_RUN_TIME 1000
#define MOTOR_TIME_OUT 10000
volatile uint32_t motorStopped=0;

	//fonctions
//void motorAlign ();
void motorGoTo (int);
void motorTurn (int);
void pushLog(String);

//config variables
#define SOLEIL 1
#define FIXE 2
#define MINIMUM 3
uint8_t openMode=SOLEIL;
uint8_t closeMode=SOLEIL;
uint8_t cur_mm=30;
uint8_t cur_hh=12;

int latitudeNord=45;
int longitudeOuest=-6;
void enterGPS (int*, int*);
void enterDepartement ();
void installationTrappe();

//adressage EEPROM
#define EEPROM_LON 10 //int
#define EEPROM_LAT 12 //int
#define EEPROM_POSO 14 //int
#define EEPROM_POSF 16 //int
#define EEPROM_POSITION 500 //int[50]
#define EEPROM_COUNT 600 //int[50]
uint8_t eeBlockCount;
uint16_t writeCount;
/*	eeBlockCount=0;
	do {
		EEPROM.get((EEPROM_COUNT+eeBlockCount),writeCount);
		if (writeCount>=10000)
			eeBlockCount++;
		else
			break;
	}while(eeBlockCount<50);*/

//functional variables
int curentPosition;

void serialSetRtc (uint8_t); //parameter is clock or alarm1 or alarm2
//create string with date "jj/mm/yyyy"
String printDate ();
//create string with time "hh:mm:ss"
String printTime ();
// interrupt routine for pin2 alarm.
//void interrupt_0 () {
	//if (digitalRead(BPOK) == 0)




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

//interrupt routine for pin change of port D
ISR (PCINT2_vect){ // handle pin change interrupt for D0 to D7 here
	bool newState[3];
	newState[BPOK]=(!(digitalRead(pinBPOK)));
	newState[BPDW]=(!(digitalRead(pinBPDW)));
	newState[BPUP]=(!(digitalRead(pinBPUP)));
	if (newState[BPOK]!=buttonState[BPOK]){
		if (newState[BPOK]==1 && ((millis()-lastPush[BPOK])>DEBOUNCE)){
			lastPush[BPOK]=millis();
			buttonPushed[BPOK]=1;
			buttonState[BPOK]=1;
		}else{
			buttonState[BPOK]=0;
		}
	}
	if (newState[BPDW]!=buttonState[BPDW]){
		if (newState[BPDW]==1 && ((millis()-lastPush[BPDW])>DEBOUNCE)){
			lastPush[BPDW]=millis();
			buttonPushed[BPDW]=1;
			buttonState[BPDW]=1;
		}else{
			buttonState[BPDW]=0;
		}
	}
	if (newState[BPUP]!=buttonState[BPUP]){
		if (newState[BPUP]==1 && ((millis()-lastPush[BPUP])>DEBOUNCE)){
			lastPush[BPUP]=millis();
			buttonPushed[BPUP]=1;
			buttonState[BPUP]=1;
		}else{
			buttonState[BPUP]=0;
		}
	}

}


// interrupt routine for pin2 alarm.
void interrupt_0 () {
	if (digitalRead(pinBPOK) == 0)
		timeToGo=1;
}

void interrupt_blinker(void)
{
	if (blink==0) {
		blink=1;
		blinkCount++;  // increase when LED turns on
	} else {
		blink=0;
	}
}

//The setup function is called once at startup of the sketch
void setup()
{


// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while(Serial.available() > 0)
		Serial.read();//flush serial input
	Serial.println(F("\n"));
	Serial.println(F("Press"));
	while (Serial.available()==0);
	while (Serial.available()!=0)
		Serial.read();


//setup du lcd
	lcd.init();
	lcd.backlight();
	lcd.clear();
	{
		uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
		 lcd.createChar(CHECK_CHAR, check);
	}{
		uint8_t check2[8] = {0x0,0x0,0x1,0x2,0x14,0x8,0x0};
		 lcd.createChar(CHECK2_CHAR, check2);
	}{
		uint8_t bat1[8] = {0x0,0x0,0x0,0x0,0xe,0x1f,0x1f};
		 lcd.createChar(BAT1_CHAR, bat1);
	}{
		uint8_t bat2[8] = {0x0,0x0,0xe,0x1f,0x1f,0x1f,0x1f};
		 lcd.createChar(BAT2_CHAR, bat2);
	}{
		uint8_t bat3[8] = {0xe,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f};
		 lcd.createChar(BAT3_CHAR, bat3);
	}{
		uint8_t deg[8] = {0xc,0x12,0x12,0x0c,0x00,0x00,0x00};
		 lcd.createChar(DEG_CHAR, deg);
	}
	lcd.setCursor(0,0);

//Init of RTC
	//RTC.set(SECS_YR_2000,CLOCK_ADDRESS);
	RTC.disableAlarm(ALARM1_ADDRESS);
	RTC.disableAlarm(ALARM2_ADDRESS);
	RTC.interruptSelect(INTB);
	RTC.resetAlarms();
	pinMode(pinAlarm, INPUT);
	attachInterrupt(digitalPinToInterrupt(3), interrupt_0 , FALLING);
	Serial.print(printDate());
	Serial.print(' ');
	Serial.print(printTime());
	Serial.println(F("changer heure? (y/n)"));
	while (Serial.available()==0);
			if(Serial.available() > 0){
				if (Serial.read()=='y')
					serialSetRtc(CLOCK_ADDRESS);
			}


//initialisation de la carte SD
	//Serial.println(F("Init SD card"));

	// we'll use the initialization code from the utility libraries
	// see if the card is present and can be initialized:
	pinMode(SDchipSelect,OUTPUT);
	if (!SD.begin(SDchipSelect)) {
		Serial.println(ERR_SDINIT);
		// don't do anything more:
		while(1);
	}
	Serial.println(INF_SDINIT);
	File LogFile = SD.open(logFileName, FILE_WRITE);
	// if the file is available, write to it:
	if (LogFile) {
		LogFile.close();
	}
	else
		Serial.println(ERR_SDOPEN);

	pushLog("\n");
	pushLog("\n");
	pushLog(INF_LOGNEW);
	pushLog(printDate());
	pushLog(";");
	pushLog(printTime());
	pushLog("\n");

	//initialize motor position
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

    //motorAlign();

//setup des boutons
	pinMode(pinBPUP, INPUT);
	pinMode(pinBPDW, INPUT);
	pinMode(pinBPOK, INPUT);
	digitalWrite(pinBPUP,1);
	digitalWrite(pinBPDW,1);
	digitalWrite(pinBPOK,1);
//setup des interuptions boutons
	*digitalPinToPCMSK(pinBPUP) |= bit (digitalPinToPCMSKbit(pinBPUP));  // enable pin
	*digitalPinToPCMSK(pinBPDW) |= bit (digitalPinToPCMSKbit(pinBPDW));  // enable pin
	*digitalPinToPCMSK(pinBPOK) |= bit (digitalPinToPCMSKbit(pinBPOK));  // enable pin
	PCIFR  |= bit (digitalPinToPCICRbit(pinBPDW)); // clear any outstanding interrupt
	PCICR  |= bit (digitalPinToPCICRbit(pinBPDW)); // enable interrupt for the group
	delay(50);
	lastPush[BPOK]=millis();
	lastPush[BPUP]=millis();
	lastPush[BPDW]=millis();
	clearButtons();

	Serial.println(F("end of setup"));


}

// The loop function is called in an endless loop
void loop()
{
	int motorDistance=0;
	int16_t positionHaute = 0;

	//debut import ihm
	clearButtons();
	userInterface();
	lcd.noBacklight();
	lcd.setCursor(0,0);
	lcdClearLine();
	lcd.setCursor(0,1);
	lcdClearLine();
	delay(1000);
	lcd.backlight();
	delay(1000);
	//fin import ihm

	//set up low position
	while(1){
		while(1){
			while(Serial.available() > 0)
				Serial.read();//flush serial input
			Serial.println(F("réglage posL"));
			Serial.println(F("0 pour finir"));
			while (Serial.available()==0);
			if(Serial.available() > 0){
				if(Serial.peek()=='0'){
					motorDistance=0;
					break;
				}
				motorDistance = Serial.parseInt();
				if(motorDistance !=0)
					break;
			}
		}
		if(motorDistance!=0){
			motorTurn(motorDistance);
			//motorAlign();
		}
		motorPosition=0;
		while(Serial.available() > 0)
			Serial.read();//flush serial input
		Serial.read();//flush serial input
		Serial.print(F("PosL ok? (y/n)"));
		while (Serial.available()==0);
		if(Serial.available() > 0)
			if (Serial.read()=='y'){
				pushLog(printTime());
				pushLog(";PosL :");
				pushLog(String(positionHaute));
				pushLog("\n");
				break;
			}
	}

	//set up high position
	while(1){
		while(1){
			while(Serial.available() > 0)
				Serial.read();//flush serial input
			Serial.println(F("réglage posH"));
			Serial.println(F("0 pour finir"));
			while (Serial.available()==0);
			if(Serial.available() > 0){
				if(Serial.peek()=='0'){
					motorDistance=0;
					break;
				}
				motorDistance = Serial.parseInt();
				if(motorDistance !=0)
					break;
				//verification
				/*Serial.print(F("nombre de tour à faire : "));
				Serial.println(MotorDistance);
				Serial.print(F("Validez (y/n)"));
				while (Serial.available()==0);
				if(Serial.available() > 0){
					if (Serial.read()=='y')*/
				Serial.println(F("ko"));
			}
			Serial.read();//flush serial input
			while(Serial.available() > 0)
				Serial.read();//flush serial input
		}
		motorTurn(motorDistance);
		positionHaute = motorPosition;
		while(Serial.available() > 0)
			Serial.read();//flush serial input
		Serial.println(F("PosH ok? (y/n)"));
		while (Serial.available()==0);
		if(Serial.available() > 0)
			if (Serial.read()=='y'){
				pushLog(printTime());
				pushLog(";PosH :");
				pushLog(String(positionHaute));
				pushLog("\n");
				break;
			}
	}

	//Serial.end();
	motorGoTo(0);
	uint16_t nbTours=0;
	while(1){
		nbTours++;
		if(nbTours%1100==0){
			//while(Serial.available() > 0)
			//	Serial.read();//flush serial input
			//Serial.println(F("Press"));
			//while (Serial.available()==0);
			//if(Serial.available() > 0)
			//	Serial.read();
			pushLog(printTime());
			pushLog(";end");
			while(1);
		}
		pushLog("Iteration ");
		pushLog(String(nbTours));
		pushLog("\n");
		//Serial.print(F("Iteration "));
		//Serial.println(nbTours);
		motorGoTo(positionHaute);
		pushLog(printTime());
		pushLog(";PosH\n");
		delay(10000);
		motorGoTo(0);
		pushLog(printTime());
		pushLog(";PosL\n");
		delay(10000);

	}

	pushLog(printTime());
	pushLog(";end");
	while(1){
		//Serial.println(F("end"));
		delay(2000);
	}
}
/*
void motorAlign ()
{
	bool maquadratureA = digitalRead(pinQuadratureA);
	bool maquadratureB = digitalRead(pinQuadratureB);
	uint8_t maquadrature = (maquadratureA + maquadratureA + maquadratureB);
	//Serial.print(F("alignequadrature : "));
	//Serial.println(quadrature);
	if (maquadrature==2){
		return;
	}
	digitalWrite(pinMotorForward,LOW);
	digitalWrite(pinMotorBackward,LOW);
	unsigned long timeMax=10000+millis();
	do
	{
		delay(50);
		if (digitalRead(pinQuadratureA))
			digitalWrite(pinMotorBackward,HIGH);
		else
			digitalWrite(pinMotorForward,HIGH);
		delay(5);
		digitalWrite(pinMotorForward,LOW);
		digitalWrite(pinMotorBackward,LOW);

		if(millis()>timeMax){
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);


			Serial.print(ERR_TIMEOUT);
			Serial.println(IN_MOTALIGN);
			pushLog(ERR_TIMEOUT);
			pushLog(IN_MOTALIGN);
			pushLog(";");
			pushLog(printTime());
			pushLog("\n");
			break;
		}
		maquadratureA = digitalRead(pinQuadratureA);
		maquadratureB = digitalRead(pinQuadratureB);
		maquadrature = (maquadratureA + maquadratureA + maquadratureB);
	} while(maquadrature != 1);//=quadrature=1

	//report
	pushLog(INF_POSITION);
	pushLog(";");
	pushLog(String(motorPosition));
	pushLog(";");
	pushLog(printTime());
	pushLog("\n");
	Serial.print(F("moteur aligné P:"));
	Serial.print(motorPosition);
}*/

void loadPosition(void){/*
	switch(oldMotorPosition%4){//use the variable as buffer before we use it for its true function
	case 0:
		oldQuadrature=QUAD0;
		oldQuadratureA=0;
		oldQuadratureB=1;
		break;
	case 1:
		oldQuadrature=QUAD1;
		oldQuadratureA=1;
		oldQuadratureB=1;
		break;
	case 2:
		oldQuadrature=QUAD2;
		oldQuadratureA=1;
		oldQuadratureB=0;
		break;
	case 3:
		oldQuadrature=QUAD3;
		oldQuadratureA=0;
		oldQuadratureB=0;
		break;
	}
	quadratureA = digitalRead(pinQuadratureA);
	quadratureB = digitalRead(pinQuadratureB);
	quadrature = (quadratureA + quadratureA + quadratureB);*/
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
		Serial.println(F("Press to exit broken"));
		while (Serial.available()==0);
		if(Serial.available() > 0){
			Serial.read();
			machineState=BROKEN;
		}
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




	/*
	machineState=PRERUN;
	while(1){

		if (machineState==EXIT)//here because we execute une dernière mise à jour position avant de quitter
			break;
		if(motorPosition==targetPosition){
			machineState=POSTRUN;
			postRunStamp=(millis()+POST_RUN_TIME);
		}


		//gestion du prérun,inrun,postrun
		switch(machineState){
		case PRERUN:
			 motorDirection = (targetPosition>motorPosition);
			 motorLastMoved=millis();
			 if(motorDirection){
				digitalWrite(pinMotorBackward,LOW);
				delay(100);
				digitalWrite(pinMotorForward,HIGH);
			}else{
				digitalWrite(pinMotorForward,LOW);
				delay(100);
				digitalWrite(pinMotorBackward,HIGH);
			}
			machineState=INRUN;
		case INRUN:
			while(1){// wait for a change in
				quadratureA = digitalRead(pinQuadratureA);
				quadratureB = digitalRead(pinQuadratureB);
				if (quadratureA!=oldQuadratureA)
					break;
				if (quadratureB!=oldQuadratureB)
					break;
				if (millis()>timeMax)
					break;
			}
			if (millis()>timeMax){
				Serial.println(ERR_TIMEOUT);
				Serial.println(IN_MOTALIGN);
				pushLog(ERR_TIMEOUT);
				pushLog(IN_MOTGOTO);
				pushLog(";");
				pushLog(printTime());
				pushLog("\n");
				digitalWrite(pinMotorForward,LOW);
				digitalWrite(pinMotorBackward,LOW);
				machineState=BROKEN;

			}

			quadrature = (quadratureA + quadratureA + quadratureB);
			if (quadrature==QUAD0)
				timeMax=millis()+motorTimeOut;
			break;
		case POSTRUN:
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);
			while(1){// wait for a change in
				quadratureA = digitalRead(pinQuadratureA);
				quadratureB = digitalRead(pinQuadratureB);
				if (quadratureA!=oldQuadratureA)
					break;
				if (quadratureB!=oldQuadratureB)
					break;
				if (millis()>timeMax)
					break;
			}
			quadrature = (quadratureA + quadratureA + quadratureB);
			if (millis()>postRunStamp)
				machineState = EXIT;
			break;
		case BROKEN:
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);
			pushLog(printTime());
			pushLog("broken");
			pushLog("\n");
			Serial.println(F("Press to exit broken"));
			while (Serial.available()==0);
			if(Serial.available() > 0){
				Serial.read();
				machineState=EXIT;
			}
		}
	}
*/
}


void motorTurn (int16_t motorDistance)
{
	int16_t target=motorDistance+motorPosition;
	motorGoTo(target);
}




#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};
bool isDateValid(tmElements_t *te){
	if (te->Month>12)
		return 0;
	if ((te->Month==2)&&(!LEAP_YEAR(te->Year))&&(te->Day==29))
		return 1;
	if (te->Day>monthDays[(te->Month)-1])
		return 0;
	return 1;
}

bool isTimeValid(tmElements_t *te){
	if (te->Second>59 || te->Minute>59 ||te->Hour>23)
		return 0;
	return 1;
}





void pushLog (String pushee)
{
	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.
	File LogFile = SD.open(logFileName, FILE_WRITE);
	// if the file is available, write to it:
	if (LogFile) {
		LogFile.print(pushee);
		LogFile.close();

		//Serial.print(pushee);
		//Serial.println(F(" has been logged"));
	}
	else{
		//Serial.print(F("logerror "));
		//Serial.println(pushee);
	}
}


void serialSetRtc (uint8_t adress){
	uint16_t year=0;
	uint8_t month=0,day=0,hour=0,minute=0,second=0;
	Serial.println(F("set rtc"));
	Serial.println(F("year"));
	while(Serial.available()==0);
	if(Serial.available()>0)
		year=Serial.parseInt();
	Serial.println(F("month"));
	while(Serial.available()==0);
	if(Serial.available()>0)
		month=Serial.parseInt();
	Serial.println(F("day"));
	while(Serial.available()==0);
	if(Serial.available()>0)
		day=Serial.parseInt();
	Serial.println(F("hour"));
	while(Serial.available()==0);
	if(Serial.available()>0)
		hour=Serial.parseInt();
	Serial.println(F("minute"));
	while(Serial.available()==0);
	if(Serial.available()>0)
		minute=Serial.parseInt();
	Serial.println(F("second"));
	while(Serial.available()==0);
	if(Serial.available()>0)
		second=Serial.parseInt();
	timeElements.Year=CalendarYrToTm(year);
	timeElements.Month=month;
	timeElements.Day=day;
	timeElements.Hour=hour;
	timeElements.Minute=minute;
	timeElements.Second=second;
	Serial.println(F("Press"));
	while(Serial.available()==0);
	if(Serial.available()>0)
		RTC.write(timeElements,adress);
}

//create string with date (jj/mm/yy)
String printDate (){
	String date="";
	//date+=("date (jj/mm/yyyy) : ");
	RTC.read(timeElements,CLOCK_ADDRESS);
	if(timeElements.Day<10)
		date+=("0");
	date+=(timeElements.Day);
	date+=("/");
	if(timeElements.Month<10)
		date+=("0");
	date+=(timeElements.Month);
	date+=("/");
	if((tmYearToCalendar(timeElements.Year))<10)
		date+=("0");
	date+=(tmYearToCalendar(timeElements.Year));
	return date;
}

//create string with time (hh:mm:ss)
String printTime (){
	String time="";

	//time+=("time (hh:mm:ss) : ");
	RTC.read(timeElements,CLOCK_ADDRESS);
	if(timeElements.Hour<10)
		time+=("0");
	time+=(timeElements.Hour);
	time+=(":");
	if(timeElements.Minute<10)
		time+=("0");
	time+=(timeElements.Minute);
	time+=(":");
	if(timeElements.Second<10)
		time+=("0");
	time+=(timeElements.Second);
	return time;
}




//
//debut import fonctions ihm
//
//

void clearButtons(void){
    buttonPushed[BPUP]=0;
    buttonPushed[BPOK]=0;
    buttonPushed[BPDW]=0;
}

uint8_t waitButton()
{
	unsigned long topTimeout=millis();
	/*delay(500);//debouncing
	while(1){//will have to handle timeout
		if(!digitalRead(pinBPUP))
			return BPUP;
		if(!digitalRead(pinBPOK))
			return BPOK;
		if(!digitalRead(pinBPDW))
			return BPDW;
		if(millis()>timeout)
			return TIMEOUT;
	}*/
	while(1){
		if(buttonPushed[BPOK]==1){
			buttonPushed[BPOK]=0;
			return BPOK;
		}
		if(buttonPushed[BPDW]==1){
			buttonPushed[BPDW]=0;
			return BPDW;
		}
		if(buttonPushed[BPUP]==1){
			buttonPushed[BPUP]=0;
			return BPUP;
		}
		if((millis()-topTimeout) > BUTTON_TIMEOUT)
			return TIMEOUT;
	}

}

void enterNumber(uint8_t *val,uint8_t min,uint8_t max,
		uint8_t col, uint8_t lin, uint8_t digit/*,bool print0*/){
	//init du timer1 et de son interrupt de clignotage
	unsigned long blinkerTime;
	Timer1.initialize(500000);
	Timer1.attachInterrupt(interrupt_blinker);
	Timer1.start();
	clearButtons();

	//print number in the right place
	lcd.setCursor(col,lin);
	if((*val<100) && (digit==3)/* && (print0==1)*/)
		lcd.print('0');
	if((*val<10) && (digit>=2)/* && (print0==1)*/)
		lcd.print('0');
	lcd.print(*val);
	while(1){
		blinkerTime = millis();

		if (blink==1){
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print('0');
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print('0');
			lcd.print(*val);
		}else{
			lcd.setCursor(col,lin);
			lcd.print(' ');
			if (digit>=2)
				lcd.print(' ');
			if (digit==3)
				lcd.print(' ');
		}
		if((buttonState[BPUP]==1)&&(*val<max)){
			*val+=1;
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print('0');
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print('0');
			lcd.print(*val);
		}
		if((buttonState[BPDW]==1)&&(*val>min)){
			*val-=1;
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print('0');
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print('0');
			lcd.print(*val);
		}
		if(buttonPushed[BPOK]==1){
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print('0');
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print('0');
			lcd.print(*val);
			Timer1.stop();
			Timer1.detachInterrupt();
			return;
		}
		while ((millis()-blinkerTime)<BLINK_HALF_PERIOD);
	}

}

void enterTime(tmElements_t *te){

ENTER_DATE :
	lcd.setCursor(0,0);
	lcd.print(F("DATE:           "));
	lcd.setCursor(0,1);
	if (te->Day<10)
		lcd.print('0');
	lcd.print(te->Day);
	lcd.print('/');
	if (te->Month<10)
		lcd.print('0');
	lcd.print(te->Month);
	lcd.print('/');
	lcd.print((uint16_t(te->Year)+1970));
	lcd.print(F("      "));

	//update values
	{
		uint8_t tmpYear=(tmYearToY2k(te->Year));
		enterNumber(&(te->Day),1,31,0,1,2);
		enterNumber(&(te->Month),1,12,3,1,2);
		enterNumber(&tmpYear,0,99,8,1,2);
		te->Year=y2kYearToTm(tmpYear);
	}

	//check validity
	if(!isDateValid(te)){	//check date validity

		lcd.setCursor(0,0);
		lcd.print(F("DATE NON VALIDE "));
		delay(1000);
		lcd.setCursor(0,0);
		lcdClearLine();
		delay(500);
		lcd.setCursor(0,0);
		lcd.print(F("DATE NON VALIDE "));
		delay(1000);
		goto ENTER_DATE;
	}

ENTER_TIME:
	lcd.setCursor(0,0);
	lcd.print(F("HEURE:          "));
	lcd.setCursor(0,1);
	if (te->Hour<10)
		lcd.print('0');
	lcd.print(te->Hour);
	lcd.print(':');
	if (te->Minute<10)
		lcd.print('0');
	lcd.print(te->Minute);
	lcd.print(':');
	if (te->Second<10)
		lcd.print('0');
	lcd.print(te->Second);
	lcd.print(F("        "));

	//update values
	enterNumber(&(te->Hour),0,23,0,1,2);
	enterNumber(&(te->Minute),0,59,3,1,2);
	enterNumber(&(te->Second),0,59,6,1,2);

	//check validity
	if(!isDateValid(te)){	//check date validity

		lcd.setCursor(0,0);
		lcd.print(F("HEURE NON VALIDE"));
		delay(1000);
		lcd.setCursor(0,0);
		lcdClearLine();
		delay(500);
		lcd.setCursor(0,0);
		lcd.print(F("HEURE NON VALIDE"));
		delay(1000);
		goto ENTER_TIME;
	}
}

void userInterface()
{
	MENU:
		delay(500);
		while(1){
			RTC.read(timeElements,CLOCK_ADDRESS);
			lcd.setCursor(0,0);
			if (timeElements.Hour<10)
				lcd.print('0');
			lcd.print(timeElements.Hour);
			lcd.print('H');
			if (timeElements.Minute<10)
				lcd.print('0');
			lcd.print(timeElements.Minute);
			lcd.print(':');
			lcd.print(timeElements.Second);
			lcd.print(F(" BAT:XX%"));
	/*		lcd.setCursor(9,0);
			lcd.write(BAT1_CHAR);
			lcd.write(BAT2_CHAR);
			lcd.write(BAT3_CHAR);*/
			lcd.setCursor(0,1);
			lcd.print(F("OUVRIRA A HH:MM "));
		//	lcd.print(F("FERMERA A HH:MM "));

			if(buttonState[BPOK]==1)
				goto MENU_OUVERTURE;
		}

	MENU_OUVERTURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DU MODE "));
		lcd.setCursor(0,1);
		lcd.print(F("D'OUVERTURE     "));
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE;
		case BPDW:
			goto MENU_FERMETURE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_OUVERTURE_SOLEIL;
		default:
			goto MENU_ERROR;
		}

	MENU_OUVERTURE_SOLEIL:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("LEVE DU SOLEIL  "));
		if (openMode==SOLEIL){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_SOLEIL;
		case BPDW:
			goto MENU_OUVERTURE_FIXE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (openMode == SOLEIL)
				goto MENU_OUVERTURE_ENREGISTREE;
			else{
				openMode=SOLEIL;
				goto MENU_OUVERTURE_SOLEIL;
			}
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_FIXE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE FIXE      "));
		if (openMode==FIXE){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_SOLEIL;
		case BPDW:
			goto MENU_OUVERTURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (openMode == FIXE)
				goto MENU_OUVERTURE_ENREGISTREE;
			else{
				openMode=FIXE;
				goto MENU_OUVERTURE_FIXE;
			}
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_MINIMUM:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE MINIMUM   "));
		if (openMode==MINIMUM){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_FIXE;
		case BPDW:
			goto MENU_OUVERTURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (openMode == MINIMUM)
				goto MENU_OUVERTURE_ENREGISTREE;
			else{
				openMode=MINIMUM;
				goto MENU_OUVERTURE_MINIMUM;
			}
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_RETOUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_MINIMUM;
		case BPDW:
			goto MENU_OUVERTURE_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_OUVERTURE;
		default:
			goto MENU_ERROR;
		}

	MENU_OUVERTURE_ENREGISTREE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("ENREGISTREE     "));
		delay(2000);
		goto MENU;

	MENU_FERMETURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DU MODE "));
		lcd.setCursor(0,1);
		lcd.print(F("DE FERMETURE    "));
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE;
		case BPDW:
			goto MENU_DATE_HEURE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_FERMETURE_SOLEIL;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_SOLEIL:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("COUCHE DU SOLEIL"));
		if (closeMode==SOLEIL){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_SOLEIL;
		case BPDW:
			goto MENU_FERMETURE_FIXE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (closeMode == SOLEIL)
				goto MENU_FERMETURE_ENREGISTREE;
			else{
				closeMode=SOLEIL;
				goto MENU_FERMETURE_SOLEIL;
			}
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_FIXE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE FIXE      "));
		if (closeMode==FIXE){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_SOLEIL;
		case BPDW:
			goto MENU_FERMETURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (closeMode == FIXE)
				goto MENU_FERMETURE_ENREGISTREE;
			else{
				closeMode=FIXE;
				goto MENU_FERMETURE_FIXE;
			}
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_MINIMUM:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE MINIMUM   "));
		if (closeMode==MINIMUM){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_FIXE;
		case BPDW:
			goto MENU_FERMETURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (closeMode == MINIMUM)
				goto MENU_FERMETURE_ENREGISTREE;
			else{
				closeMode=MINIMUM;
				goto MENU_FERMETURE_MINIMUM;
			}
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_RETOUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_MINIMUM;
		case BPDW:
			goto MENU_FERMETURE_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_FERMETURE;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_ENREGISTREE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("ENREGISTREE     "));
		delay(2000);
		goto MENU;


	MENU_DATE_HEURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DATE    "));
		lcd.setCursor(0,1);
		lcd.print(F("ET HEURE        "));
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE;
		case BPDW:
			goto MENU_INSTALLATION;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			RTC.read(timeElements,CLOCK_ADDRESS);
			enterTime(&timeElements);
			RTC.write(timeElements,CLOCK_ADDRESS);
			lcd.setCursor(0,0);
			lcd.print(F("DATE ET HEURE   "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			goto MENU;
		default:
			goto MENU_ERROR;
		}

	MENU_INSTALLATION:
	clearButtons();
	lcd.setCursor(0,0);
	lcd.print(F("INSTALLATION "));
	lcd.setCursor(0,1);
	lcd.print(F("DE LA TRAPPE    "));
	switch(waitButton()){
	case BPUP:
		goto MENU_DATE_HEURE;
	case BPDW:
		goto MENU_EXPERT;
	case TIMEOUT:
		goto MENU_TIMEOUT;
	case BPOK:
		installationTrappe();
		goto MENU_INSTALLATION;
	default:
		goto MENU_ERROR;
	}

	MENU_EXPERT:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE AVANCES "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_INSTALLATION;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_ERROR;
		default:
			goto MENU_ERROR;
		}

	MENU_QUITTER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("QUITTER         "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_EXPERT;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU;
		default:
			goto MENU_ERROR;
		}

	MENU_ERROR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("ERREUR MENU     "));
		lcd.setCursor(0,1);
		lcdClearLine();
		delay(5000);
		goto MENU;

	MENU_TIMEOUT:
		return;

}

void installationTrappe(){
//réglage hauteur
	//réglage date-heure
	//réglage gps


	MENU_HAUTEUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE HAUTEUR "));
		lcd.setCursor(0,1);
		lcd.print(F("DE LA TRAPPE    "));
		switch(waitButton()){
		case BPUP:
			goto MENU_HAUTEUR;
		case BPDW:
			goto MENU_DATE_HEURE;
		case TIMEOUT:
			goto MENU_HAUTEUR;
		case BPOK:
			goto MENU_ERROR;
			clearButtons();
			lcd.setCursor(0,0);
			lcd.print(F("REGLAGE DE LA   "));
			lcd.setCursor(0,1);
			lcd.print(F("POSITION FERMEE "));
			delay(1000);
			//reglage position fermée
			//valider
			EEPROM.put(EEPROM_POSF,curentPosition);
			lcd.setCursor(0,0);
			lcd.print(F("POSITION FERMEE "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			clearButtons();
			lcd.setCursor(0,0);
			lcd.print(F("REGLAGE DE LA   "));
			lcd.setCursor(0,1);
			lcd.print(F("POSITION OUVERTE"));
			delay(1000);
			//reglage position ouverte
			//valider
			EEPROM.put(EEPROM_POSO,curentPosition);
			lcd.setCursor(0,0);
			lcd.print(F("POSITION OUVERTE"));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			goto MENU_DATE_HEURE;
		default:
			goto MENU_ERROR;
		}

	MENU_DATE_HEURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DATE    "));
		lcd.setCursor(0,1);
		lcd.print(F("ET HEURE        "));
		switch(waitButton()){
		case BPUP:
			goto MENU_HAUTEUR;
		case BPDW:
			goto MENU_GPS;
		case TIMEOUT:
			goto MENU_DATE_HEURE;
		case BPOK:
			RTC.read(timeElements,CLOCK_ADDRESS);
			enterTime(&timeElements);
			RTC.write(timeElements,CLOCK_ADDRESS);
			lcd.setCursor(0,0);
			lcd.print(F("DATE ET HEURE   "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			goto MENU_GPS;
		default:
			goto MENU_ERROR;
		}


	MENU_GPS:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("MODIFIER LA     "));
		lcd.setCursor(0,1);
		lcd.print(F("POSITION GPS    "));
		switch(waitButton()){
		case BPUP:
			goto MENU_DATE_HEURE;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_GPS;
		case BPOK:
			goto MENU_GPS_DPT;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_DPT:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("POSITION GPS    "));
		lcd.setCursor(0,1);
		lcd.print(F("PAR DEPARTEMENT "));
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_DPT;
		case BPDW:
			goto MENU_GPS_GPS;
		case TIMEOUT:
			goto MENU_GPS_DPT;
		case BPOK:
			enterDepartement();
			goto MENU_GPS_ENREGISTRER;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_GPS:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("POSITION GPS    "));
		lcd.setCursor(0,1);
		lcd.print(F("MANUELLE        "));
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_DPT;
		case BPDW:
			goto MENU_GPS_RETOUR;
		case TIMEOUT:
			goto MENU_GPS_GPS;
		case BPOK:
			EEPROM.get(EEPROM_LAT,latitudeNord);
			EEPROM.get(EEPROM_LON,longitudeOuest);
			enterGPS(&latitudeNord,&longitudeOuest);
			EEPROM.put(EEPROM_LAT,latitudeNord);
			EEPROM.put(EEPROM_LON,longitudeOuest);
			goto MENU_GPS_ENREGISTRER;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_RETOUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_GPS;
		case BPDW:
			goto MENU_GPS_RETOUR;
		case TIMEOUT:
			goto MENU_GPS_RETOUR;
		case BPOK:
			goto MENU_GPS;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_ENREGISTRER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("POSITION GPS    "));
		lcd.setCursor(0,1);
		lcd.print(F("ENREGISTREE     "));
		delay(2000);
		goto MENU_QUITTER;


	MENU_QUITTER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_QUITTER;
		case BPOK:
			return;
		default:
			goto MENU_ERROR;
		}

	MENU_ERROR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("ERREUR MENU INST"));
		lcd.setCursor(0,1);
		lcdClearLine();
		delay(5000);
		return;

}


void enterDepartement (){
	uint8_t dpt=38;
	lcd.setCursor(0,0);
	lcd.print(F("ENTREZ VOTRE    "));
	lcd.setCursor(0,1);
	lcd.print(F("DEPARTEMENT: 38 "));
	enterNumber(&dpt,1,95,13,1,2);
	latitudeNord=departement[((dpt-1)*2)];
	longitudeOuest=departement[((dpt*2)-1)];
}


void enterGPS (int *latN, int *lonO){

	bool latNS=1; //used also as lonEO
	uint8_t newlat; //used also as newlon
	bool lonPointer=0;


	lcd.setCursor(0,0);
	lcd.print(F("LATITUDE NORD   "));
	lcd.setCursor(0,1);
	lcd.print(F("LATITUDE SUD    "));

LATLON_LABEL:
	//init du timer1 et de son interrupt de clignotage
	Timer1.initialize(500000);
	Timer1.attachInterrupt(interrupt_blinker);
	Timer1.start();
	if (lonPointer==1){
		lcd.setCursor(0,0);
		lcd.print(F("LONGITUDE OUEST "));
		lcd.setCursor(0,1);
		lcd.print(F("LONGITUDE EST   "));
		latNS=0;
	}
	lcd.setCursor(15,0);
	lcd.write(CHECK_CHAR);
	while(1){
		noInterrupts();
		bool blinkCopy=blink;
		interrupts();
		if (blinkCopy==1){
			//set the selected
			if (latNS==1)
				lcd.setCursor(15,0);
			else
				lcd.setCursor(15,1);
			//blink the selected
			lcd.write(CHECK_CHAR);
		}else{
			lcd.setCursor(15,0);
			lcd.print(' ');
			lcd.setCursor(15,1);
			lcd.print(' ');
		}
		if(buttonPushed[BPUP]==1){
			clearButtons();
			latNS=1;
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
			lcd.setCursor(15,1);
			lcd.print(' ');
		}
		if(buttonPushed[BPDW]==1){
			clearButtons();
			latNS=0;
			lcd.setCursor(15,1);
			lcd.write(CHECK_CHAR);
			lcd.setCursor(15,0);
			lcd.print(' ');
		}
		if(buttonPushed[BPOK]==1){
			clearButtons();
			Timer1.stop();
			Timer1.detachInterrupt();
			break;
		}
	}

	lcd.setCursor(0,0);
	if (latNS==1 && lonPointer==0){
		lcd.print(F("LATITUDE NORD   "));
		if ((*latN>=0 ))
			newlat=*latN;
		else
			newlat=0;
	}
	if(latNS==0 && lonPointer==0){
		lcd.print(F("LATITUDE SUD    "));
		if (*latN<=0)
			newlat=(*latN)*(-1);
		else
			newlat=0;
	}
	if (latNS==1 && lonPointer==1){
		lcd.print(F("LONGITUDE OUEST "));
		if ((*lonO>=0 ))
			newlat=*lonO;
		else
			newlat=0;
	}
	if(latNS==0 && lonPointer==1){
		lcd.print(F("LONGITUDE EST   "));
		if (*lonO<=0)
			newlat=(*lonO)*(-1);
		else
			newlat=0;
	}

	lcd.setCursor(0,1);
	lcdClearLine();
	lcd.setCursor(3,1);
	lcd.write(DEG_CHAR);
	enterNumber(&newlat,0,180,0,1,3);
	if (latNS==1 && lonPointer==0)
		*latN=newlat;
	if(latNS==0 && lonPointer==0)
		*latN=(newlat)*(-1);
	if (latNS==1 && lonPointer==1)
		*lonO=newlat;
	if(latNS==0 && lonPointer==1)
		*lonO=(newlat)*(-1);
	if (lonPointer==0){
		lonPointer=1;
		goto LATLON_LABEL;
	}
}
