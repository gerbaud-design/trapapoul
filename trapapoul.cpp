/*
 * protopoulie.cpp
 *
 *  Created on: 22 avr. 2016
 *      Author: guiguilours
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <MCP7940.h>
#include "ERRcodes.h"


//SDconfig : set up variables using the SD utility library functions:
const uint8_t SDchipSelect = 10;
const String logFileName = "moulog.txt";
//String LogText;//log format : type;subtype;timestamp;value;comment

//parametres rtc
DateTime calendar;
RTC_MCP7940 RTC;
void serialSetRtc ();//serial ihm set time
void serialSetAlarm();
String printDate ();//create string with date (jj/mm/yyyy)
String printTime ();//create string with time (hh:mm:ss)

//parametres de moteur
#define QUAD0 1
#define QUAD1 3
#define QUAD2 2
#define QUAD3 0
#define PRERUN 0
#define INRUN 1
#define POSTRUN 2
#define BROKEN 8
#define EXIT 9

const uint16_t motorTimeOut=10000;
const uint8_t motorOverturn=10;
int motorPosition=0;
	//pin config
const uint8_t pinquadratureA =4;
const uint8_t pinquadratureB =5;
const uint8_t pinMotorForward =6;
const uint8_t pinMotorBackward =7;
	//fonctions
void motorInit ();
void motorAlign ();
void motorGoTo (int);
void motorTurn (int);
void pushLog(String);

//The setup function is called once at startup of the sketch
void setup()
{


// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while(Serial.available() > 0)
		Serial.read();//flush serial input
	Serial.println("\n");
	Serial.println("Press");
	while (Serial.available()==0);
	while (Serial.available()!=0)
		Serial.read();

	//Init of RTC
	RTC.begin();
	calendar=RTC.now();
	Serial.println(printDate());
	//test RTC
	if ((RTC.isset())==0){
		serialSetRtc();
	}


//initialisation de la carte SD
	//Serial.println("Init SD card");

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
	pinMode(6,OUTPUT);
	pinMode(7,OUTPUT);
	pinMode(4,INPUT);
	pinMode(5,INPUT);
	motorInit();
}

// The loop function is called in an endless loop
void loop()
{
	int motorDistance=0;
	int16_t positionHaute = 0;

	//set up low position
	while(1){
		while(1){
			while(Serial.available() > 0)
				Serial.read();//flush serial input
			Serial.println("réglage posL");
			Serial.println("0 pour finir");
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
			motorAlign();
		}
		motorPosition=0;
		while(Serial.available() > 0)
			Serial.read();//flush serial input
		Serial.read();//flush serial input
		Serial.print("PosL ok? (y/n)");
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
			Serial.println("réglage posH");
			Serial.println("0 pour finir");
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
				/*Serial.print("nombre de tour à faire : ");
				Serial.println(MotorDistance);
				Serial.print("Validez (y/n)");
				while (Serial.available()==0);
				if(Serial.available() > 0){
					if (Serial.read()=='y')*/
				Serial.println("ko");
			}
			Serial.read();//flush serial input
			while(Serial.available() > 0)
				Serial.read();//flush serial input
		}
		motorTurn(motorDistance);
		positionHaute = motorPosition;
		while(Serial.available() > 0)
			Serial.read();//flush serial input
		Serial.println("PosH ok? (y/n)");
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

	Serial.end();
	motorGoTo(0);
	uint16_t nbTours=0;
	while(1){
		nbTours++;
		if(nbTours%1100==0){
			//while(Serial.available() > 0)
			//	Serial.read();//flush serial input
			//Serial.println("Press");
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
		//Serial.print("Iteration ");
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
		//Serial.println("end");
		delay(2000);
	}
}

void motorAlign ()
{
	bool quadratureA = digitalRead(pinquadratureA);
	bool quadratureB = digitalRead(pinquadratureB);
	uint8_t quadrature = (quadratureA + quadratureA + quadratureB);
	//Serial.print("alignequadrature : ");
	//Serial.println(quadrature);
	if (quadrature==2){
		return;
	}
	digitalWrite(pinMotorForward,LOW);
	digitalWrite(pinMotorBackward,LOW);
	unsigned long timeMax=10000+millis();
	do
	{
		delay(50);
		if (digitalRead(pinquadratureA))
			digitalWrite(pinMotorBackward,HIGH);
		else
			digitalWrite(pinMotorForward,HIGH);
		delay(2);
		digitalWrite(pinMotorForward,LOW);
		digitalWrite(pinMotorBackward,LOW);

		if(millis()>timeMax){
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);


			//Serial.println(ERR_TIMEOUT);
			//Serial.println(IN_MOTALIGN);
			pushLog(ERR_TIMEOUT);
			pushLog(IN_MOTALIGN);
			pushLog(";");
			pushLog(printTime());
			pushLog("\n");
			break;
		}
		quadratureA = digitalRead(pinquadratureA);
		quadratureB = digitalRead(pinquadratureB);
		quadrature = (quadratureA + quadratureA + quadratureB);
	} while(quadrature != 1);//=quadrature=1

	//report
	pushLog(INF_POSITION);
	pushLog(";");
	pushLog(String(motorPosition));
	pushLog(";");
	pushLog(printTime());
	pushLog("\n");
}

void motorInit ()
{
	pinMode(pinquadratureA,INPUT);
	pinMode(pinquadratureB,INPUT);
	pinMode(pinMotorForward,OUTPUT);
	pinMode(pinMotorBackward,OUTPUT);

/*	//ask serial the safety timer
	while(1){
		Serial.println("entrer le timeout moteur 1 tour en millisecondes");
		while (Serial.available()==0);
		if(Serial.available() > 0){
			MotorTimeOut = Serial.parseInt();
			Serial.print("timeout choisi : ");
			Serial.println(MotorTimeOut);
			Serial.print("Validez (y/n)");
			while (Serial.available()==0);
			if(Serial.available() > 0){
				if (Serial.read()=='y')
					break;
			}
		}
		else
			Serial.println("raté essaye encore!");
	}*/
	motorAlign();


}


void motorGoTo (int targetPosition)
{

	unsigned long timeMax=0;
	bool motorDirection;
	bool quadratureA;
	bool quadratureB;
	bool oldQuadratureA;
	bool oldQuadratureB;
	uint8_t quadrature;
	uint8_t oldQuadrature;
	uint8_t machineState=PRERUN;
	int oldMotorPosition=motorPosition;

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
	quadratureA = digitalRead(pinquadratureA);
	quadratureB = digitalRead(pinquadratureB);
	quadrature = (quadratureA + quadratureA + quadratureB);
	while(1){ //loop updating position starting before motor and ending after
		oldMotorPosition=motorPosition;
		switch(quadrature){//update position
		case QUAD0 :
			switch(oldQuadrature){
			case QUAD1 :
				motorPosition-=1;
				break;
			case QUAD2 :
				//Serial.println(ERR_POSLOSS);
				pushLog(ERR_POSLOSS);
				pushLog(";");
				pushLog(printTime());
				pushLog("\n");
				break;
			case QUAD3 :
				motorPosition+=1;
				break;
			}
			break;
		case QUAD1 :
			switch(oldQuadrature){
			case QUAD2 :
				motorPosition-=1;
				break;
			case QUAD3 :
				//Serial.println(ERR_POSLOSS);
				pushLog(ERR_POSLOSS);
				pushLog(";");
				pushLog(printTime());
				pushLog("\n");
				break;
			case QUAD0 :
				motorPosition+=1;
				break;
			}
			break;
		case QUAD2 :
			switch(oldQuadrature){
			case QUAD3 :
				motorPosition-=1;
				break;
			case QUAD0 :
				//Serial.println(ERR_POSLOSS);
				pushLog(ERR_POSLOSS);
				pushLog(";");
				pushLog(printTime());
				pushLog("\n");
				break;
			case QUAD1 :
				motorPosition+=1;
				break;
			}
			break;
		case QUAD3 :
			switch(oldQuadrature){
			case QUAD0 :
				motorPosition-=1;
				break;
			case QUAD1 :
				//Serial.println(ERR_POSLOSS);
				pushLog(ERR_POSLOSS);
				pushLog(";");
				pushLog(printTime());
				pushLog("\n");
				break;
			case QUAD2 :
				motorPosition+=1;
				break;
			}
			break;

		}
		oldQuadrature=quadrature;
		oldQuadratureA=quadratureA;
		oldQuadratureB=quadratureB;
		//Serial.print("Q");
		//Serial.println(quadrature);
		//Serial.print("P");
		//Serial.println(motorPosition);
		if (machineState==EXIT)//execute une dernière mise à jour position avant de quitter
			break;
		if(motorPosition==targetPosition)
			machineState=POSTRUN;


		//gestion du prérun,inrun,postrun
		switch(machineState){
		case PRERUN:
			 motorDirection = (targetPosition>motorPosition);
			 timeMax=millis()+motorTimeOut;
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
				quadratureA = digitalRead(pinquadratureA);
				quadratureB = digitalRead(pinquadratureB);
				if (quadratureA!=oldQuadratureA)
					break;
				if (quadratureB!=oldQuadratureB)
					break;
				if (millis()>timeMax)
					break;
			}
			if (millis()>timeMax){
				//Serial.println(ERR_TIMEOUT);
				//Serial.println(IN_MOTALIGN);
				pushLog(ERR_TIMEOUT);
				pushLog(IN_MOTGOTO);
				pushLog(";");
				pushLog(printTime());
				pushLog("\n");
				digitalWrite(pinMotorForward,LOW);
				digitalWrite(pinMotorBackward,LOW);
				machineState=BROKEN;

			}/*
			if((oldMotorPosition>motorPosition)&&motorDirection){
				Serial.println("sensinverse");
				digitalWrite(pinMotorForward,LOW);
				digitalWrite(pinMotorBackward,LOW);
				machineState=PRERUN;
			}
			if((oldMotorPosition<motorPosition)&&!motorDirection){
				Serial.println("sensinverse");
				digitalWrite(pinMotorForward,LOW);
				digitalWrite(pinMotorBackward,LOW);
				machineState=PRERUN;
			}*/

			quadrature = (quadratureA + quadratureA + quadratureB);
			if (quadrature==QUAD0)
				timeMax=millis()+motorTimeOut;
			break;
		case POSTRUN:
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);
			//wait for motor to stop and update quadrature
			delay(500);
			quadratureA = digitalRead(pinquadratureA);
			quadratureB = digitalRead(pinquadratureB);
			quadrature = (quadratureA + quadratureA + quadratureB);
			machineState = EXIT;
			break;
		case BROKEN:
			digitalWrite(pinMotorForward,LOW);
			digitalWrite(pinMotorBackward,LOW);
			pushLog(printTime());
			pushLog("broken");
			pushLog("\n");
			while(1){

			}

		}
	}

	//report
	//Serial.print(printTime());
	//Serial.print(" P");
	//Serial.println(motorPosition);
}


void motorTurn (int16_t motorDistance)
{
	int16_t target=motorDistance+motorPosition;
	motorGoTo(target);
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
		//Serial.println(" has been logged");
	}
	else{
		//Serial.print("logerror ");
		//Serial.println(pushee);
	}
}


void serialSetRtc (){
	uint16_t year=0;
	uint8_t month=0,day=0,hour=0,minute=0,second=0;
	Serial.println("set rtc");
	Serial.println("year");
	while(Serial.available()==0);
	if(Serial.available()>0)
		year=Serial.parseInt();
	Serial.println("month");
	while(Serial.available()==0);
	if(Serial.available()>0)
		month=Serial.parseInt();
	Serial.println("day");
	while(Serial.available()==0);
	if(Serial.available()>0)
		day=Serial.parseInt();
	Serial.println("hour");
	while(Serial.available()==0);
	if(Serial.available()>0)
		hour=Serial.parseInt();
	Serial.println("minute");
	while(Serial.available()==0);
	if(Serial.available()>0)
		minute=Serial.parseInt();
	Serial.println("second");
	while(Serial.available()==0);
	if(Serial.available()>0)
		second=Serial.parseInt();
	calendar = DateTime(year,month,day,hour,minute,second);
	Serial.println("Press");
	while(Serial.available()==0);
	if(Serial.available()>0)
		RTC.adjust(calendar);
}

void serialSetAlarm(){
	/*char ok=0;
	Serial.println("set the alarm to every minute? (y/n)");
	while(Serial.available()==0);
	if(Serial.available()>0)
		if(ok!='y')
			return;*/
	RTC.setAlarm(B00000001);
	RTC.configure(B10010000);
}

//create string with date (jj/mm/yyyy)
String printDate (){
	String date="";
	//date+=("date (jj/mm/yyyy) : ");
	calendar=RTC.now();
	if(calendar.day()<10)
		date+=("0");
	date+=(calendar.day());
	date+=("/");
	if(calendar.month()<10)
		date+=("0");
	date+=(calendar.month());
	date+=("/");
	if(calendar.year()<10)
		date+=("0");
	date+=(calendar.year());
	return date;
}

//create string with time (hh:mm:ss)
String printTime (){
	String time="";

	//time+=("time (hh:mm:ss) : ");
	calendar=RTC.now();
	if(calendar.hour()<10)
		time+=("0");
	time+=(calendar.hour());
	time+=(":");
	if(calendar.minute()<10)
		time+=("0");
	time+=(calendar.minute());
	time+=(":");
	if(calendar.second()<10)
		time+=("0");
	time+=(calendar.second());
	return time;
}
