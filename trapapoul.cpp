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


//pin config
#define BPUP 0
#define BPOK 3
#define pinQuadratureA 7
#define pinQuadratureB 4
#define pinMotorForward 6
#define pinMotorBackward 5
#define pinAlarm 2
#define pinChargeOFF 8
#define pinBuzzer 9


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

	//fonctions
void motorInit ();
void motorAlign ();
void motorGoTo (int);
void motorTurn (int);
void pushLog(String);



void serialSetRtc (uint8_t); //parameter is clock or alarm1 or alarm2
//create string with date "jj/mm/yyyy"
String printDate ();
//create string with time "hh:mm:ss"
String printTime ();
// interrupt routine for pin2 alarm.
void interrupt_0 () {
	//if (digitalRead(BPOK) == 0)
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
	Serial.println("\n");
	Serial.println("Press");
	while (Serial.available()==0);
	while (Serial.available()!=0)
		Serial.read();

/*
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
	}
	lcd.setCursor(0,0);
*/
//Init of RTC
	//RTC.set(SECS_YR_2000,CLOCK_ADDRESS);
	Serial.print("befRTC");
	RTC.disableAlarm(ALARM1_ADDRESS);
	//RTC.disableAlarm(ALARM2_ADDRESS);
	/*RTC.interruptSelect(INTB);
	RTC.resetAlarms();
	pinMode(pinAlarm, INPUT);
	attachInterrupt(digitalPinToInterrupt(3), interrupt_0 , FALLING);*/
	Serial.print(printDate());
	Serial.print(' ');
	Serial.print(printTime());
	Serial.println("changer heure? (y/n)");
	while (Serial.available()==0);
			if(Serial.available() > 0){
				if (Serial.read()=='y')
					serialSetRtc(CLOCK_ADDRESS);
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
	pinMode(pinMotorForward,OUTPUT);
	pinMode(pinMotorBackward,OUTPUT);
	pinMode(pinQuadratureA,INPUT);
	pinMode(pinQuadratureB,INPUT);
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
	bool quadratureA = digitalRead(pinQuadratureA);
	bool quadratureB = digitalRead(pinQuadratureB);
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
		if (digitalRead(pinQuadratureA))
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
		quadratureA = digitalRead(pinQuadratureA);
		quadratureB = digitalRead(pinQuadratureB);
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
	pinMode(pinQuadratureA,INPUT);
	pinMode(pinQuadratureB,INPUT);
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
	quadratureA = digitalRead(pinQuadratureA);
	quadratureB = digitalRead(pinQuadratureB);
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
			quadratureA = digitalRead(pinQuadratureA);
			quadratureB = digitalRead(pinQuadratureB);
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
		//Serial.println(" has been logged");
	}
	else{
		//Serial.print("logerror ");
		//Serial.println(pushee);
	}
}


void serialSetRtc (uint8_t adress){
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
	timeElements.Year=CalendarYrToTm(year);
	timeElements.Month=month;
	timeElements.Day=day;
	timeElements.Hour=hour;
	timeElements.Minute=minute;
	timeElements.Second=second;
	Serial.println("Press");
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
