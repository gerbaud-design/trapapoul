/*
 * battery-tester.cpp
 *
 *  Created on: 22 avr. 2016
 *      Author: guiguilours
 */

#include <Arduino.h>
#include <DS1337RTC.h>
#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <EEPROM.h>
#include "ERRcodes.h"
#include "departementGPS.h"
#include "trapapoul_UI.h"
#include "trapapoul_config.h"
#include "trapapoul_motor.h"
#include "logSD.h"


//config variables
uint8_t cur_mm=30;
uint8_t cur_hh=12;
uint16_t nbCycles;

extern tmElements_t timeElements;


//adressage EEPROM
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



void serialSetRtc (uint8_t); //parameter is clock or alarm1 or alarm2

// interrupt routine for pin2 alarm.
//void interrupt_0 () {
	//if (digitalRead(BPOK) == 0)





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
	lcd.noBacklight();
	lcd.clear();
	uploadChar(CHECK_CHAR,checkChar);
	uploadChar(DEG_CHAR,degChar);
	lcd.setCursor(0,0);

//Init of RTC
	//RTC.set(SECS_YR_2000,CLOCK_ADDRESS);
	RTC.disableAlarm(ALARM1_ADDRESS);
	RTC.disableAlarm(ALARM2_ADDRESS);
	RTC.interruptSelect(INTB);
	RTC.resetAlarms();
	pinMode(pinAlarm, INPUT);
	Serial.print(printDate());
	Serial.print(' ');
	Serial.print(printTime());
	Serial.println(F("changer heure? (y/n)"));
	while (Serial.available()==0);
			if(Serial.available() > 0){
				if (Serial.read()=='y')
					serialSetRtc(CLOCK_ADDRESS);
			}


	//analog init
	analogReference(EXTERNAL);
	//digital init
	pinMode(6,OUTPUT);
	digitalWrite(6,0);
	pinMode(7,OUTPUT);
	digitalWrite(7,1);

	logSdInit();

	Serial.println(F("end of setup"));


}

// The loop function is called in an endless loop
void loop()
{
	uint8_t stopMinute,stopHour,nowHour,nowMinute;

	uint32_t startTime;
	uint16_t value[10];
	uint16_t newValue;
	uint16_t moyeneTemp;
	uint16_t moyeneVres;
	uint16_t moyeneVbat;
	uint16_t moyeneVgnd;
	uint8_t i;

	//cycle de décharge
	digitalWrite(6,1);//start sink
	do{
		//temperature

		//init value array:
		for (i=0;i<10;i++)
			value[i]=0;

		startTime=millis();
		while((millis()- 2000)<startTime){
			newValue=analogRead(A0);
			if(newValue<value[9])
				continue;
			value[9]=newValue;
			for(i=8;i>0;i--){
				if (value[i]<=value[i+1]){
					newValue=value[i];
					value[i]=value[i+1];
					value[i+1]=newValue;
				}
			}
		}/*
		Serial.print("value9 : ");
		Serial.println(value[9]);
		Serial.print("value8 : ");
		Serial.println(value[8]);
		Serial.print("value7 : ");
		Serial.println(value[7]);
		Serial.print("value6 : ");
		Serial.println(value[6]);
		Serial.print("value5 : ");
		Serial.println(value[5]);
		Serial.print("value4 : ");
		Serial.println(value[4]);
		Serial.print("value3 : ");
		Serial.println(value[3]);
		Serial.print("value2 : ");
		Serial.println(value[2]);
		Serial.print("value1 : ");
		Serial.println(value[1]);
		Serial.print("value0 : ");
		Serial.println(value[0]);*/
		moyeneTemp=0;
		for(i=2;i<8;i++)
			moyeneTemp+=value[i];
		moyeneTemp/=6;

		Serial.print(printTime());
		Serial.print("D   temperature : ");
		Serial.println(moyeneTemp);
		pushLog("D;");
		pushLog(printTime());
		pushLog(";");
		pushLog(String(moyeneTemp));

		//voltages
		for (i=0;i<3;i++){
			value[i]=analogRead(1);
			delay(10);
			value[i+3]=analogRead(2);
			delay(10);
			value[i+6]=analogRead(3);
			delay(70);
		}
		moyeneVres=value[0]+value[1]+value[2];
		moyeneVbat=value[3]+value[4]+value[5];
		moyeneVgnd=value[6]+value[7]+value[8];

		Serial.print("Vres: ");
		Serial.print(moyeneVres);
		pushLog(";");
		pushLog(String(moyeneVres));
		Serial.print("  Vbat: ");
		Serial.print(moyeneVbat);
		pushLog(";");
		pushLog(String(moyeneVbat));
		Serial.print("  Vgnd: ");
		Serial.println(moyeneVgnd);
		pushLog(";");
		pushLog(String(moyeneVgnd));
		pushLog("/n");
	}while((moyeneVbat-moyeneVgnd)>2460);
	digitalWrite(6,0);//stop sink

	delay(1000000);

	//cycle de charge
	digitalWrite(7,0);//start charge
	RTC.read(timeElements,CLOCK_ADDRESS);
	//test 10min
	//stopHour=(((timeElements.Hour)+10)%24);
	//stopMinute=timeElements.Minute;
	stopHour=timeElements.Hour;
	stopMinute=(((timeElements.Minute)+10)%60);
	do{
		//temperature

		//init value array:
		for (i=0;i<10;i++)
			value[i]=0;

		startTime=millis();
		while((millis()- 2000)<startTime){
			newValue=analogRead(A0);
			if(newValue<value[9])
				continue;
			value[9]=newValue;
			for(i=8;i>0;i--){
				if (value[i]<=value[i+1]){
					newValue=value[i];
					value[i]=value[i+1];
					value[i+1]=newValue;
				}
			}
		}/*
		Serial.print("value9 : ");
		Serial.println(value[9]);
		Serial.print("value8 : ");
		Serial.println(value[8]);
		Serial.print("value7 : ");
		Serial.println(value[7]);
		Serial.print("value6 : ");
		Serial.println(value[6]);
		Serial.print("value5 : ");
		Serial.println(value[5]);
		Serial.print("value4 : ");
		Serial.println(value[4]);
		Serial.print("value3 : ");
		Serial.println(value[3]);
		Serial.print("value2 : ");
		Serial.println(value[2]);
		Serial.print("value1 : ");
		Serial.println(value[1]);
		Serial.print("value0 : ");
		Serial.println(value[0]);*/
		moyeneTemp=0;
		for(i=2;i<8;i++)
			moyeneTemp+=value[i];
		moyeneTemp/=6;

		Serial.print(printTime());
		Serial.print("C   temperature : ");
		Serial.println(moyeneTemp);
		pushLog("C;");
		pushLog(printTime());
		pushLog(";");
		pushLog(String(moyeneTemp));

		//voltages
		for (i=0;i<3;i++){
			value[i]=analogRead(1);
			delay(10);
			value[i+3]=analogRead(2);
			delay(10);
			value[i+6]=analogRead(3);
			delay(70);
		}
		moyeneVres=value[0]+value[1]+value[2];
		moyeneVbat=value[3]+value[4]+value[5];
		moyeneVgnd=value[6]+value[7]+value[8];

		Serial.print("Vres: ");
		Serial.print(moyeneVres);
		pushLog(";");
		pushLog(String(moyeneVres));
		Serial.print("  Vbat: ");
		Serial.print(moyeneVbat);
		pushLog(";");
		pushLog(String(moyeneVbat));
		Serial.print("  Vgnd: ");
		Serial.println(moyeneVgnd);
		pushLog(";");
		pushLog(String(moyeneVgnd));
		pushLog("/n");

		//exit test
		RTC.read(timeElements,CLOCK_ADDRESS);
		nowHour=timeElements.Hour;
		nowMinute=timeElements.Minute;
		if (stopHour==nowHour && stopMinute==nowMinute)
			break;
	} while(1);
	digitalWrite(7,1);//stop charge


	//end of cycle
	Serial.println("END OF CYCLE");
	Serial.println(moyeneVgnd);
	pushLog("END OF CYCLE /n");
	delay(1000000);
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



