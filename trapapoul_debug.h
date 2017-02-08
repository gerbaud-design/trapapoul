/*
 * trapapoul_debug.h
 *
 *  Created on: 8 févr. 2017
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_DEBUG_H_
#define TRAPAPOUL_DEBUG_H_


#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <DS1337RTC.h>
#include <TimerOne.h>
#include "trapapoul_config.h"



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


/*	lcd.backlight();
	while(1){

		lcd.setCursor(0,0);
		if (digitalRead(pinBPUP)==0)
			lcd.print("UP");
		else
			lcd.print("NU");

		lcd.setCursor(3,0);
		if (digitalRead(pinBPDW)==0)
			lcd.print("DW");
		else
			lcd.print("ND");

		lcd.setCursor(6,0);
		if (digitalRead(pinBPOK)==0)
			lcd.print("OK");
		else
			lcd.print("NO");
		delay(500);*/



#endif /* TRAPAPOUL_DEBUG_H_ */
