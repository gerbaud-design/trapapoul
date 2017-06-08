/*
 * GDI_time.h
 *
 *  Created on: 5 avr. 2017
 *      Author: guiguilours
 */

#ifndef GDI_TIME_H_
#define GDI_TIME_H_


#include "Rtc_Pcf8563.h"


typedef struct  {
  uint8_t S;
  uint8_t M;
  uint8_t H;
} 	gdiTime_t;


typedef struct  {
  uint8_t Y;
  uint8_t M;
  uint8_t D;
} 	gdiDate_t;

PROGMEM const uint8_t daysInMonths[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

gdiTime_t nowTime;
gdiDate_t nowDate;
Rtc_Pcf8563 RTC;

void updateDateTime(){
	RTC.getDateTime();
	nowTime.H=RTC.getHour();
	nowTime.M=RTC.getMinute();
	nowTime.S=RTC.getSecond();
	nowDate.Y=RTC.getYear();
	nowDate.M=RTC.getMonth();
	nowDate.D=RTC.getDay();
}


#define SECS_PER_MIN 60
#define SECS_PER_HOUR 3600
#define SECS_PER_DAY 86400
void julianTranslate(uint8_t *hour,uint8_t *minute,uint8_t *second, double julian){
	//fill in hour minute and second from a -0.5<julian<0.5
	julian+=0.5;
	julian*=SECS_PER_DAY;
	uint32_t julianSec =julian;
	*second=(julianSec%SECS_PER_MIN);
	*minute=(julianSec%SECS_PER_HOUR)/SECS_PER_MIN;
	*hour=julianSec/SECS_PER_HOUR;
}

#endif /* GDI_TIME_H_ */
