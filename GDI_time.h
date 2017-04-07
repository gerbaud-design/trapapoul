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



#endif /* GDI_TIME_H_ */
