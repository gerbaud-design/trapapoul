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

void updateDate();
void updateTime();
void updateDateTime();
void julianTranslate(uint8_t *hour,uint8_t *minute,uint8_t *second, double julian);
void summerTimeDates (uint8_t year, uint8_t *start, uint8_t *finish);
bool isSummerTime();

PROGMEM const uint8_t daysInMonths[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

gdiTime_t nowUTCTime;
gdiTime_t nowLocalTime;
gdiDate_t nowDate;
bool isTodaySummerTime=0;
Rtc_Pcf8563 RTC;

void updateDate(){
	RTC.getDateTime();
	nowDate.Y=RTC.getYear();
	nowDate.M=RTC.getMonth();
	nowDate.D=RTC.getDay();

}

void updateTime(){
	RTC.getDateTime();
	nowUTCTime.H=RTC.getHour();
	nowUTCTime.M=RTC.getMinute();
	nowUTCTime.S=RTC.getSecond();
}

void updateDateTime(){
	RTC.getDateTime();
	nowUTCTime.H=RTC.getHour();
	nowUTCTime.M=RTC.getMinute();
	nowUTCTime.S=RTC.getSecond();
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

void summerTimeDates (uint8_t year, uint8_t *start, uint8_t *finish){
	uint8_t weekDay3103, weekDay3110;
	weekDay3103=RTC.whatWeekday(31,3,0,year);
	weekDay3110=RTC.whatWeekday(31,10,0,year);
	*start=31-weekDay3103;
	*finish=31-weekDay3110;
}



//summertime from last sunday of march to last sunday of october
bool isSummerTime(){
	if (nowDate.M<3) return 0;
	if (nowDate.M>10) return 0;
	if (nowDate.M>3 && nowDate.M<10) return 1;
	if (nowDate.M==3){
		if(nowDate.D<(31-RTC.whatWeekday(31,3,0,nowDate.Y))) return 0;
		else return 1;
	}
	if (nowDate.M==10){
		if(nowDate.D<(31-RTC.whatWeekday(31,10,0,nowDate.Y))) return 1;
		else return 0;
	}
	return -1;
}

void updateSummerTime(){
	isTodaySummerTime=isSummerTime();
}



/*trucs pour summer time conversion
 * nowUTCTime=nowTime;
		if (isTodaySummerTime){
			if (nowUTCTime.H==0){
				nowUTCTime.H=23;
				if (nowDate.D==1){
					nowDate.D=pgm_read_byte(daysInMonths+nowDate.M-2);
					if (nowDate.M==1){
						nowDate.M=12;
						nowDate.Y-=1;
					}
					else nowDate.M -= 1;
				}
				else nowDate.D -= 1;
			}
			else nowUTCTime.H -= 1;
		}






	if (isTodaySummerTime){
		nowSummerTime.H +=1;
		if (nowSummerTime.H>23){
			nowSummerTime.H-=24;
		}
	}


		*/


#endif /* GDI_TIME_H_ */
