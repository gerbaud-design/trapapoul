/*
 * trapapoul_UI.h
 *
 *  Created on: 22 juil. 2016
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_UI_H_
#define TRAPAPOUL_UI_H_

#include <LiquidCrystal_I2C.h>
#include <Time.h>
//#include <Rtc_Pcf8563.h>
#include <TimerOne.h>
#include "trapapoul_config.h"



typedef struct  {
  uint8_t S;
  uint8_t M;
  uint8_t H;
} 	gdiTime_t;



//lcd_I2C config
LiquidCrystal_I2C lcd(LCD_ADDRESS,16,2);

int latitudeNord=45;
int longitudeOuest=-6;

//RTC
tmElements_t timeElements;
//extern Rtc_Pcf8563 RTC;

//timer1config
volatile bool blink=0;

//boutons config
#define BPUP 0
#define BPDW 1
#define BPOK 2
#define TIMEOUT 50
volatile unsigned long lastPush[3];
volatile bool buttonState[3];
volatile bool buttonPushed[3];

//used in isDateValid
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};

unsigned long topTimeout;

void enterNumber(uint8_t *val,uint8_t min,uint8_t max,uint8_t col, uint8_t lin, uint8_t digit);
void enterTime(tmElements_t*);
void clearButtons();
uint8_t waitButton();
bool isTimeValid(tmElements_t*);
bool isDateValid(tmElements_t*);
String printDate ();
String printTime (gdiTime_t,bool);
void julianTranslate(uint8_t*hour,uint8_t*minute,uint8_t*second, double);
void enterGPS (int*, int*);
void enterDepartement ();
void uploadChar (uint8_t location, const uint8_t charmap[]);

void lcdClearLine(){lcd.print(F("                "));}

void interrupt_blinker(void)
{
	if (blink==0) {
		blink=1;
	} else {
		blink=0;
	}
}

void uploadChar (uint8_t location, const uint8_t charmap[]){
	uint8_t lcdchar[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
	for (uint8_t k = 0; k < 7; k++)
	  {
		lcdchar[k] = pgm_read_word_near(charmap+k);
		lcd.createChar(location, lcdchar);
	  }
}

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

void clearButtons(void){
    buttonPushed[BPUP]=0;
    buttonPushed[BPOK]=0;
    buttonPushed[BPDW]=0;
}

uint8_t waitButton()
{
	topTimeout=millis();
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
	blink=1;
	clearButtons();

	//print number in the right place
//	lcd.setCursor(col,lin);
//	if((*val<100) && (digit==3)/* && (print0==1)*/)
//		lcd.print('0');
//	if((*val<10) && (digit>=2)/* && (print0==1)*/)
//		lcd.print('0');
//	lcd.print(*val);
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

ENTER_TIME:
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
	if(!isTimeValid(te)){	//check date validity

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

void enterDate(tmElements_t *te){

ENTER_DATE :
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

void updateTime (){
/*RTC.getDateTime();
timeElements.Day=RTC.getDay();
timeElements.Month=RTC.getMonth();
timeElements.Year=RTC.getYear()+30;
timeElements.Minute=RTC.getMinute();
timeElements.Hour=RTC.getHour();
timeElements.Second=RTC.getSecond();
*/
}

String printDate (){//create string with date (jj/mm/yy)
	String date="";
	//date+=("date (jj/mm/yyyy) : ");
	updateTime();
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

String printTime (gdiTime_t time){//create string with time (hh:mm:ss)
	String printed="";

	//time+=("time (hh:mm:ss) : ");
	updateTime();
	if(time.H<10)
		printed+=("0");
	printed+=(time.H);
	printed+=(":");
	if(time.M<10)
		printed+=("0");
	printed+=(time.M);
	/*if(sec){
		printed+=(":");
		if(timeElements.Second<10)
			printed+=("0");
		printed+=(timeElements.Second);
	}*/
	return printed;
}

void julianTranslate(uint8_t *hour,uint8_t *minute,uint8_t *second, double julian){
	//fill in hour minute and second from a -0.5<julian<0.5
	julian+=0.5;
	uint32_t julianSec =julian*SECS_PER_DAY;
	*second=(julianSec%SECS_PER_MIN);
	*minute=(julianSec%SECS_PER_HOUR)/SECS_PER_MIN;
	*hour=julianSec/SECS_PER_HOUR;
}


#endif /* TRAPAPOUL_UI_H_ */
