/*
 * trapapoul_UI.h
 *
 *  Created on: 22 juil. 2016
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_UI_H_
#define TRAPAPOUL_UI_H_

#include <LiquidCrystal_I2C.h>
#include "GDI_time.h"
#include <TimerOne.h>
#include "trapapoul_config.h"


//lcd_I2C config
LiquidCrystal_I2C lcd(LCD_ADDRESS,16,2);

int8_t latitudeNord=45;
int16_t longitudeOuest=-6;

//RTC

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

unsigned long topTimeout;

void enterNumber(uint8_t *val,uint8_t min,uint8_t max,uint8_t col, uint8_t lin, uint8_t digit);
void enterTime(gdiTime_t*);
void clearButtons();
uint8_t waitButton();
bool isTimeValid(gdiTime_t*);
bool isDateValid(gdiDate_t*);
void printDateLCD ();
void printTimeLCD (gdiTime_t,bool);
void julianTranslate(uint8_t*hour,uint8_t*minute,uint8_t*second, double);
void enterGPS (int*, int*);
void enterDepartement ();
void uploadChar (uint8_t location, const uint8_t charmap[]);

void lcdClearLine(){lcd.print(F("                "));}

void interrupt_blinker(void)
{
	blink=!blink;
}

void uploadChar (uint8_t location, const uint8_t charmap[]){
	uint8_t lcdchar[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
	for (uint8_t k = 0; k < 7; k++)
	  {
		lcdchar[k] = pgm_read_word_near(charmap+k);
		lcd.createChar(location, lcdchar);
	  }
}

bool isDateValid(gdiDate_t *date){
	if (date->M>12){
		return 0;
	}
	if ((date->M==2)&&(!LEAP_YEAR(date->Y))&&(date->D==29)){
		return 1;
	}
	if (date->D>daysInMonths[(date->M)-1]){
		return 0;
	}
	return 1;
}

bool isTimeValid(gdiTime_t *time){
	if (time->S>59 || time->M>59 ||time->H>23){
		return 0;
	}
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
	while((millis()-topTimeout) < BUTTON_TIMEOUT){
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
		//if((millis()-topTimeout) > BUTTON_TIMEOUT)
		//	return TIMEOUT;
	}
	return TIMEOUT;
}

void enterNumberold(uint8_t *val,uint8_t min,uint8_t max,
		uint8_t col, uint8_t lin, uint8_t digit/*,bool print0*/){
	//init du timer1 et de son interrupt de clignotage
	unsigned long blinkerTime;
	Timer1.initialize(500000);
	Timer1.attachInterrupt(interrupt_blinker);
	Timer1.start();
	blink=1;
	clearButtons();

	while(1){
		blinkerTime = millis();

		if (blink==1){
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print(F("0"));
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print(F("0"));
			lcd.print(*val);
		}else{
			lcd.setCursor(col,lin);
			lcd.print(F(" "));
			if (digit>=2)
				lcd.print(F(" "));
			if (digit==3)
				lcd.print(F(" "));
		}
		if((buttonState[BPUP]==1)&&(*val<max)){
			*val+=1;
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print(F("0"));
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print(F("0"));
			lcd.print(*val);
		}
		if((buttonState[BPDW]==1)&&(*val>min)){
			*val-=1;
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print(F("0"));
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print(F("0"));
			lcd.print(*val);
		}
		if(buttonPushed[BPOK]==1){
			lcd.setCursor(col,lin);
			if((*val<100) && (digit==3)/* && (print0==1)*/)
				lcd.print(F("0"));
			if ((*val<10) && (digit>=2)/* && (print0==1)*/)
				lcd.print(F("0"));
			lcd.print(*val);
			Timer1.stop();
			Timer1.detachInterrupt();
			return;
		}
		while ((millis()-blinkerTime)<BLINK_HALF_PERIOD);
	}
	Timer1.stop();

}

void printNumberFixedDigits(uint8_t val, uint8_t digit){
	if((val<100) && (digit==3)/* && (print0==1)*/){
		lcd.print(F("0"));
	}
	if ((val<10) && (digit>=2)/* && (print0==1)*/){
		lcd.print(F("0"));
	}
	lcd.print(val);
}

void enterNumber(uint8_t *val,uint8_t min,uint8_t max,
		uint8_t col, uint8_t lin, uint8_t digit/*,bool print0*/){
	//init du timer1 et de son interrupt de clignotage
	if(*val<min ||*val>max){
		*val=min;
	}
	Timer1.initialize(500000);
	Timer1.attachInterrupt(interrupt_blinker);
	Timer1.start();
	blink=1;
	clearButtons();
	topTimeout=millis();
	while(1){
		lcd.setCursor(col,lin);
		if (blink){
			printNumberFixedDigits(*val,digit);
		}else{
			for(uint8_t i=0;i<digit;++i){
				lcd.print(F(" "));
			}
		}
		lcd.setCursor(col,lin);

		if((buttonPushed[BPUP]==1)&&(*val<max)){
			*val+=1;
			Timer1.restart();
			lcd.setCursor(col,lin);
			blink=1;
			printNumberFixedDigits(*val,digit);
			while(buttonState[BPUP] && blink);
			while(buttonState[BPUP] && (*val<max)){
				*val+=1;
				lcd.setCursor(col,lin);
				printNumberFixedDigits(*val,digit);
				delay(FAST_BUTTON);
			}
			Timer1.restart();
			topTimeout=millis();
			blink=0;
			buttonPushed[BPUP]=0;
		}
		if((buttonPushed[BPDW]==1)&&(*val>min)){
			*val-=1;
			Timer1.restart();
			lcd.setCursor(col,lin);
			blink=1;
			printNumberFixedDigits(*val,digit);
			while(buttonState[BPDW] && blink);
			while(buttonState[BPDW] && *val>min){
				*val-=1;
				lcd.setCursor(col,lin);
				printNumberFixedDigits(*val,digit);
				delay(FAST_BUTTON);
			}
			Timer1.restart();
			topTimeout=millis();
			blink=0;
			buttonPushed[BPDW]=0;
		}
		if(buttonPushed[BPOK]==1 || (millis()-topTimeout)>BUTTON_TIMEOUT){
			buttonPushed[BPOK]=0;
			break;
		}
	}
	Timer1.stop();
	Timer1.detachInterrupt();
}




void enterTime(gdiTime_t *time){

	lcd.setCursor(0,1);
	if (time->H<10)
		lcd.print(F("0"));
	lcd.print(time->H);
	lcd.print(F(":"));
	if (time->M<10)
		lcd.print(F("0"));
	lcd.print(time->M);
	lcd.print(F(":"));
	if (time->S<10)
		lcd.print(F("0"));
	lcd.print(time->S);
	lcd.print(F("        "));
	enterNumber(&(time->H),0,23,0,1,2);
	enterNumber(&(time->M),0,59,3,1,2);
	enterNumber(&(time->S),0,59,6,1,2);


	//check validity
	if(isTimeValid(time)){	//check date validity
		//update values
	}else{
		lcd.setCursor(0,0);
		lcd.print(F("HEURE NON VALIDE"));
		delay(1000);
		lcd.setCursor(0,0);
		lcdClearLine();
		delay(500);
		lcd.setCursor(0,0);
		lcd.print(F("HEURE NON VALIDE"));
		delay(1000);
	}
}

void enterDate(gdiDate_t *date){

	lcd.setCursor(0,1);
	if (date->D<10)
		lcd.print(F("0"));
	lcd.print(date->D);
	lcd.print(F("/"));
	if (date->M<10)
		lcd.print(F("0"));
	lcd.print(date->M);
	lcd.print(F("/20"));
	lcd.print(date->Y);
	lcd.print(' ');
	enterNumber(&(date->D),1,31,0,1,2);
	enterNumber(&(date->M),1,12,3,1,2);
	enterNumber(&(date->Y),0,99,8,1,2);



	//check validity
	if(isDateValid(date)){	//check date validity
		//update values

		lcd.setCursor(0,0);
		lcd.print(F("DATE VALIDE     "));
		delay(1000);
	}else{
		lcd.setCursor(0,0);
		lcd.print(F("DATE NON VALIDE "));
		delay(1000);
		lcd.setCursor(0,0);
		lcdClearLine();
		delay(500);
		lcd.setCursor(0,0);
		lcd.print(F("DATE NON VALIDE "));
		delay(1000);
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

void enterGPS (int8_t *latN, int16_t *lonO){

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
			lcd.print(F(" "));
			lcd.setCursor(15,1);
			lcd.print(F(" "));
		}
		if(buttonPushed[BPUP]==1){
			clearButtons();
			latNS=1;
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
			lcd.setCursor(15,1);
			lcd.print(F(" "));
		}
		if(buttonPushed[BPDW]==1){
			clearButtons();
			latNS=0;
			lcd.setCursor(15,1);
			lcd.write(CHECK_CHAR);
			lcd.setCursor(15,0);
			lcd.print(F(" "));
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


void printDateLCD (){//print date on LCD (jj/mm/yy)
	RTC.getDateTime();
	if(RTC.getDay()<10)
		lcd.print('0');
	lcd.print(RTC.getDay());
	lcd.print('/');
	if(RTC.getMonth()<10)
		lcd.print('0');
	lcd.print(RTC.getMonth());
	lcd.print('/');
	if((RTC.getYear())<10)
		lcd.print('0');
	lcd.print(RTC.getYear());

}

void printTimeLCD (gdiTime_t time,bool printSec){//create string with time (hh:mm:ss)
	if(time.H<10)
		lcd.print('0');
	lcd.print(time.H);
	lcd.print(':');
	if(time.M<10)
		lcd.print('0');
	lcd.print(time.M);
	if(printSec){
		lcd.print(':');
		if(time.S<10)
			lcd.print('0');
		lcd.print(time.S);
	}
}

#define SECS_PER_MIN 60
#define SECS_PER_HOUR 3600
#define SECS_PER_DAY 86400
void julianTranslate(uint8_t *hour,uint8_t *minute,uint8_t *second, double julian){
	//fill in hour minute and second from a -0.5<julian<0.5
	julian+=0.5;
	uint32_t julianSec =julian*SECS_PER_DAY;
	*second=(julianSec%SECS_PER_MIN);
	*minute=(julianSec%SECS_PER_HOUR)/SECS_PER_MIN;
	*hour=julianSec/SECS_PER_HOUR;
}


#endif /* TRAPAPOUL_UI_H_ */
