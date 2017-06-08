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
#include "trapapoul_text.h"


//lcd_I2C config
LiquidCrystal_I2C lcd(LCD_ADDRESS,16,2);

volatile int8_t latitudeNord=45;
volatile int16_t longitudeOuest=-6;

//RTC

//timer1config
volatile bool blink=0;

//boutons config
#define BPUP 0
#define BPDW 1
#define BPOK 2
#define TIMEOUT 50
#define ERROR 66
volatile unsigned long lastPush[3];
volatile bool buttonState[3];
volatile bool buttonPushed[3];

volatile unsigned long topTimeout;

uint8_t enterNumber(uint8_t *val,uint8_t min,uint8_t max,uint8_t col, uint8_t lin, uint8_t digit);
uint8_t enterTime(gdiTime_t*);
void clearButtons();
uint8_t waitButton();
bool isTimeValid(gdiTime_t*);
bool isDateValid(gdiDate_t*);
void printDateLCD ();
void printTimeLCD (gdiTime_t,bool);
void julianTranslate(uint8_t*hour,uint8_t*minute,uint8_t*second, double);
uint8_t enterGPS (int*, int*);
uint8_t enterDepartement ();
void uploadChar (uint8_t location, const uint8_t charmap[]);

void lcdClearLine(bool line){
	lcd.setCursor(0,line);
	for (uint8_t i=0;i<16;++i){
		lcd.write(' ');
	}
}

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
	if ((date->M)>12){
		return 0;
	}
	if (RTC.isLeapYear(0,date->Y)&&((date->M==2)&&(date->D==29))){
		return 1;
	}
	if ((date->D)>pgm_read_byte(daysInMonths+(date->M)-1)){
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

#define LCDON lcd.backlight();lcd.display()
#define LCDOFF lcd.noBacklight();lcd.noDisplay()

void lcdPrintLine(const uint8_t text[16],bool line){
	lcd.setCursor(0,line);
	for(uint8_t i=0;i<16;++i){
		lcd.write((pgm_read_byte(&text[i])));
	}
}

void lcdPrintScreen(const uint8_t ptr[2][16]){
	lcdPrintLine(ptr[0],0);
	lcdPrintLine(ptr[1],1);
}

void printNumberFixedDigits(uint8_t val, uint8_t digit){
	if (digit==3){
		lcd.write((val/100)+'0');
	}
	if (digit>=2){
		lcd.write(((val%100)/10)+'0');
	}
	lcd.write((val%10)+'0');
}

uint8_t enterNumber(uint8_t *val,uint8_t min,uint8_t max,
		uint8_t col, uint8_t lin, uint8_t digit/*,bool print0*/){
	//init du timer1 et de son interrupt de clignotage
	topTimeout=millis();
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
		if ((millis()-topTimeout)>BUTTON_TIMEOUT){
			Timer1.stop();
			Timer1.detachInterrupt();
			return TIMEOUT;
		}
		lcd.setCursor(col,lin);
		if (blink){
			printNumberFixedDigits(*val,digit);
		}else{
			for(uint8_t i=0;i<digit;++i){
				lcd.write(' ');
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
			topTimeout=millis();
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
			topTimeout=millis();
		}
		if(buttonPushed[BPOK]==1){
			buttonPushed[BPOK]=0;
			printNumberFixedDigits(*val,digit);
			break;
		}
	}
	Timer1.stop();
	Timer1.detachInterrupt();
	return 0;
}

void lcdPrintDate (gdiDate_t *date){//print date on LCD (jj/mm/yy)
	lcd.write((date->D/10)+'0');
	lcd.write((date->D%10)+'0');
	lcd.write('/');
	lcd.write((date->M/10)+'0');
	lcd.write((date->M%10)+'0');
	lcd.write('/');
	lcd.write('2');
	lcd.write('0');
	lcd.write((date->Y/10)+'0');
	lcd.write((date->Y%10)+'0');

}

void lcdPrintTime (gdiTime_t *time,bool printSec){//create string with time (hh:mm:ss)
	lcd.write((time->H/10)+'0');
	lcd.write((time->H%10)+'0');
	lcd.write(':');
	lcd.write((time->M/10)+'0');
	lcd.write((time->M%10)+'0');
	if(printSec){
		lcd.write(':');
		lcd.write((time->S/10)+'0');
		lcd.write((time->S%10)+'0');
	}
}

uint8_t enterTime(gdiTime_t *time,bool sec){

	lcdClearLine(1);
	lcd.setCursor(0,1);
	lcdPrintTime(time,sec);
	if(enterNumber(&(time->H),0,23,0,1,2)==TIMEOUT){
		return TIMEOUT;
	}
	if(enterNumber(&(time->M),0,59,3,1,2)==TIMEOUT){
		return TIMEOUT;
	}
	if (sec){
		if(enterNumber(&(time->S),0,59,6,1,2)==TIMEOUT){
			return TIMEOUT;
		}
	}

	//check validity
	if(isTimeValid(time)){	//check date validity
		return 0;
	}
		return ERROR;
}

uint8_t enterDate(gdiDate_t *date){

	lcdClearLine(1);
	lcd.setCursor(0,1);
	lcdPrintDate(date);
	if(enterNumber(&(date->D),1,31,0,1,2)==TIMEOUT){
		return TIMEOUT;
	}
	if(enterNumber(&(date->M),1,12,3,1,2)==TIMEOUT){
		return TIMEOUT;
	}
	if(enterNumber(&(date->Y),0,99,8,1,2)==TIMEOUT){
		return TIMEOUT;
	}

	//check validity
	if(isDateValid(date)==0){
		lcdPrintLine(DATE_NON_VALIDE,0);
		delay(2000);
		return ERROR;
	}
	return 0;
}

uint8_t enterDepartement (){
	uint8_t dpt=38;
	lcdPrintLine(ENTREZ_VOTRE,0);
	lcdPrintLine(DEPARTEMENT,1);
	if(enterNumber(&dpt,1,95,13,1,2)==TIMEOUT){
		return TIMEOUT;
	}
	latitudeNord=departement[((dpt-1)*2)];
	longitudeOuest=departement[((dpt*2)-1)];
	return 0;
}

uint8_t enterGPS (int8_t *latN, int16_t *lonO){

	bool latNS=1; //used also as lonEO
	uint8_t newlat; //used also as newlon
	bool lonPointer=0;


	lcdPrintLine(LATITUDE_NORD,0);
	lcdPrintLine(LATITUDE_SUD,1);

LATLON_LABEL:
	//init du timer1 et de son interrupt de clignotage
	Timer1.initialize(500000);
	Timer1.attachInterrupt(interrupt_blinker);
	Timer1.start();
	if (lonPointer==1){
		lcdPrintLine(LONGITUDE_OUEST,0);
		lcdPrintLine(LONGITUDE_EST,1);
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
			lcd.write(' ');
			lcd.setCursor(15,1);
			lcd.write(' ');
		}
		if(buttonPushed[BPUP]==1){
			clearButtons();
			latNS=1;
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
			lcd.setCursor(15,1);
			lcd.write(' ');
		}
		if(buttonPushed[BPDW]==1){
			clearButtons();
			latNS=0;
			lcd.setCursor(15,1);
			lcd.write(CHECK_CHAR);
			lcd.setCursor(15,0);
			lcd.write(' ');
		}
		if(buttonPushed[BPOK]==1){
			clearButtons();
			Timer1.stop();
			Timer1.detachInterrupt();
			break;
		}
	}

	if (latNS==1 && lonPointer==0){
		lcdPrintLine(LATITUDE_NORD,0);
		if ((*latN>=0 ))
			newlat=*latN;
		else
			newlat=0;
	}
	if(latNS==0 && lonPointer==0){
		lcdPrintLine(LATITUDE_SUD,0);
		if (*latN<=0)
			newlat=(*latN)*(-1);
		else
			newlat=0;
	}
	if (latNS==1 && lonPointer==1){
		lcdPrintLine(LONGITUDE_OUEST,0);
		if ((*lonO>=0 ))
			newlat=*lonO;
		else
			newlat=0;
	}
	if(latNS==0 && lonPointer==1){
		lcdPrintLine(LONGITUDE_EST,0);
		if (*lonO<=0)
			newlat=(*lonO)*(-1);
		else
			newlat=0;
	}

	lcdClearLine(1);
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
	return 0;
}

#endif /* TRAPAPOUL_UI_H_ */
