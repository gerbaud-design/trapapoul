/*
 * protopoulie.cpp
 *
 *  Created on: 22 avr. 2016
 *      Author: guiguilours
 */

#include <Arduino.h>
#include <Rtc_Pcf8563.h>
//#include <Time.h>
#include <EEPROM.h>
#include "ERRcodes.h"
#include "departementGPS.h"
#include "trapapoul_UI.h"
#include "trapapoul_config.h"
#include "trapapoul_motor.h"
#include "ephemeride.h"
#include "sleep.h"
#include "trapapoul_expert.h"

extern volatile int motorPosition;
extern volatile unsigned long topTimeout;
extern volatile unsigned long lastPush[3];
extern volatile bool buttonState[3];
extern volatile bool buttonPushed[3];

extern int8_t latitudeNord;
extern int16_t longitudeOuest;

extern uint8_t resetSource;

//config variables
#define SOLEIL 0
#define FIXE 1
#define MINIMUM 2
uint8_t openMode=SOLEIL;
uint8_t closeMode=SOLEIL;
gdiTime_t openUTCTime,closeUTCTime;
uint8_t closeDelay=DEFAULT_CLOSE_DELAY;

enum wakeUpSource_t{
	wu_button,
	wu_alarm,
	wu_unknown
}wakeUpSource=wu_unknown;

enum doorState_t {
	ds_unknown,
	ds_opened,
	ds_closed,
	ds_earlyOpened,
	ds_earlyClosed,
	ds_forceOpened,
	ds_forceClosed
}doorState=ds_unknown;
uint16_t anaRead;

uint8_t configured=0;
int16_t positionHaute = 0;
uint16_t nbCycles;


float lever,meridien,coucher;
volatile bool AlarmTriggered=0;
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


//functions declaration


uint8_t installationTrappe();
uint8_t manualMoveMotor();
void updateOpenTime();
void updateCloseTime();
uint8_t userInterface();
uint8_t menu();
//expert functions
uint8_t expertCharge();
uint8_t expertCyclage();
uint8_t expertEphemerides();
uint8_t expertDebug();
uint8_t expertAlarm();

//interrupt routine for pin change of port D
ISR (PCINT2_vect){ // handle pin change interrupt for D0 to D7 here
	wakeUpSource=wu_button;
	if(!digitalRead(pinAlarm))
		AlarmTriggered=1;
	bool newState[3];
	newState[BPOK]=(!(digitalRead(pinBPOK)));
	newState[BPDW]=(!(digitalRead(pinBPDW)));
	newState[BPUP]=(!(digitalRead(pinBPUP)));

	if (newState[BPOK]!=buttonState[BPOK]){
		topTimeout=millis();
		if (newState[BPOK]==1 && ((millis()-lastPush[BPOK])>DEBOUNCE)){
			buttonPushed[BPOK]=1;
			buttonState[BPOK]=1;
		}
		if(newState[BPOK]==0){
			buttonState[BPOK]=0;
			lastPush[BPOK]=millis();
		}
	}
	if (newState[BPDW]!=buttonState[BPDW]){
		topTimeout=millis();
		if (newState[BPDW]==1 && ((millis()-lastPush[BPDW])>DEBOUNCE)){
			buttonPushed[BPDW]=1;
			buttonState[BPDW]=1;
		}
		if(newState[BPDW]==0){
			buttonState[BPDW]=0;
			lastPush[BPDW]=millis();
		}
	}
	if (newState[BPUP]!=buttonState[BPUP]){
		topTimeout=millis();
		if (newState[BPUP]==1 && ((millis()-lastPush[BPUP])>DEBOUNCE)){
			buttonPushed[BPUP]=1;
			buttonState[BPUP]=1;
		}
		if(newState[BPUP]==0){
			buttonState[BPUP]=0;
			lastPush[BPUP]=millis();
		}
	}

}

inline void clearInterrupts() {	EIFR=0x03;\
								PCIFR=0x07;}

// interrupt routine for pin2 alarm.
void interrupt_0 () {
	//handle rtc alarm
	sleep_disable();
	detachInterrupt(0);
	wakeUpSource=wu_alarm;
}

// interrupt routine for pin3 bpok
void interrupt_1 () {
	sleep_disable();
	detachInterrupt(1);
	wakeUpSource=wu_button;

}

//The setup function is called once at startup of the sketch
void setup()
{

//setup du sleep mode
	WDTCSR=0x00;//disable watchdog just in case
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

//initialisation of analog inputs
	analogReference(EXTERNAL);

//setup du lcd
	activateVpp();
	LCDON;
	lcdPrintLine(SYSTEMRESET,0);
	lcd.setCursor(0,1);
	lcd.write(resetSource%10+'0');
	delay(2000);
	lcd.clear();
	LCDOFF;
	deactivateVpp();

//Init of RTC
	//RTC.set(SECS_YR_2000,CLOCK_ADDRESS);
	RTC.clearAlarm();
	RTC.clearTimer();
	RTC.clearSquareWave();
	pinMode(pinAlarm, INPUT);
	AlarmTriggered=0;
	attachInterrupt(digitalPinToInterrupt(pinAlarm), interrupt_0 , FALLING);

	//initialize motor
	motorInit();

	//init charge
	pinMode(pinChargeOff,OUTPUT);
	digitalWrite(pinChargeOff,1);//turn charge ON when =1

//setup des boutons
	pinMode(pinBPUP, INPUT);
	pinMode(pinBPDW, INPUT);
	pinMode(pinBPOK, INPUT);
	//digitalWrite(pinBPUP,1);
	//digitalWrite(pinBPDW,1);
	//digitalWrite(pinBPOK,1);


//desactive int1  et int0 (or loop in int1 while pbok pushed)
	detachInterrupt(0);
	detachInterrupt(1);

//setup des interuptions boutons
	*digitalPinToPCMSK(pinBPUP) |= bit (digitalPinToPCMSKbit(pinBPUP));  // enable pin
	*digitalPinToPCMSK(pinBPDW) |= bit (digitalPinToPCMSKbit(pinBPDW));  // enable pin
	*digitalPinToPCMSK(pinBPOK) |= bit (digitalPinToPCMSKbit(pinBPOK));  // enable pin
	PCIFR  |= bit (digitalPinToPCICRbit(pinBPOK)); // clear any outstanding interrupt
	PCICR  |= bit (digitalPinToPCICRbit(pinBPOK)); // enable interrupt for the group
	delay(5);
	lastPush[BPOK]=millis();
	lastPush[BPUP]=millis();
	lastPush[BPDW]=millis();
	clearButtons();
	configured=0;

}


#define DISABLE_ADC ADCSRA &= ~(1 << ADEN)
#define ENABLE_ADC ADCSRA |= (1 << ADEN)
#define DISABLE_ACOMP ACSR &= ~(1 << ACD)

// The loop function is called in an endless loop
void loop()
{

	activateVpp();
	RTC.getDateTime();
	switch(wakeUpSource){
	case wu_unknown:
		LCDON;
		lcdPrintLine(UNKNOWN_WAKE_UP,0);
		delay(500);
		LCDOFF;
		//nobreak
	case wu_button:
		lcd.clear();
		LCDON;
		clearButtons();
		updateSummerTime();
		if(configured<3){
			installationTrappe();
		}else{
			userInterface();
		}
		LCDOFF;
		lcd.clear();
		updateCloseTime();
		updateOpenTime();
		//nobreak
	case wu_alarm:
		LCDON;
		lcdPrintLine(ALARMtxt,0);
		delay(1000);
		LCDOFF;
		if(configured>=3){
			updateDateTime();
			if(RTC.getVoltLow()){
				LCDON;
				lcdPrintLine(DEBUG,0);
				delay(10000);
				LCDOFF;
			}

			if((((RTC.getHour()>openUTCTime.H)||\
					((RTC.getHour()==openUTCTime.H)&&(RTC.getMinute()>=openUTCTime.M)))\
					&&((RTC.getHour()<closeUTCTime.H)||\
					((RTC.getHour()==closeUTCTime.H)&&(RTC.getMinute()<closeUTCTime.M))))==1){
				if(doorState==ds_earlyOpened){
					doorState=ds_opened;
				}
				if(doorState==ds_closed){
					activateMotor();
					motorGoTo(positionHaute);//open door
					deactivateMotor();
					doorState=ds_opened;
				}
				RTC.setAlarm(closeUTCTime.M, closeUTCTime.H, 99, 99);
				//set alarm to closeTime
			}
			else{
				if(doorState==ds_earlyClosed){
					doorState=ds_closed;
				}
				if(doorState==ds_opened){
					activateMotor();
					motorGoTo(0);//close door
					deactivateMotor();
					doorState=ds_closed;
				}
				RTC.setAlarm(openUTCTime.M, openUTCTime.H, 99, 99);
				//set alarm to openTime
			}
		}
	}

	//shutdown everything
	deactivateVpp();
	DISABLE_ADC;
	DISABLE_ACOMP;
	WDTCSR=0x00;//disable watchdog just in case
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	cli();
	clearInterrupts();
	attachInterrupt(0,interrupt_0,FALLING);
	attachInterrupt(1,interrupt_1,FALLING);
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	/* wake up here */
	sleep_disable();
	clearButtons();
	ENABLE_ADC;
}

uint8_t userInterface()
{
	updateSummerTime();
	topTimeout=millis();
		while(1){
			//time display
			updateTime();
			lcd.setCursor(0,0);
			lcdPrintTime(&nowUTCTime,1);
			if(!resetSource) lcd.write('X');
			//battery voltage measurement
			anaRead=0;
			for(uint8_t i=0;i<64;++i){
				anaRead+=analogRead(pinMesVbat);
				delay(5);
			}
			anaRead /=ratioVbat;

			//battery voltage display

			lcd.setCursor(12,0);
			lcd.write((anaRead/100)+'0');
			lcd.write(',');
			lcd.write(((anaRead%100)/10)+'0');
			lcd.write((anaRead%10)+'0');

			//display dooring planning
			switch(doorState){
			case ds_opened:
			case ds_earlyOpened:
				lcdPrintLine(FERMERA_A,1);
				lcd.setCursor(10,1);
				lcdPrintTime(&closeUTCTime,0);
				break;
			case ds_closed:
			case ds_earlyClosed:
				lcdPrintLine(OUVRIRA_A,1);
				lcd.setCursor(10,1);
				lcdPrintTime(&openUTCTime,0);
				break;
			case ds_unknown:
				lcdPrintLine(PARAMETREZ_MOI,1);
				break;
			case ds_forceOpened:
				lcdPrintLine(OUVERT_INFINI,1);
				break;
			case ds_forceClosed:
				lcdPrintLine(FERME_INFINI,1);
				break;
			}
			//	lcd.setCursor(0,1);
			//	lcd.print(doorState);


			if(buttonPushed[BPOK]==1)
				switch(menu()){
				case ERROR:
					clearButtons();
					lcdPrintLine(ERREUR_MENU,0);
					lcdClearLine(1);
					delay(5000);
				case TIMEOUT:
					lcdPrintLine(TIMEOUTtxt,0);
					delay(200);
					return TIMEOUT;
				default:
					clearButtons();
					lcdClearLine(1);
				}
			if((millis()-topTimeout)>BUTTON_TIMEOUT)
					return TIMEOUT;
		}
/*
	MENU_OUVERTURE:
		clearButtons();
		lcdPrintScreen(menuText[0]);
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE;
		case BPDW:
			goto MENU_FERMETURE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_OUVERTURE_SOLEIL;
		default:
			goto MENU_ERROR;
		}

	MENU_OUVERTURE_SOLEIL:
		clearButtons();
		lcdPrintLine(OUVERTURE,0);
		lcdPrintLine(menuScheduleText[0],1);
		if (openMode==SOLEIL){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_SOLEIL;
		case BPDW:
			goto MENU_OUVERTURE_FIXE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (openMode == SOLEIL)
				goto MENU_OUVERTURE_ENREGISTREE;
			else{
				openMode=SOLEIL;
				goto MENU_OUVERTURE_SOLEIL;
			}
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_FIXE:
		clearButtons();
		lcdPrintLine(OUVERTURE,0);
		lcdPrintLine(menuScheduleText[1],1);
		if (openMode==FIXE){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_SOLEIL;
		case BPDW:
			goto MENU_OUVERTURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (openMode == FIXE)
				goto MENU_OUVERTURE_HEURE;
			else{
				openMode=FIXE;
				goto MENU_OUVERTURE_FIXE;
			}
		default:
			goto MENU_ERROR;
		}

	MENU_OUVERTURE_HEURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcdPrintLine(HEURE_OUVERTURE,0);
		lcd.setCursor(0,1);
		lcdPrintTime(&openTime,0);
		enterNumber(&openTime.H,0,23,0,1,2);
		enterNumber(&openTime.M,0,59,3,1,2);
		goto MENU_OUVERTURE_ENREGISTREE;

	MENU_OUVERTURE_MINIMUM:
		clearButtons();
		lcdPrintLine(OUVERTURE,0);
		lcdPrintLine(menuScheduleText[2],1);
		if (openMode==MINIMUM){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_FIXE;
		case BPDW:
			goto MENU_OUVERTURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (openMode == MINIMUM)
				goto MENU_OUVERTURE_ENREGISTREE;
			else{
				openMode=MINIMUM;
				goto MENU_OUVERTURE_MINIMUM;
			}
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_RETOUR:
		clearButtons();
		lcdPrintLine(RETOUR,0);
		lcdClearLine(1);
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_MINIMUM;
		case BPDW:
			goto MENU_OUVERTURE_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_OUVERTURE;
		default:
			goto MENU_ERROR;
		}

	MENU_OUVERTURE_ENREGISTREE:
		clearButtons();
		lcd.setCursor(0,0);
		lcdPrintLine(OUVERTURE,0);
		lcd.setCursor(0,1);
		lcdPrintLine(ENREGISTREE,1);
		updateOpenTime();
		//delay(2000);
		goto MENU;

	MENU_FERMETURE:
		clearButtons();
		lcdPrintScreen(menuText[menu_fermeture]);
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE;
		case BPDW:
			goto MENU_DATE_HEURE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_FERMETURE_SOLEIL;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_SOLEIL:
		clearButtons();
		lcdPrintLine(FERMETURE,0);
		lcdPrintLine(menuScheduleText[1],1);
		if (closeMode==SOLEIL){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_SOLEIL;
		case BPDW:
			goto MENU_FERMETURE_FIXE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (closeMode == SOLEIL)
				goto MENU_FERMETURE_DELAY;
			else{
				closeMode=SOLEIL;
				goto MENU_FERMETURE_SOLEIL;
			}
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_DELAY:
		clearButtons();
		lcdPrintLine(DELAI_FERMETURE,0);
		lcd.setCursor(0,1);
		enterNumber(&closeDelay,0,60,0,1,2);
		goto MENU_FERMETURE_ENREGISTREE;

	MENU_FERMETURE_FIXE:
		clearButtons();
		lcdPrintLine(FERMETURE,0);
		lcdPrintLine(menuScheduleText[1],1);
		if (closeMode==FIXE){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_SOLEIL;
		case BPDW:
			goto MENU_FERMETURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (closeMode == FIXE)
				goto MENU_FERMETURE_HEURE;
			else{
				closeMode=FIXE;
				goto MENU_FERMETURE_FIXE;
			}
		default:
			goto MENU_ERROR;
		}

		MENU_FERMETURE_HEURE:
			clearButtons();
			lcd.setCursor(0,0);
			lcdPrintLine(HEURE_FERMETURE,0);
			lcd.setCursor(0,1);
			lcdPrintTime(&closeTime,0);
			enterNumber(&closeTime.H,0,23,0,1,2);
			enterNumber(&closeTime.M,0,59,3,1,2);
			goto MENU_FERMETURE;

	MENU_FERMETURE_MINIMUM:
		clearButtons();
		lcdPrintLine(FERMETURE,0);
		lcdPrintLine(menuScheduleText[2],1);
		if (closeMode==MINIMUM){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_FIXE;
		case BPDW:
			goto MENU_FERMETURE_MINIMUM;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			if (closeMode == MINIMUM)
				goto MENU_FERMETURE_ENREGISTREE;
			else{
				closeMode=MINIMUM;
				goto MENU_FERMETURE_MINIMUM;
			}
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_RETOUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcdPrintLine(RETOUR,0);
		lcdClearLine(1);
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_MINIMUM;
		case BPDW:
			goto MENU_FERMETURE_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_FERMETURE;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_ENREGISTREE:
		clearButtons();
		lcdPrintLine(FERMETURE,0);
		lcdPrintLine(ENREGISTREE,1);
		updateCloseTime();
		//delay(2000);
		goto MENU;


	MENU_DATE_HEURE:
		clearButtons();
		lcdPrintLine(REGLAGE_DATE,0);
		lcdPrintLine(ET_HEURE,1);
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE;
		case BPDW:
			goto MENU_INSTALLATION;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			updateDateTime();
			lcdPrintLine(HEURE,0);
			enterTime(&nowTime,1);
			RTC.setTime(nowTime.H,nowTime.M,nowTime.S);
			lcdPrintLine(HEURE,0);
			lcdPrintLine(ENREGISTREE,1);
			delay(2000);
			lcd.clear();
			lcdPrintLine(DATE,0);
			enterDate(&nowDate);
			RTC.setDate(nowDate.D,0,nowDate.M,0,nowDate.Y);
			lcdPrintLine(DATE,0);
			lcdPrintLine(ENREGISTREE,1);
			RTC.clearVoltLow();
			updateCloseTime();
			updateOpenTime();
			delay(2000);
			goto MENU;
		default:
			goto MENU_ERROR;
		}

	MENU_INSTALLATION:
	clearButtons();
	lcdPrintLine(INSTALLATION,0);
	lcdPrintLine(DE_LA_TRAPPE,1);
	switch(waitButton()){
	case BPUP:
		goto MENU_DATE_HEURE;
	case BPDW:
		goto MENU_EXPERT;
	case TIMEOUT:
		goto MENU_TIMEOUT;
	case BPOK:
		installationTrappe();
		goto MENU_INSTALLATION;
	default:
		goto MENU_ERROR;
	}

	MENU_EXPERT:
		clearButtons();
		lcdPrintScreen(menuText[menu_expert]);
		lcdClearLine(1);
		switch(waitButton()){
		case BPUP:
			goto MENU_INSTALLATION;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_EXPERT_CHARGE;
		default:
			goto MENU_ERROR;
		}
*/



}
uint8_t SheduleSetting(bool opening){//(opening=0)=>closing
	uint8_t menuSchedulePointer=menu_schedule_soleil;
	while(1){
		clearButtons();
		lcdPrintLine(menuScheduleText[menuSchedulePointer],0);
		if ((opening && openMode==menuSchedulePointer)||(!opening && closeMode==menuSchedulePointer)){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			if (menuSchedulePointer>0){
				--menuSchedulePointer;
			}
			continue;
		case BPDW:
			if (menuSchedulePointer<menu_quitter){
				++menuSchedulePointer;
			}
			continue;
		case TIMEOUT:
			return TIMEOUT;
		case BPOK:
			if (menuSchedulePointer==menu_schedule_soleil){
				if(opening){
					updateOpenTime();
				}else{
					lcdPrintLine(DELAI,1);
					enterNumber(&closeDelay,0,240,8,1,3);
					updateCloseTime();
				}

			}
			if (menuSchedulePointer==menu_schedule_fixe){
				gdiTime_t *toSet;
				if(opening){
					toSet=&openUTCTime;
				}else{
					toSet=&closeUTCTime;
				}
				lcdPrintLine(HEURE,1);
				lcd.setCursor(6,1);
				lcd.write(':');
				enterTime(toSet,0);
			}
			updateOpenTime();
			//update openclose
			return 0;
		default:
			return ERROR;
		}
	}
}




uint8_t menuOuverture(){
	return SheduleSetting(1);

}
uint8_t menuFermeture(){
	return SheduleSetting(1);
}
uint8_t reglageDateHeure(){
	updateTime();
	lcd.clear();
	lcdPrintLine(DATE,0);
	if(enterDate(&nowDate)==0){
		RTC.setDate(nowDate.D,1,nowDate.M,0,nowDate.Y);
		lcdPrintLine(DATE,0);
		lcdPrintLine(ENREGISTREE,1);
		updateSummerTime();
		//delay(500);
		lcd.clear();
	}else{
		return ERROR;
	}
	lcd.clear();
	lcdPrintLine(HEURE,0);
	if(enterTime(&nowUTCTime,1)==0){

		RTC.setDateTime(nowDate.D,RTC.whatWeekday(nowDate.D,nowDate.M,0,nowDate.Y),nowDate.M,0,nowDate.Y,nowUTCTime.H,nowUTCTime.M,nowUTCTime.S);
		lcdPrintLine(HEURE,0);
		lcdPrintLine(ENREGISTREE,1);
		//delay(500);
		lcd.clear();
	}else{
		return ERROR;
	}

	RTC.clearVoltLow();
	updateCloseTime();
	updateOpenTime();
	return 0;
}

uint8_t (*menuExpertFunctionPointers[menu_expert_quitter])()={expertCharge,expertCyclage,expertEphemerides,expertDebug,expertAlarm};
uint8_t menuExpert(){
	uint8_t menuExpertPointer=0;
	while(1){
		clearButtons();
		lcd.clear();
		lcdPrintLine(menuExpertText[menuExpertPointer],0);
		switch(waitButton()){
		case BPUP:
			if (menuExpertPointer>0){
				--menuExpertPointer;
			}
			continue;
		case BPDW:
			if (menuExpertPointer<menu_expert_quitter){
				++menuExpertPointer;
			}
			continue;
		case TIMEOUT:
			return TIMEOUT;
		case BPOK:
			if(menuExpertPointer==menu_expert_quitter){
				return 0;
			}
			switch ((*menuExpertFunctionPointers[menuExpertPointer])()){
			case TIMEOUT:
				return TIMEOUT;
				break;
			case ERROR:
				return ERROR;
				break;
			default:
				break;
			}
			continue;
		default:
			return ERROR;
		}
	}
}

uint8_t (*menuFunctionPointers[menu_quitter])()={menuOuverture,menuFermeture,reglageDateHeure,installationTrappe,menuExpert};

uint8_t menu(){
	uint8_t menuPointer=menu_ouverture;

	while(1){
		clearButtons();
		lcdPrintScreen(menuText[menuPointer]);
		switch(waitButton()){
		case BPUP:
			if (menuPointer>0){
				--menuPointer;
			}
			continue;
		case BPDW:
			if (menuPointer<menu_quitter){
				++menuPointer;
			}
			continue;
		case TIMEOUT:
			return TIMEOUT;
		case BPOK:
			if(menuPointer==menu_quitter){
				return 0;
			}
			switch ((*menuFunctionPointers[menuPointer])()){
			case TIMEOUT:
				return TIMEOUT;
				lcdPrintLine(TIMEOUTtxt,0);
				delay(200);
				break;
			case ERROR:
				return ERROR;
				lcdPrintLine(ERRORtxt,0);
				delay(200);
				break;
			default:
				break;
			}
			continue;
		default:
			return ERROR;
		}
	}
}

uint8_t reglageHauteur(){
	clearButtons();
	activateMotor();

	lcdPrintLine(REGLAGE,0);
	lcdPrintLine(POSITION_FERMEE,1);
	if(manualMoveMotor()==TIMEOUT){
		return TIMEOUT;
	}
	resetPosition();
	lcdPrintLine(POSITION_FERMEE,0);
	lcdPrintLine(ENREGISTREE,1);
	delay(MESSAGE_TIME);
	clearButtons();
	lcdPrintLine(REGLAGE,0);
	lcdPrintLine(POSITION_OUVERTE,1);
	if(manualMoveMotor()==TIMEOUT){
		return TIMEOUT;
	}
	positionHaute=motorPosition;
	EEPROM.put(EEPROM_POSOPEN,positionHaute);
	deactivateMotor();
	lcdPrintLine(POSITION_OUVERTE,0);
	lcdPrintLine(ENREGISTREE,1);
	delay(MESSAGE_TIME);
	doorState=ds_opened;
	updateCloseTime();
	return 0;
}

uint8_t reglageGPS(){
	return enterDepartement();

}

uint8_t installationTrappe(){
	topTimeout=millis();
	configured=0;
	switch(configured){
	case 0:
		if(reglageHauteur()!=0){
			return ERROR;
		}
		configured=1;
	case 1:
		if(reglageGPS()!=0){
			return ERROR;
		}
		configured=2;
	case 2:
		if(reglageDateHeure()!=0){
			return ERROR;
		}
		configured=3;
		updateOpenTime();
		updateCloseTime();
	}
	return 0;
}

void updateOpenTime(){
	updateDateTime();
	calculerEphemeride(nowDate.D,nowDate.M,nowDate.Y,longitudeOuest,latitudeNord,&lever,&meridien,&coucher);
	if (lever<-0.5) lever+=1;
	if (lever>=0.5) lever-=1;
	if (openMode==SOLEIL){
		julianTranslate(&openUTCTime.H,&openUTCTime.M,&openUTCTime.S,lever);//nowTime used only as trash result to limit variable number
	}
}

void updateCloseTime(){
	updateDateTime();
	calculerEphemeride(nowDate.D,nowDate.M,nowDate.Y,longitudeOuest,latitudeNord,&lever,&meridien,&coucher);
	if (coucher<-0.5) lever+=1;
	if (coucher>=0.5) lever-=1;
	if (closeMode==SOLEIL){
		julianTranslate(&closeUTCTime.H,&closeUTCTime.M,&closeUTCTime.S,coucher);//nowTime used only to limit variable number
		closeUTCTime.M+=closeDelay;
		while(closeUTCTime.M>59){
			closeUTCTime.H+=1;
			closeUTCTime.M-=60;
		}
	}
}

///expret functions:


uint8_t expertCharge(){
	clearButtons();
			/*pinMode(pinVppEn,OUTPUT);
			digitalWrite(pinVppEn,1);
			lcd.setCursor(0,0);
			lcd.print(F("CHARGE READY    "));
			lcd.setCursor(0,1);
			lcdClearLine();
			while(waitButton()!=BPOK);
			lcd.setCursor(0,0);
			lcd.print(F("CHARGE ON       "));
			//chargeStartTime=millis();
			digitalWrite(pinChargeOff,1);
			while((millis()-chargeStartTime)<3600000){
				float batVoltage;
				anaRead=0;
				for (i8_1=0;i8_1<16;i8_1++){
					anaRead += analogRead(pinMesVbat);
					delay(10);
				}
				batVoltage=anaRead*vrefVoltage*3.13/10240;
				lcd.setCursor(0,1);
				lcd.print(batVoltage);
				lcd.print('V');
				if (batVoltage>7.2) break;
				delay(1000);
			}*/
			digitalWrite(pinChargeOff,0);
			clearButtons();
			waitButton();
			return 0;
}

uint8_t expertCyclage(){

			activateMotor();
			lcd.setCursor(0,0);
			lcdPrintLine(CYCLAGE,0);
			nbCycles=0;
			while(1){
				motorGoTo(positionHaute);
				delay(10000);
				if(machineState==TIMEOUT)
					break;
				motorGoTo(0);
				if(machineState==TIMEOUT)
					break;
				nbCycles++;
				lcd.setCursor(10,1);
				lcd.print(nbCycles);
				if(nbCycles==NB_ITERATION)
					break;

				delay(10000);
			}
			if(machineState==TIMEOUT){
			}
			manualMoveMotor();
			deactivateMotor();
	return 0;
}


extern float lever,coucher,meridien;
uint8_t expertEphemerides(){
	gdiDate_t tmpDate={0,0,0};
	gdiTime_t tmpTime;
	while(1){
		lcd.clear();
		enterDate(&tmpDate);
		lcd.clear();
	//	calculerEphemeride(timeElements.Day,timeElements.Month,timeElements.Year,45,-5,&lever,&meridien,&coucher);
		calculerEphemeride(tmpDate.D,tmpDate.M,tmpDate.Y,longitudeOuest,latitudeNord,&lever,&meridien,&coucher);
	/*	if (lever<-0.5) lever+=1;
		if (lever>=0.5) lever-=1;
		if (coucher<-0.5) lever+=1;
		if (coucher>=0.5) lever-=1;*/
		lcd.setCursor(0,0);
		lcd.print(lever);
		lcd.setCursor(0,1);
		lcd.print(coucher);
		julianTranslate(&tmpTime.H,&tmpTime.M,&tmpTime.S,lever);
		lcd.setCursor(6,0);
		lcdPrintTime(&tmpTime,1);
		julianTranslate(&tmpTime.H,&tmpTime.M,&tmpTime.S,coucher);
		lcd.setCursor(6,1);
		lcdPrintTime(&tmpTime,1);
		while(buttonPushed[BPOK]==0){
			if (buttonPushed[BPDW]==1) continue;
		}
		break;
	}
	return 0;
}

uint8_t expertDebug(){


/*	activateMotor();
motorForward();
clearButtons();
lcd.setCursor(0,0);
lcd.print(F("MOTOR ON        "));
lcd.setCursor(0,1);
lcdClearLine();
while(1){
	anaRead=0;
	for(i8_1=0;i8_1<64;i8_1++){
		anaRead+=analogRead(pinMesImot);
		delay(2);
	}

	anaRead /=ratioImot;
	lcd.setCursor(10,0);
	lcd.print(anaRead);
	lcd.print('"mA");

	anaRead=0;
	for(i8_1=0;i8_1<64;i8_1++){
		anaRead+=analogRead(pinMesVbat);
		delay(2);
	}
	anaRead /=ratioVbat;
	lcd.setCursor(10,1);
	lcd.print(anaRead);
	lcd.setCursor(11,1);
	lcd.print(anaRead);
	lcd.print('V');
	lcd.setCursor(11,1);
	lcd.print(',');

	if (buttonPushed[BPUP]){
		motorStop();
		motorForward();
		clearButtons();
	}
	if (buttonPushed[BPDW]){
		motorStop();
		motorBackward();
		clearButtons();
	}
	if (buttonPushed[BPOK]){
		motorStop();
		clearButtons();
		break;
	}*/
	return 0;
}


uint8_t expertAlarm(){
	gdiTime_t tmpTime;
	lcd.clear();
	enterTime(&tmpTime,0);
	RTC.setAlarm(tmpTime.M,tmpTime.H,99,99);
	delay(1000);
	return 0;
}

