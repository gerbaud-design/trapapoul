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


extern volatile int motorPosition;
extern unsigned long topTimeout;
extern volatile unsigned long lastPush[3];
extern volatile bool buttonState[3];
extern volatile bool buttonPushed[3];

extern uint8_t resetSource;


//config variables
#define SOLEIL 1
#define FIXE 2
#define MINIMUM 3
uint8_t openMode=SOLEIL;
uint8_t closeMode=SOLEIL;

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
gdiTime_t openTime,closeTime;
uint8_t closeDelay=DEFAULT_CLOSE_DELAY;


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


void installationTrappe();
void manualMoveMotor();
void updateOpenTime();
void updateCloseTime();
void userInterface();

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
		if (newState[BPOK]==1 && ((millis()-lastPush[BPOK])>DEBOUNCE)){
			lastPush[BPOK]=millis();
			buttonPushed[BPOK]=1;
			buttonState[BPOK]=1;
		}
		if(newState[BPOK]==0) buttonState[BPOK]=0;
	}
	if (newState[BPDW]!=buttonState[BPDW]){
		if (newState[BPDW]==1 && ((millis()-lastPush[BPDW])>DEBOUNCE)){
			lastPush[BPDW]=millis();
			buttonPushed[BPDW]=1;
			buttonState[BPDW]=1;
		}
		if(newState[BPDW]==0) buttonState[BPDW]=0;
	}
	if (newState[BPUP]!=buttonState[BPUP]){
		if (newState[BPUP]==1 && ((millis()-lastPush[BPUP])>DEBOUNCE)){
			lastPush[BPUP]=millis();
			buttonPushed[BPUP]=1;
			buttonState[BPUP]=1;
		}
		if(newState[BPUP]==0) buttonState[BPUP]=0;
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

//setup du lcd
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0,0);
	lcd.print(F("  SYSTEM RESET  "));
	lcd.setCursor(0,1);
//	lcd.print(F("value:"));
	lcd.print(resetSource,HEX);
	delay(2000);
	lcd.noBacklight();
	lcd.clear();
	uploadChar(CHECK_CHAR,checkChar);
	uploadChar(DEG_CHAR,degChar);
	lcd.setCursor(0,0);

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

//initialisation of analog inputs
	analogReference(EXTERNAL);


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

}


#define DISABLE_ADC ADCSRA &= ~(1 << ADEN)
#define ENABLE_ADC ADCSRA |= (1 << ADEN)
#define DISABLE_ACOMP ACSR &= ~(1 << ACD)

// The loop function is called in an endless loop
void loop()
{

	activateVpp();
	switch(wakeUpSource){
	case wu_unknown:
		lcd.backlight();
		lcd.setCursor(0,0);
		lcd.print(F("UNKNOWN WAKE UP"));
		delay(2000);
		//nobreak
	case wu_button:
		clearButtons();
		lcd.clear();
		lcd.backlight();
		userInterface();
		lcd.clear();
		lcd.noBacklight();
		//nobreak
	case wu_alarm:
		if(configured!=0){
			updateCloseTime();
			updateOpenTime();
			updateDateTime();
			if((((RTC.getHour()>openTime.H)||\
					((RTC.getHour()==openTime.H)&&(RTC.getMinute()>=openTime.M)))\
					&&((RTC.getHour()<closeTime.H)||\
					((RTC.getHour()==closeTime.H)&&(RTC.getMinute()<closeTime.M))))==1){
				if(doorState==ds_earlyOpened){
					doorState=ds_opened;
				}
				if(doorState==ds_closed){
					activateMotor();
					if(RTC.getVoltLow()){
						motorGoTo(positionHaute);//open door
						deactivateMotor();
						doorState=ds_opened;
					}else{
						doorState=ds_forceClosed;
					}
				}
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


void loadPosition(void){
	/*
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
	quadrature = (quadratureA + quadratureA + quadratureB);*/
}


void userInterface()
{
	MENU:
	 topTimeout=millis();
		while(1){


			//time display
			updateDateTime();
			lcd.setCursor(0,0);
			printTimeLCD(nowTime,1);
			if(!resetSource) lcd.print('X');
			//battery voltage measurement
			anaRead=0;
			for(uint8_t i=0;i<64;++i){
				anaRead+=analogRead(pinMesVbat);
				delay(5);
			}
			anaRead /=ratioVbat;

			//battery voltage display

			lcd.setCursor(12,0);
			lcd.print((float(anaRead))/100);
			/*lcd.write(BAT1_CHAR);
			lcd.write(BAT2_CHAR);
			lcd.write(BAT3_CHAR);*/


			//display dooring planning
			lcd.setCursor(0,1);
			switch(doorState){
			case ds_opened:
			case ds_earlyOpened:
				lcd.print(F("FERMERA A       "));
				lcd.setCursor(10,1);
				printTimeLCD(closeTime,0);
				break;
			case ds_closed:
			case ds_earlyClosed:
				lcd.print(F("OUVRIRA A       "));
				lcd.setCursor(10,1);
				printTimeLCD(openTime,0);
				break;
			case ds_unknown:
				lcd.print(F("PARAMETREZ MOI  "));
				break;
			case ds_forceOpened:
				lcd.print(F("OUVERT INFINI   "));
				break;
			case ds_forceClosed:
				lcd.print(F("FERME INFINI    "));
				break;
			}

			if(buttonPushed[BPOK]==1)
				goto MENU_OUVERTURE;
			if((millis()-topTimeout)>BUTTON_TIMEOUT)
					goto MENU_TIMEOUT;
		}

	MENU_OUVERTURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DU MODE "));
		lcd.setCursor(0,1);
		lcd.print(F("D'OUVERTURE     "));
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
		lcd.setCursor(0,0);
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("LEVE DU SOLEIL  "));
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
		lcd.setCursor(0,0);
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE FIXE      "));
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
		lcd.print(F("HEURE OUVERTURE "));
		lcd.setCursor(0,1);
		printTimeLCD(openTime,0);
		enterNumber(&openTime.H,0,23,0,1,2);
		enterNumber(&openTime.M,0,59,3,1,2);
		goto MENU_OUVERTURE_ENREGISTREE;

	MENU_OUVERTURE_MINIMUM:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE MINIMUM   "));
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
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
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
		lcd.print(F("OUVERTURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("ENREGISTREE     "));
		updateOpenTime();
		//delay(2000);
		goto MENU;

	MENU_FERMETURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DU MODE "));
		lcd.setCursor(0,1);
		lcd.print(F("DE FERMETURE    "));
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
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("COUCHE DU SOLEIL"));
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
		lcd.setCursor(0,0);
		lcd.print(F("DELAI FERMETURE "));
		lcd.setCursor(0,1);
		enterNumber(&closeDelay,0,60,0,1,2);
		goto MENU_FERMETURE_ENREGISTREE;

	MENU_FERMETURE_FIXE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE FIXE      "));
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
			lcd.print(F("HEURE FERMETURE "));
			lcd.setCursor(0,1);
			printTimeLCD(closeTime,0);
			enterNumber(&closeTime.H,0,23,0,1,2);
			enterNumber(&closeTime.M,0,59,3,1,2);
			goto MENU_FERMETURE;

	MENU_FERMETURE_MINIMUM:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("HEURE MINIMUM   "));
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
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
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
		lcd.setCursor(0,0);
		lcd.print(F("FERMETURE       "));
		lcd.setCursor(0,1);
		lcd.print(F("ENREGISTREE     "));
		updateCloseTime();
		//delay(2000);
		goto MENU;


	MENU_DATE_HEURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DATE    "));
		lcd.setCursor(0,1);
		lcd.print(F("ET HEURE        "));
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE;
		case BPDW:
			goto MENU_INSTALLATION;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			updateDateTime();

			lcd.setCursor(0,0);
			lcd.print(F("HEURE:          "));
			enterTime(&nowTime);
			RTC.setTime(nowTime.H,nowTime.M,nowTime.S);
			lcd.setCursor(0,0);
			lcd.print(F("HEURE           "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREES    "));
			delay(2000);
			lcd.setCursor(0,0);
			lcd.print(F("DATE:           "));
			enterDate(&nowDate);
			RTC.setDate(nowDate.D,0,nowDate.M,0,nowDate.Y);
			lcd.setCursor(0,0);
			lcd.print(F("DATE            "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREES    "));
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
	lcd.setCursor(0,0);
	lcd.print(F("INSTALLATION    "));
	lcd.setCursor(0,1);
	lcd.print(F("DE LA TRAPPE    "));
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
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE AVANCES "));
		lcd.setCursor(0,1);
		lcdClearLine();
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

MENU_EXPERT_CHARGE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("CHARGE          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_EXPERT_CHARGE;
		case BPDW:
			goto MENU_EXPERT_CYCLAGE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
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

			goto MENU;
		default:
			goto MENU_ERROR;
		}

	MENU_EXPERT_CYCLAGE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("CYCLAGE         "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_EXPERT_CHARGE;
		case BPDW:
			goto MENU_EXPERT_EPHEMERIDES;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:/*
			activateMotor();
			lcd.setCursor(0,0);
			lcd.print(F("CYCLAGE EN COURS"));
			lcd.setCursor(0,1);
			lcd.print(F("ITERATION       "));
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
				pushLog(printTime());
				pushLog(" Iteration ");
				pushLog(String(nbCycles));
				pushLog("\n");
				if(nbCycles==NB_ITERATION)
					break;

				delay(10000);
			}
			if(machineState==TIMEOUT){
				pushLog(printTime());
				pushLog(" Broken ");
				pushLog("\n");
				lcd.setCursor(0,1);
				lcd.print(F("BROKEN AT       "));
			}
			manualMoveMotor();
			deactivateMotor();*/
			goto MENU_EXPERT;
		default:
			goto MENU_ERROR;
		}
	MENU_EXPERT_EPHEMERIDES:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("EPHEMERIDES     "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_EXPERT_CYCLAGE;
		case BPDW:
			goto MENU_EXPERT_DEBUG;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			updateDateTime();
		//	calculerEphemeride(timeElements.Day,timeElements.Month,timeElements.Year,45,-5,&lever,&meridien,&coucher);
			calculerEphemeride(RTC.getDay(),RTC.getMonth(),RTC.getYear(),-5,45,&lever,&meridien,&coucher);
			lcd.setCursor(0,0);
			lcd.print(lever);
			waitButton();
			lcd.setCursor(0,1);
			lcd.print(coucher);
			waitButton();
			lcd.clear();
			julianTranslate(&nowTime.H,&nowTime.M,&nowTime.S,lever);
			lcd.setCursor(0,1);
			printTimeLCD(nowTime,1);
			waitButton();
			goto MENU_EXPERT_EPHEMERIDES;
		default:
			goto MENU_ERROR;
		}

		MENU_EXPERT_DEBUG:
			clearButtons();
			lcd.setCursor(0,0);
			lcd.print(F("DEBUG           "));
			lcd.setCursor(0,1);
			lcdClearLine();
			switch(waitButton()){
			case BPUP:
				goto MENU_EXPERT_EPHEMERIDES;
			case BPDW:
				goto MENU_EXPERT_RETOUR;
			case TIMEOUT:
				goto MENU_TIMEOUT;
			case BPOK:
	/*			activateMotor();
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
					}
				}*/
				goto MENU_EXPERT;
			default:
				goto MENU_ERROR;
			}

	MENU_EXPERT_RETOUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_EXPERT_EPHEMERIDES;
		case BPDW:
			goto MENU_EXPERT_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_EXPERT;
		default:
			goto MENU_ERROR;
		}

	MENU_QUITTER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("QUITTER         "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_EXPERT;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			return;
		default:
			goto MENU_ERROR;
		}

	MENU_ERROR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("ERREUR MENU     "));
		lcd.setCursor(0,1);
		lcdClearLine();
		delay(5000);
		goto MENU;

	MENU_TIMEOUT:
		lcd.noBacklight();
		return;

}


void installationTrappe(){
//réglage hauteur
	//réglage date-heure
	//réglage gps

	MENU_HAUTEUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE HAUTEUR "));
		lcd.setCursor(0,1);
		lcd.print(F("DE LA TRAPPE    "));
		switch(waitButton()){
		case BPUP:
			goto MENU_HAUTEUR;
		case BPDW:
			goto MENU_GPS;
		case TIMEOUT:
			goto MENU_HAUTEUR;
		case BPOK:
			clearButtons();
			activateMotor();

			lcd.setCursor(0,0);
			lcd.print(F("REGLAGE DE LA   "));
			lcd.setCursor(0,1);
			lcd.print(F("POSITION FERMEE "));
			manualMoveMotor();
			resetPosition();
			lcd.setCursor(0,0);
			lcd.print(F("POSITION FERMEE "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			clearButtons();
			lcd.setCursor(0,0);
			lcd.print(F("REGLAGE DE LA   "));
			lcd.setCursor(0,1);
			lcd.print(F("POSITION OUVERTE"));
			manualMoveMotor();
			positionHaute=motorPosition;
			EEPROM.put(EEPROM_POSOPEN,positionHaute);
			deactivateMotor();
			lcd.setCursor(0,0);
			lcd.print(F("POSITION OUVERTE"));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			doorState=ds_opened;
			updateCloseTime();

			goto MENU_GPS;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("MODIFIER LA     "));
		lcd.setCursor(0,1);
		lcd.print(F("POSITION GPS    "));
		switch(waitButton()){
		case BPUP:
			goto MENU_HAUTEUR;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_GPS;
		case BPOK:
			goto MENU_GPS_DPT;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_DPT:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("POSITION GPS    "));
		lcd.setCursor(0,1);
		lcd.print(F("PAR DEPARTEMENT "));
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_DPT;
		case BPDW:
			goto MENU_GPS_GPS;
		case TIMEOUT:
			goto MENU_GPS_DPT;
		case BPOK:
			enterDepartement();
			goto MENU_GPS_ENREGISTRER;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_GPS:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("POSITION GPS    "));
		lcd.setCursor(0,1);
		lcd.print(F("MANUELLE        "));
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_DPT;
		case BPDW:
			goto MENU_GPS_RETOUR;
		case TIMEOUT:
			goto MENU_GPS_GPS;
		case BPOK:
			EEPROM.get(EEPROM_LAT,latitudeNord);
			EEPROM.get(EEPROM_LON,longitudeOuest);
			enterGPS(&latitudeNord,&longitudeOuest);
			EEPROM.put(EEPROM_LAT,latitudeNord);
			EEPROM.put(EEPROM_LON,longitudeOuest);
			goto MENU_GPS_ENREGISTRER;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_RETOUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_GPS;
		case BPDW:
			goto MENU_GPS_RETOUR;
		case TIMEOUT:
			goto MENU_GPS_RETOUR;
		case BPOK:
			goto MENU_GPS;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_ENREGISTRER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("POSITION GPS    "));
		lcd.setCursor(0,1);
		lcd.print(F("ENREGISTREE     "));
		delay(2000);
		goto MENU_QUITTER;


	MENU_QUITTER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcdClearLine();
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_QUITTER;
		case BPOK:
			return;
		default:
			goto MENU_ERROR;
		}

	MENU_ERROR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("ERREUR MENU INST"));
		lcd.setCursor(0,1);
		lcdClearLine();
		delay(5000);
		return;
}

void updateOpenTime(){
	updateDateTime();
	calculerEphemeride(RTC.getDay(),RTC.getMonth(),RTC.getYear(),-5,45,&lever,&meridien,&coucher);
	if (openMode==SOLEIL){
		julianTranslate(&openTime.H,&openTime.M,&nowTime.S,lever);//nowTime used only as trash result to limit variable number
	}
}

void updateCloseTime(){
	updateDateTime();
	calculerEphemeride(RTC.getDay(),RTC.getMonth(),RTC.getYear(),-5,45,&lever,&meridien,&coucher);
	if (closeMode==SOLEIL){
		julianTranslate(&closeTime.H,&closeTime.M,&nowTime.S,coucher);//nowTime used only to limit variable number
		closeTime.M+=closeDelay;
		while(closeTime.M>59){
			closeTime.H+=1;
			closeTime.M-=60;
		}
	}
}



