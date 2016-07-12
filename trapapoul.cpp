/*
 * protopoulie.cpp
 *
 *  Created on: 22 avr. 2016
 *      Author: guiguilours
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DS1337RTC.h>
#include <Time.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include "departementGPS.h"
#include <AVR/pgmspace.h>


//SDconfig : set up variables using the SD utility library functions:
const uint8_t SDchipSelect = 10;
const String LogFileName = "moulog.txt";
//String LogText;//log format : type;subtype;timestamp;value;comment


//RTCconfig
volatile bool timeToGo=0;
#define alarmPin 2
tmElements_t timeElements;
bool isTimeValid(tmElements_t*);
bool isDateValid(tmElements_t*);

//timer1config
volatile bool blink=0;
volatile uint8_t blinkCount=0;

//lcd_I2C config
LiquidCrystal_I2C lcd(0x27,16,2);
#define CHECK_CHAR 0
#define CHECK2_CHAR 1
#define CHECK3_CHAR 2
#define BAT1_CHAR 3
#define BAT2_CHAR 4
#define BAT3_CHAR 5
#define DEG_CHAR 6

//boutons config
#define pinBPUP 1
#define pinBPDW 0
#define pinBPOK 3
#define BPUP 0
#define BPDW 1
#define BPOK 2
#define DEBOUNCE 400 //ms
#define BUTTON_TIMEOUT 10000 //ms
volatile unsigned long lastPush[3];
volatile bool buttonState[3];
volatile bool buttonPushed[3];
void clearButtons();
uint8_t waitButton();

//menu variables
volatile uint16_t menuPointer=0;
void userInterface();
#define TIMEOUT 50
void enterNumber(uint8_t *val,uint8_t min,uint8_t max,
		uint8_t col, uint8_t lin, uint8_t digit/*, bool print0*/);
void enterTime(tmElements_t*);
#define BLINK_HALF_PERIOD 500


//config variables
#define SOLEIL 1
#define FIXE 2
#define MINIMUM 3
uint8_t openMode=SOLEIL;
uint8_t closeMode=SOLEIL;
uint8_t cur_mm=30;
uint8_t cur_hh=12;

int latitudeNord=45;
int longitudeOuest=-6;
void enterGPS (int*, int*);
void enterDepartement ();
void installationTrappe();

//adressage EEPROM
#define EEPROM_LON 10 //int
#define EEPROM_LAT 12 //int
#define EEPROM_POSL 14 //int
#define EEPROM_POSH 16 //int




//analogconfig
uint16_t mes1=0,mes2=0,mes0=0;
uint32_t tot1=0,tot2=0,tot0=0;
uint8_t toti=0;



// interrupt routine for pin2 alarm.
void interrupt_0 () {
	if (digitalRead(pinBPOK) == 0)
		timeToGo=1;
}

void interrupt_blinker(void)
{
	if (blink==0) {
		blink=1;
		blinkCount++;  // increase when LED turns on
	} else {
		blink=0;
	}
}

//interrupt routine for pin change of port D
ISR (PCINT2_vect){ // handle pin change interrupt for D0 to D7 here
	bool newState[3];
	newState[BPOK]=(!(digitalRead(pinBPOK)));
	newState[BPDW]=(!(digitalRead(pinBPDW)));
	newState[BPUP]=(!(digitalRead(pinBPUP)));
	if (newState[BPOK]!=buttonState[BPOK]){
		if (newState[BPOK]==1 && ((millis()-lastPush[BPOK])>DEBOUNCE)){
			lastPush[BPOK]=millis();
			buttonPushed[BPOK]=1;
			buttonState[BPOK]=1;
		}else{
			buttonState[BPOK]=0;
		}
	}
	if (newState[BPDW]!=buttonState[BPDW]){
		if (newState[BPDW]==1 && ((millis()-lastPush[BPDW])>DEBOUNCE)){
			lastPush[BPDW]=millis();
			buttonPushed[BPDW]=1;
			buttonState[BPDW]=1;
		}else{
			buttonState[BPDW]=0;
		}
	}
	if (newState[BPUP]!=buttonState[BPUP]){
		if (newState[BPUP]==1 && ((millis()-lastPush[BPUP])>DEBOUNCE)){
			lastPush[BPUP]=millis();
			buttonPushed[BPUP]=1;
			buttonState[BPUP]=1;
		}else{
			buttonState[BPUP]=0;
		}
	}

}

//interrupt routine for timer1


//void serialSetRtc ();

//void serialSetAlarm();

//create string with date "jj/mm/yyyy"
//String printDate ();

//create string with time "hh:mm:ss"
//String printTime ();


void pushLog(String);

//The setup function is called once at startup of the sketch
void setup()
{


// Open serial communications and wait for port to open:
	Serial.begin(9600);
	Serial.print(F("\n\n"));
	Serial.println(F("Beggining of protortc"));

//setup des boutons
	pinMode(pinBPUP, INPUT);
	pinMode(pinBPDW, INPUT);
	pinMode(pinBPOK, INPUT);
	digitalWrite(pinBPUP,1);
	digitalWrite(pinBPDW,1);
	digitalWrite(pinBPOK,1);
//setup des interuptions boutons
	*digitalPinToPCMSK(pinBPUP) |= bit (digitalPinToPCMSKbit(pinBPUP));  // enable pin
 	*digitalPinToPCMSK(pinBPDW) |= bit (digitalPinToPCMSKbit(pinBPDW));  // enable pin
 	*digitalPinToPCMSK(pinBPOK) |= bit (digitalPinToPCMSKbit(pinBPOK));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pinBPDW)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pinBPDW)); // enable interrupt for the group
    delay(50);
    lastPush[BPOK]=millis();
    lastPush[BPUP]=millis();
    lastPush[BPDW]=millis();
    clearButtons();

//setup du lcd
	lcd.init();
	lcd.backlight();
	lcd.clear();
	{
		uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
		 lcd.createChar(CHECK_CHAR, check);
	}{
		uint8_t check2[8] = {0x0,0x0,0x1,0x2,0x14,0x8,0x0};
		 lcd.createChar(CHECK2_CHAR, check2);
	}{
		uint8_t bat1[8] = {0x0,0x0,0x0,0x0,0xe,0x1f,0x1f};
		 lcd.createChar(BAT1_CHAR, bat1);
	}{
		uint8_t bat2[8] = {0x0,0x0,0xe,0x1f,0x1f,0x1f,0x1f};
		 lcd.createChar(BAT2_CHAR, bat2);
	}{
		uint8_t bat3[8] = {0xe,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f};
		 lcd.createChar(BAT3_CHAR, bat3);
	}{
		uint8_t deg[8] = {0xc,0x12,0x12,0x0c,0x00,0x00,0x00};
		 lcd.createChar(DEG_CHAR, deg);
	}
	lcd.setCursor(0,0);

	//Init of RTC
	//RTC.set(SECS_YR_2000,CLOCK_ADDRESS);
    RTC.disableAlarm(ALARM1_ADDRESS);
    RTC.disableAlarm(ALARM2_ADDRESS);
    RTC.interruptSelect(INTB);
    RTC.resetAlarms();
	pinMode(alarmPin, INPUT);
	attachInterrupt(digitalPinToInterrupt(3), interrupt_0 , FALLING);


/*
	//initialisation de la carte SD
	Serial.println(F("Initializing SD card..."));
	// we'll use the initialization code from the utility libraries
	// see if the card is present and can be initialized:
	pinMode(SDchipSelect,OUTPUT);
	if (!SD.begin(SDchipSelect)) {
		Serial.println(F("SDcard failed, or not present"));
		// don't do anything more:
		while(1);
	}
	Serial.println(F("SDcard initialized."));
	File LogFile = SD.open(LogFileName, FILE_WRITE);
	// if the file is available, write to it:
	if (LogFile) {
		LogFile.close();
		Serial.println(F("log file opened"));
	}
	else
		Serial.println(F("logfile opening error"));
	pushLog("\n\n\n\n\n\nNew log session ");
	pushLog(printDate());
	pushLog(" ");
	pushLog(printTime());
	pushLog("Let the log beggin!\n");

	//analog init
	analogReference(EXTERNAL);
*/
	Serial.println(F("end of setup"));
}

// The loop function is called in an endless loop
void loop()
{


//set alarme
//	serialSetAlarm();

//until the end of time
	while(1){/*
		if(timeToGo==1){
			uint32_t res1=0,res2=0,res0=0;
			//Serial.print(printTime());
			timeToGo=0;
//			RTC.setAlarm(B00000001);
			//Serial.println(F("alarme minute déclenchée"));
			res1=(tot1/toti)/5;
			res2=(tot2/toti)/5;
			res0=(tot0/toti)/5;
			tot1=0;
			tot2=0;
			tot0=0;
			toti=0;*/
/*			pushLog(printTime());
			pushLog(";");
			pushLog(String(res1));
			pushLog(";");
			pushLog(String(res2));
			pushLog(";");
			pushLog(String(res3));
			pushLog("\n");*/
/*			Serial.print(F("moyennes ;"));
			Serial.print(res1);
			Serial.print(F(" mA, "));
			Serial.print(res2);
			Serial.print(F("mA , "));
			Serial.print(res3);
			Serial.println(F("mA"));*/
/*		}
		mes0=analogRead(A0);
		mes1=analogRead(A1);
		mes2=analogRead(A2);
		//mes*2 = mV (res=10Ohm => mes*2/10=mA)
		tot1+=mes1;
		tot2+=mes2;
		tot0+=mes0;
		toti++;
		//Serial.print(printTime());

		Serial.print(F(";"));
		Serial.print(mes0);
		Serial.print(F(";"));
		Serial.print(mes1);
		Serial.println(F(""));
		delay(1000);

	*/
	/*
		if(!digitalRead(pinBPUP)){
			lcd.setCursor(14,0);
			lcd.print(F("UP"));
		}else{
			lcd.setCursor(14,0);
			lcd.print(F("  "));
		}
		if(!digitalRead(pinBPOK)){
			lcd.setCursor(7,0);
			lcd.print(F("OK"));
		}else{
			lcd.setCursor(7,0);
			lcd.print(F("  "));
		}
		if(!digitalRead(pinBPDW)){
			lcd.setCursor(0,0);
			lcd.print(F("DW"));
		}else{
			lcd.setCursor(0,0);
			lcd.print(F("  "));
		}
		if(timeToGo==1){

			lcd.setCursor(6,1);
			lcd.print(F("BOOM"));
			delay(1000);
			lcd.setCursor(6,1);
			lcd.print(F("    "));
			timeToGo=0;
		}
	*/


		clearButtons();
		userInterface();
		lcd.noBacklight();
		lcd.setCursor(0,0);
		lcd.print(F("                "));
		lcd.setCursor(0,1);
		lcd.print(F("                "));
		delay(1000);
		lcd.backlight();
		delay(1000);
	}
}

void clearButtons(void){
    buttonPushed[BPUP]=0;
    buttonPushed[BPOK]=0;
    buttonPushed[BPDW]=0;
}

uint8_t waitButton()
{
	unsigned long topTimeout=millis();
	/*delay(500);//debouncing
	while(1){//will have to handle timeout
		if(!digitalRead(pinBPUP))
			return BPUP;
		if(!digitalRead(pinBPOK))
			return BPOK;
		if(!digitalRead(pinBPDW))
			return BPDW;
		if(millis()>timeout)
			return TIMEOUT;
	}*/
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
	clearButtons();

	//print number in the right place
	lcd.setCursor(col,lin);
	if((*val<100) && (digit==3)/* && (print0==1)*/)
		lcd.print('0');
	if((*val<10) && (digit>=2)/* && (print0==1)*/)
		lcd.print('0');
	lcd.print(*val);
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

#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};
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

void enterTime(tmElements_t *te){

ENTER_DATE :
	lcd.setCursor(0,0);
	lcd.print(F("DATE:           "));
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
		lcd.print(F("                "));
		delay(500);
		lcd.setCursor(0,0);
		lcd.print(F("DATE NON VALIDE "));
		delay(1000);
		goto ENTER_DATE;
	}

ENTER_TIME:
	lcd.setCursor(0,0);
	lcd.print(F("HEURE:          "));
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
	if(!isDateValid(te)){	//check date validity

		lcd.setCursor(0,0);
		lcd.print(F("HEURE NON VALIDE"));
		delay(1000);
		lcd.setCursor(0,0);
		lcd.print(F("                "));
		delay(500);
		lcd.setCursor(0,0);
		lcd.print(F("HEURE NON VALIDE"));
		delay(1000);
		goto ENTER_TIME;
	}
}

void userInterface()
{
	MENU:
		delay(500);
		while(1){
			RTC.read(timeElements,CLOCK_ADDRESS);
			lcd.setCursor(0,0);
			if (timeElements.Hour<10)
				lcd.print('0');
			lcd.print(timeElements.Hour);
			lcd.print('H');
			if (timeElements.Minute<10)
				lcd.print('0');
			lcd.print(timeElements.Minute);
			lcd.print(':');
			lcd.print(timeElements.Second);
			lcd.print(F(" BAT:XX%"));
	/*		lcd.setCursor(9,0);
			lcd.write(BAT1_CHAR);
			lcd.write(BAT2_CHAR);
			lcd.write(BAT3_CHAR);*/
			lcd.setCursor(0,1);
			lcd.print(F("OUVRIRA A HH:MM "));
		//	lcd.print(F("FERMERA A HH:MM "));

			if(buttonState[BPOK]==1)
				goto MENU_OUVERTURE;
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
				goto MENU_OUVERTURE_ENREGISTREE;
			else{
				openMode=FIXE;
				goto MENU_OUVERTURE_FIXE;
			}
		default:
			goto MENU_ERROR;
		}


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
		lcd.print(F("                "));
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
		delay(2000);
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
				goto MENU_FERMETURE_ENREGISTREE;
			else{
				closeMode=SOLEIL;
				goto MENU_FERMETURE_SOLEIL;
			}
		default:
			goto MENU_ERROR;
		}

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
				goto MENU_FERMETURE_ENREGISTREE;
			else{
				closeMode=FIXE;
				goto MENU_FERMETURE_FIXE;
			}
		default:
			goto MENU_ERROR;
		}

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
		lcd.print(F("                "));
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
		delay(2000);
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
			RTC.read(timeElements,CLOCK_ADDRESS);
			enterTime(&timeElements);
			RTC.write(timeElements,CLOCK_ADDRESS);
			lcd.setCursor(0,0);
			lcd.print(F("DATE ET HEURE   "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			goto MENU;
		default:
			goto MENU_ERROR;
		}

	MENU_INSTALLATION:
	clearButtons();
	lcd.setCursor(0,0);
	lcd.print(F("INSTALLATION "));
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
		lcd.print(F("                "));
		switch(waitButton()){
		case BPUP:
			goto MENU_INSTALLATION;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_ERROR;
		default:
			goto MENU_ERROR;
		}

	MENU_QUITTER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("QUITTER         "));
		lcd.setCursor(0,1);
		lcd.print(F("                "));
		switch(waitButton()){
		case BPUP:
			goto MENU_EXPERT;
		case BPDW:
			goto MENU_QUITTER;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU;
		default:
			goto MENU_ERROR;
		}

	MENU_ERROR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("ERREUR MENU     "));
		lcd.setCursor(0,1);
		lcd.print(F("                "));
		delay(5000);
		goto MENU;

	MENU_TIMEOUT:
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
		lcd.print(F("REGLAGE HAUTEUR "));
		lcd.setCursor(0,1);
		lcd.print(F("DE LA TRAPPE    "));
		switch(waitButton()){
		case BPUP:
			goto MENU_HAUTEUR;
		case BPDW:
			goto MENU_DATE_HEURE;
		case TIMEOUT:
			goto MENU_HAUTEUR;
		case BPOK:
			goto MENU_ERROR;
		//	EEPROM.put(EEPROM_POSH,posH);
		//	EEPROM.put(EEPROM_POSL,posL);
			lcd.setCursor(0,0);
			lcd.print(F("POSITION OUVERTE"));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			goto MENU_DATE_HEURE;
		default:
			goto MENU_ERROR;
		}


	MENU_DATE_HEURE:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("REGLAGE DATE    "));
		lcd.setCursor(0,1);
		lcd.print(F("ET HEURE        "));
		switch(waitButton()){
		case BPUP:
			goto MENU_HAUTEUR;
		case BPDW:
			goto MENU_GPS;
		case TIMEOUT:
			goto MENU_DATE_HEURE;
		case BPOK:
			RTC.read(timeElements,CLOCK_ADDRESS);
			enterTime(&timeElements);
			RTC.write(timeElements,CLOCK_ADDRESS);
			lcd.setCursor(0,0);
			lcd.print(F("DATE ET HEURE   "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
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
			goto MENU_DATE_HEURE;
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
			EEPROM.put(EEPROM_LAT,latitudeNord);
			EEPROM.put(EEPROM_LON,longitudeOuest);
			lcd.setCursor(0,0);
			lcd.print(F("POSITION GPS    "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			goto MENU_QUITTER;
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
			lcd.setCursor(0,0);
			lcd.print(F("POSITION GPS    "));
			lcd.setCursor(0,1);
			lcd.print(F("ENREGISTREE     "));
			delay(2000);
			goto MENU_QUITTER;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_RETOUR:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("RETOUR          "));
		lcd.setCursor(0,1);
		lcd.print(F("                "));
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

	MENU_QUITTER:
		clearButtons();
		lcd.setCursor(0,0);
		lcd.print(F("ABANDONNER      "));
		lcd.setCursor(0,1);
		lcd.print(F("L'INSTALLATION  "));
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
		lcd.print(F("ERREUR MENU     "));
		lcd.setCursor(0,1);
		lcd.print(F("                "));
		delay(5000);
		return;

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
	lcd.print(F("                "));
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


void pushLog (String pushee)
{
	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.
	File LogFile = SD.open(LogFileName, FILE_WRITE);
	// if the file is available, write to it:
	if (LogFile) {
		LogFile.print(pushee);
		LogFile.close();

		//Serial.print(pushee);
		//Serial.println(F(" has been logged"));
	}
	else{
		Serial.print(F("logerror "));
		Serial.println(pushee);
	}
}

