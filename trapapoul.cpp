/*
 * protopoulie.cpp
 *
 *  Created on: 22 avr. 2016
 *      Author: guiguilours
 */

#include "arduino.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DS1337RTC.h>
#include <Time.h>
#include <TimerOne.h>



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


//boutons config
#define BPUP 1
#define BPDW 0
#define BPOK 3


//menu variables
volatile uint16_t menuPointer=0;
void userInterface();
uint8_t waitButton();
#define TIMEOUT 50
void numberInput(uint8_t min,uint8_t max);
void enterTime(tmElements_t*);


//config variables
#define SOLEIL 1
#define FIXE 2
#define MINIMUM 3
uint8_t openMode=SOLEIL;
uint8_t closeMode=SOLEIL;
uint8_t cur_mm=30;
uint8_t cur_hh=12;

double latitudeNord;
double longitudeOuest;
void enterGPS (double*, double*);


//analogconfig
uint16_t mes1=0,mes2=0,mes0=0;
uint32_t tot1=0,tot2=0,tot0=0;
uint8_t toti=0;



// interrupt routine for pin2 alarm.
void interrupt_0 () {
	if (digitalRead(BPOK) == 0)
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
	Serial.print("\n\n");
	Serial.println("Beggining of protortc");

//setup des boutons
	pinMode(BPUP, INPUT);
	pinMode(BPDW, INPUT);
	pinMode(BPOK, INPUT);
	digitalWrite(BPUP,1);
	digitalWrite(BPDW,1);
	digitalWrite(BPOK,1);


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
	Serial.println("Initializing SD card...");
	// we'll use the initialization code from the utility libraries
	// see if the card is present and can be initialized:
	pinMode(SDchipSelect,OUTPUT);
	if (!SD.begin(SDchipSelect)) {
		Serial.println("SDcard failed, or not present");
		// don't do anything more:
		while(1);
	}
	Serial.println("SDcard initialized.");
	File LogFile = SD.open(LogFileName, FILE_WRITE);
	// if the file is available, write to it:
	if (LogFile) {
		LogFile.close();
		Serial.println("log file opened");
	}
	else
		Serial.println("logfile opening error");
	pushLog("\n\n\n\n\n\nNew log session ");
	pushLog(printDate());
	pushLog(" ");
	pushLog(printTime());
	pushLog("Let the log beggin!\n");

	//analog init
	analogReference(EXTERNAL);
*/
	Serial.println("end of setup");
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
			//Serial.println("alarme minute déclenchée");
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
/*			Serial.print("moyennes ;");
			Serial.print(res1);
			Serial.print(" mA, ");
			Serial.print(res2);
			Serial.print("mA , ");
			Serial.print(res3);
			Serial.println("mA");*/
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

		Serial.print(";");
		Serial.print(mes0);
		Serial.print(";");
		Serial.print(mes1);
		Serial.println("");
		delay(1000);

	*/
	/*
		if(!digitalRead(BPUP)){
			lcd.setCursor(14,0);
			lcd.print("UP");
		}else{
			lcd.setCursor(14,0);
			lcd.print("  ");
		}
		if(!digitalRead(BPOK)){
			lcd.setCursor(7,0);
			lcd.print("OK");
		}else{
			lcd.setCursor(7,0);
			lcd.print("  ");
		}
		if(!digitalRead(BPDW)){
			lcd.setCursor(0,0);
			lcd.print("DW");
		}else{
			lcd.setCursor(0,0);
			lcd.print("  ");
		}
		if(timeToGo==1){

			lcd.setCursor(6,1);
			lcd.print("BOOM");
			delay(1000);
			lcd.setCursor(6,1);
			lcd.print("    ");
			timeToGo=0;
		}
	*/

		userInterface();
		lcd.noBacklight();
		lcd.setCursor(0,0);
		lcd.print("                ");
		lcd.setCursor(0,1);
		lcd.print("                ");
		delay(1000);
		lcd.backlight();
		delay(1000);
	}
}

uint8_t waitButton()
{
	uint32_t timeout=millis()+10000;
	delay(500);//debouncing
	while(1){//will have to handle timeout
		if(!digitalRead(BPUP))
			return BPUP;
		if(!digitalRead(BPOK))
			return BPOK;
		if(!digitalRead(BPDW))
			return BPDW;
		if(millis()>timeout)
			return TIMEOUT;
	}

}

void numberInput(uint8_t min,uint8_t max){



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

	//init du timer1 et de son interrupt de clignotage
	Timer1.initialize(500000);
	Timer1.attachInterrupt(interrupt_blinker);
	Timer1.start();
ENTER_DATE:
	lcd.setCursor(0,0);
	lcd.print("DATE:           ");
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
	lcd.print("      ");
	while(1){
		delay(500);
		noInterrupts();
		bool blinkCopy=blink;
		interrupts();
		if (blinkCopy==1){
			lcd.setCursor(0,1);
			if (te->Day<10)
				lcd.print('0');
			lcd.print(te->Day);
		}else{
			lcd.setCursor(0,1);
			lcd.print("  ");
		}
		if((!digitalRead(BPUP))&&(te->Day<31)){
			te->Day+=1;
			lcd.setCursor(0,1);
			if (te->Day<10)
				lcd.print('0');
			lcd.print(te->Day);
		}
		if((!digitalRead(BPDW))&&(te->Day>0)){
			te->Day-=1;
			lcd.setCursor(0,1);
			if (te->Day<10)
				lcd.print('0');
			lcd.print(te->Day);
		}
		if(!digitalRead(BPOK)){
			lcd.setCursor(0,1);
			if (te->Day<10)
				lcd.print('0');
			lcd.print(te->Day);
			break;
		}
	}
	while(1){
		delay(500);
		noInterrupts();
		bool blinkCopy=blink;
		interrupts();
		if (blinkCopy==1){
			lcd.setCursor(3,1);
			if (te->Month<10)
				lcd.print('0');
			lcd.print(te->Month);
		}else{
			lcd.setCursor(3,1);
			lcd.print("  ");
		}
		if((!digitalRead(BPUP))&&(te->Month<12)){
			te->Month+=1;
			lcd.setCursor(3,1);
			if (te->Month<10)
				lcd.print('0');
			lcd.print(te->Month);
		}
		if((!digitalRead(BPDW))&&(te->Month>0)){
			te->Month-=1;
			lcd.setCursor(3,1);
			if (te->Month<10)
				lcd.print('0');
			lcd.print(te->Month);
		}
		if(!digitalRead(BPOK)){
			lcd.setCursor(3,1);
			if (te->Month<10)
				lcd.print('0');
			lcd.print(te->Month);
			break;
		}
	}
	while(1){
		delay(500);
		noInterrupts();
		bool blinkCopy=blink;
		interrupts();
		if (blinkCopy==1){
			lcd.setCursor(6,1);
			lcd.print(uint16_t(te->Year)+1970);
		}else{
			lcd.setCursor(6,1);
			lcd.print("    ");
		}
		if((!digitalRead(BPUP))&&(te->Year<254)){
			te->Year+=1;
			lcd.setCursor(6,1);
			lcd.print(uint16_t(te->Year)+1970);
		}
		if((!digitalRead(BPDW))&&(te->Year>0)){
			te->Year-=1;
			lcd.setCursor(6,1);
			lcd.print(uint16_t(te->Year)+1970);
		}
		if(!digitalRead(BPOK)){
			lcd.setCursor(0,1);
			lcd.print(uint16_t(te->Year)+1970);
			break;
		}
	}

	if(!isDateValid(te)){	//check date validity

		lcd.setCursor(0,0);
		lcd.print("DATE NON VALIDE ");
		delay(1000);
		lcd.setCursor(0,0);
		lcd.print("                ");
		delay(500);
		lcd.setCursor(0,0);
		lcd.print("DATE NON VALIDE ");
		delay(1000);
		goto ENTER_DATE;
	}

ENTER_TIME:
	lcd.setCursor(0,0);
	lcd.print("HEURE:          ");
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
	lcd.print("        ");
	while(1){
		delay(500);
		noInterrupts();
		bool blinkCopy=blink;
		interrupts();
		if (blinkCopy==1){
			lcd.setCursor(0,1);
			if (te->Hour<10)
				lcd.print('0');
			lcd.print(te->Hour);
		}else{
			lcd.setCursor(0,1);
			lcd.print("  ");
		}
		if((!digitalRead(BPUP))&&te->Hour<23){
			te->Hour+=1;
			lcd.setCursor(0,1);
			if (te->Hour<10)
				lcd.print('0');
			lcd.print(te->Hour);
		}
		if((!digitalRead(BPDW))&&te->Hour>0){
			te->Hour-=1;
			lcd.setCursor(0,1);
			if (te->Hour<10)
				lcd.print('0');
			lcd.print(te->Hour);
		}
		if(!digitalRead(BPOK)){
			lcd.setCursor(0,1);
			if (te->Hour<10)
				lcd.print('0');
			lcd.print(te->Hour);
			break;
		}
	}
	while(1){
		delay(500);
		noInterrupts();
		bool blinkCopy=blink;
		interrupts();
		if (blinkCopy==1){
			lcd.setCursor(3,1);
			if (te->Minute<10)
				lcd.print('0');
			lcd.print(te->Minute);
		}else{
			lcd.setCursor(3,1);
			lcd.print("  ");
		}
		if((!digitalRead(BPUP))&&te->Minute<59){
			te->Minute+=1;
			lcd.setCursor(3,1);
			if (te->Minute<10)
				lcd.print('0');
			lcd.print(te->Minute);
		}
		if((!digitalRead(BPDW))&&te->Minute>0){
			te->Minute-=1;
			lcd.setCursor(3,1);
			if (te->Minute<10)
				lcd.print('0');
			lcd.print(te->Minute);
		}
		if(!digitalRead(BPOK)){
			lcd.setCursor(3,1);
			if (te->Minute<10)
				lcd.print('0');
			lcd.print(te->Minute);
			break;
		}
	}
	while(1){
		delay(500);
		noInterrupts();
		bool blinkCopy=blink;
		interrupts();
		if (blinkCopy==1){
			lcd.setCursor(6,1);
			if (te->Second<10)
				lcd.print('0');
			lcd.print(te->Second);
		}else{
			lcd.setCursor(6,1);
			lcd.print("  ");
		}
		if((!digitalRead(BPUP))&&te->Second<59){
			te->Second+=1;
			lcd.setCursor(6,1);
			if (te->Second<10)
				lcd.print('0');
			lcd.print(te->Second);
		}
		if((!digitalRead(BPDW))&&te->Second>0){
			te->Second-=1;
			lcd.setCursor(6,1);
			if (te->Second<10)
				lcd.print('0');
			lcd.print(te->Second);
		}
		if(!digitalRead(BPOK)){
			lcd.setCursor(6,1);
			if (te->Second<10)
				lcd.print('0');
			lcd.print(te->Second);
			break;
		}
	}

	if(!isDateValid(te)){	//check date validity

		lcd.setCursor(0,0);
		lcd.print("HEURE NON VALIDE");
		delay(1000);
		lcd.setCursor(0,0);
		lcd.print("                ");
		delay(500);
		lcd.setCursor(0,0);
		lcd.print("HEURE NON VALIDE");
		delay(1000);
		goto ENTER_TIME;
	}

WRAP_UP:
	Timer1.stop();
	Timer1.detachInterrupt();

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
			lcd.print(" BAT:XX%");
	/*		lcd.setCursor(9,0);
			lcd.write(BAT1_CHAR);
			lcd.write(BAT2_CHAR);
			lcd.write(BAT3_CHAR);*/
			lcd.setCursor(0,1);
			lcd.print("OUVRIRA A HH:MM ");
		//	lcd.print("FERMERA A HH:MM ");

			if(!digitalRead(BPOK))
				goto MENU_OUVERTURE;
		}

	MENU_OUVERTURE:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE DU MODE ");
		lcd.setCursor(0,1);
		lcd.print("D'OUVERTURE     ");
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
		lcd.setCursor(0,0);
		lcd.print("OUVERTURE       ");
		lcd.setCursor(0,1);
		lcd.print("LEVE DU SOLEIL  ");
		if (openMode==SOLEIL){
			lcd.setCursor(13,0);
			lcd.write(CHECK_CHAR);
			lcd.print(' ');
			lcd.write(CHECK2_CHAR);

		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_SOLEIL;
		case BPDW:
			goto MENU_OUVERTURE_FIXE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			openMode=SOLEIL;
			goto MENU_OUVERTURE_SOLEIL;
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_FIXE:
		lcd.setCursor(0,0);
		lcd.print("OUVERTURE       ");
		lcd.setCursor(0,1);
		lcd.print("HEURE FIXE      ");
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
			openMode=FIXE;
			goto MENU_OUVERTURE_FIXE;
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_MINIMUM:
		lcd.setCursor(0,0);
		lcd.print("OUVERTURE       ");
		lcd.setCursor(0,1);
		lcd.print("HEURE MINIMUM   ");
		if (openMode==MINIMUM){
			lcd.setCursor(15,0);
			lcd.write(CHECK2_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE_FIXE;
		case BPDW:
			goto MENU_OUVERTURE_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			openMode=MINIMUM;
			goto MENU_OUVERTURE_MINIMUM;
		default:
			goto MENU_ERROR;
		}


	MENU_OUVERTURE_RETOUR:
		lcd.setCursor(0,0);
		lcd.print("RETOUR          ");
		lcd.setCursor(0,1);
		lcd.print("                ");
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


	MENU_FERMETURE:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE DU MODE ");
		lcd.setCursor(0,1);
		lcd.print("DE FERMETURE    ");
		switch(waitButton()){
		case BPUP:
			goto MENU_OUVERTURE;
		case BPDW:
			goto MENU_HEURE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_FERMETURE_SOLEIL;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_SOLEIL:
		lcd.setCursor(0,0);
		lcd.print("FERMETURE       ");
		lcd.setCursor(0,1);
		lcd.print("COUCHE DU SOLEIL");
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
			closeMode=SOLEIL;
			goto MENU_FERMETURE_SOLEIL;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_FIXE:
		lcd.setCursor(0,0);
		lcd.print("FERMETURE       ");
		lcd.setCursor(0,1);
		lcd.print("HEURE FIXE      ");
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
			closeMode=FIXE;
			goto MENU_FERMETURE_FIXE;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_MINIMUM:
		lcd.setCursor(0,0);
		lcd.print("FERMETURE       ");
		lcd.setCursor(0,1);
		lcd.print("HEURE MINIMUM   ");
		if (closeMode==MINIMUM){
			lcd.setCursor(15,0);
			lcd.write(CHECK_CHAR);
		}
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE_FIXE;
		case BPDW:
			goto MENU_FERMETURE_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			closeMode=MINIMUM;
			goto MENU_FERMETURE_MINIMUM;
		default:
			goto MENU_ERROR;
		}

	MENU_FERMETURE_RETOUR:
		lcd.setCursor(0,0);
		lcd.print("RETOUR          ");
		lcd.setCursor(0,1);
		lcd.print("                ");
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

	MENU_HEURE:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE HEURE   ");
		lcd.setCursor(0,1);
		lcd.print("                ");
		switch(waitButton()){
		case BPUP:
			goto MENU_FERMETURE;
		case BPDW:
			goto MENU_DATE;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			RTC.read(timeElements,CLOCK_ADDRESS);
			enterTime(&timeElements);
			RTC.write(timeElements,CLOCK_ADDRESS);
			goto MENU_HEURE;
		default:
			goto MENU_ERROR;
		}

	MENU_DATE:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE DATE    ");
		lcd.setCursor(0,1);
		lcd.print("                ");
		switch(waitButton()){
		case BPUP:
			goto MENU_HEURE;
		case BPDW:
			goto MENU_HAUTEUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_ERROR;
		default:
			goto MENU_ERROR;
		}

	MENU_HAUTEUR:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE HAUTEUR ");
		lcd.setCursor(0,1);
		lcd.print("DE LA TRAPPE    ");
		switch(waitButton()){
		case BPUP:
			goto MENU_DATE;
		case BPDW:
			goto MENU_GPS;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_ERROR;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE DE LA   ");
		lcd.setCursor(0,1);
		lcd.print("POSITION GPS    ");
		switch(waitButton()){
		case BPUP:
			goto MENU_HAUTEUR;
		case BPDW:
			goto MENU_EXPERT;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_GPS_DPT;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_DPT:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE PAR     ");
		lcd.setCursor(0,1);
		lcd.print("DEPARTEMENT     ");
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_DPT;
		case BPDW:
			goto MENU_GPS_GPS;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_ERROR;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_GPS:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE DE LA   ");
		lcd.setCursor(0,1);
		lcd.print("POSITION GPS    ");
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_DPT;
		case BPDW:
			goto MENU_GPS_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			//enterGPS(&latitudeNord,&longitudeOuest);
			//sauver en EEPROM
			lcd.setCursor(0,0);
			lcd.print("POSITION GPS    ");
			lcd.setCursor(0,1);
			lcd.print("ENREGISTREE     ");
			delay(1000);
			goto MENU;
		default:
			goto MENU_ERROR;
		}

	MENU_GPS_RETOUR:
		lcd.setCursor(0,0);
		lcd.print("RETOUR          ");
		lcd.setCursor(0,1);
		lcd.print("                ");
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS_GPS;
		case BPDW:
			goto MENU_GPS_RETOUR;
		case TIMEOUT:
			goto MENU_TIMEOUT;
		case BPOK:
			goto MENU_GPS;
		default:
			goto MENU_ERROR;
		}

	MENU_EXPERT:
		lcd.setCursor(0,0);
		lcd.print("REGLAGE AVANCES ");
		lcd.setCursor(0,1);
		lcd.print("                ");
		switch(waitButton()){
		case BPUP:
			goto MENU_GPS;
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
		lcd.setCursor(0,0);
		lcd.print("QUITTER         ");
		lcd.setCursor(0,1);
		lcd.print("                ");
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
		lcd.setCursor(0,0);
		lcd.print("ERREUR MENU     ");
		lcd.setCursor(0,1);
		lcd.print("                ");
		delay(5000);
		goto MENU;

	MENU_TIMEOUT:
		return;

}





/*
void serialSetRtc (){
	uint16_t year=0;
	uint8_t month=0,day=0,hour=0,minute=0,second=0;
	Serial.println("set the time and date");
	Serial.println("enter year");
	while(Serial.available()==0);
	if(Serial.available()>0)
		year=Serial.parseInt();
	Serial.println("enter month");
	while(Serial.available()==0);
	if(Serial.available()>0)
		month=Serial.parseInt();
	Serial.println("enter day");
	while(Serial.available()==0);
	if(Serial.available()>0)
		day=Serial.parseInt();
	Serial.println("enter hour");
	while(Serial.available()==0);
	if(Serial.available()>0)
		hour=Serial.parseInt();
	Serial.println("enter minute");
	while(Serial.available()==0);
	if(Serial.available()>0)
		minute=Serial.parseInt();
	Serial.println("enter second");
	while(Serial.available()==0);
	if(Serial.available()>0)
		second=Serial.parseInt();
	calendar = DateTime(year,month,day,hour,minute,second);
	Serial.println("waiting for the top (hit key) to update time");
	while(Serial.available()==0);
	if(Serial.available()>0)
		RTC.adjust(calendar);
}
*/
void serialSetAlarm(){
	/*char ok=0;
	Serial.println("set the alarm to every minute? (y/n)");
	while(Serial.available()==0);
	if(Serial.available()>0)
		if(ok!='y')
			return;*/
//	RTC.setAlarm(B00000001);
//	RTC.configure(B10010000);
}
/*
//create string with date (jj/mm/yyyy)
String printDate (){
	String date="";
	//date+=("date (jj/mm/yyyy) : ");
//	calendar=RTC.now();
	if(calendar.day()<10)
		date+=("0");
	date+=(calendar.day());
	date+=("/");
	if(calendar.month()<10)
		date+=("0");
	date+=(calendar.month());
	date+=("/");
	if(calendar.year()<10)
		date+=("0");
	date+=(calendar.year());
	return date;
}

//create string with time (hh:mm:ss)
String printTime (){
	String time="";

	//time+=("time (hh:mm:ss) : ");
//	calendar=RTC.now();
	if(calendar.hour()<10)
		time+=("0");
	time+=(calendar.hour());
	time+=(":");
	if(calendar.minute()<10)
		time+=("0");
	time+=(calendar.minute());
	time+=(":");
	if(calendar.second()<10)
		time+=("0");
	time+=(calendar.second());
	return time;
}
*/

/*
void enterGPS (double *latN, double *lonO){
	uint8_t val100_000000;
	uint8_t val010_000000;
	uint8_t val001_000000;
	uint8_t val000_100000;
	uint8_t val000_010000;
	uint8_t val000_001000;
	uint8_t val000_000100;
	uint8_t val000_000010;
	uint8_t val000_000001;
	//init du timer1 et de son interrupt de clignotage
	Timer1.initialize(500000);
	Timer1.attachInterrupt(interrupt_blinker);
	Timer1.start();

	lcd.setCursor(0,0);
	lcd.print("LATITUDE NORD   ");
	lcd.setCursor(0,1);
	lcd.print("LATITUDE SUD    ");
	lcd.setCursor(15,0);
	lcd.write(CHECK_CHAR);
	while(1){
			delay(500);
			noInterrupts();
			bool blinkCopy=blink;
			interrupts();
			if (blinkCopy==1){
				lcd.setCursor(0,1);
				if (te->Day<10)
					lcd.print('0');
				lcd.print(te->Day);
			}else{
				lcd.setCursor(0,1);
				lcd.print("  ");
			}
			if((!digitalRead(BPUP))&&(te->Day<31)){
				te->Day+=1;
				lcd.setCursor(0,1);
				if (te->Day<10)
					lcd.print('0');
				lcd.print(te->Day);
			}
			if((!digitalRead(BPDW))&&(te->Day>0)){
				te->Day-=1;
				lcd.setCursor(0,1);
				if (te->Day<10)
					lcd.print('0');
				lcd.print(te->Day);
			}
			if(!digitalRead(BPOK)){
				lcd.setCursor(0,1);
				if (te->Day<10)
					lcd.print('0');
				lcd.print(te->Day);
				break;
			}
		}
	switch(waitButton()){
	}
}
*/

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
		//Serial.println(" has been logged");
	}
	else{
		Serial.print("logerror ");
		Serial.println(pushee);
	}
}

