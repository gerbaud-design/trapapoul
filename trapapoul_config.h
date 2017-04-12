/*
 * trapapoul_config.h
 *
 *  Created on: 22 juil. 2016
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_CONFIG_H_
#define TRAPAPOUL_CONFIG_H_


//digital pins
#define pinQuadratureA 17 //PC3
#define pinQuadratureB 16 //PC2
#define pinMotorForward 8
#define pinMotorBackward 9
#define pinAlarm 2
#define pinChargeOff 6
#define pinVppEn 7
#define pinBPUP 4
#define pinBPDW 5
#define pinBPOK 3
#define pinSDSS 10
//analog pins
#define pinMesImot 0
#define pinMesVbat 7
#define pinMesVsol 6
#define vrefVoltage 455 //centiVolt
#define VbatMesDivider 2
#define ratioVbat 72 // anaread*64/ratiovbat=centivolt =1024/vrefVoltage*64/VbatMesDivider //(centivolt)
#define ratioImot 249 //anaread*vref/ratioImot=Imot(mA)
//R=0.26Ohm gain=55 70=1000/0.26/55



//button timing const
#define DEBOUNCE 400 //ms
#define BUTTON_TIMEOUT 10000 //ms
#define BLINK_HALF_PERIOD 500
#define DEFAULT_CLOSE_DELAY 30 //min

//EEPROM adresses
#define EEPROM_LON 10 //int
#define EEPROM_LAT 12 //int
#define EEPROM_POSOPEN 14 //int
#define EEPROM_POSITION 500 //int[50]
#define EEPROM_COUNT 600 //int[50]

//motor
#define POST_RUN_TIME 1000
#define MOTOR_TIME_OUT 10000
#define NB_ITERATION 1100

//log config

//const String logFileName = "moulog.txt";

//LCD
#define LCD_ADDRESS 0x27//38//20
#define CHECK_CHAR 0
#define BAT1_CHAR 3
#define BAT2_CHAR 4
#define BAT3_CHAR 5
#define DEG_CHAR 6
const PROGMEM uint8_t checkChar[7] =	{0x00,0x01,0x03,0x16,0x1c,0x08,0x00};
const PROGMEM uint8_t check2Char[7] = 	{0x00,0x00,0x01,0x02,0x14,0x08,0x00};
const PROGMEM uint8_t bat1Char[7] = 	{0x00,0x00,0x00,0x00,0x0e,0x1f,0x1f};
const PROGMEM uint8_t bat2Char[7] = 	{0x00,0x00,0x0e,0x1f,0x1f,0x1f,0x1f};
const PROGMEM uint8_t bat3Char[7] = 	{0x0e,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f};
const PROGMEM uint8_t degChar[7] =		{0x0c,0x12,0x12,0x0c,0x00,0x00,0x00};




#endif /* TRAPAPOUL_CONFIG_H_ */
