/*
 * trapapoul_config.h
 *
 *  Created on: 22 juil. 2016
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_CONFIG_H_
#define TRAPAPOUL_CONFIG_H_


//pins
#define pinQuadratureA 5
#define pinQuadratureB 4
#define pinMotorForward 7
#define pinMotorBackward 8
#define pinAlarm 2
#define pinChargeOFF 6
#define pinBuzzer 9
#define pinBPUP 3
#define pinBPDW 1
#define pinBPOK 0
#define pinSDSS 10


//button timing const
#define DEBOUNCE 400 //ms
#define BUTTON_TIMEOUT 10000 //ms
#define BLINK_HALF_PERIOD 500

//EEPROM adresses
#define EEPROM_LON 10 //int
#define EEPROM_LAT 12 //int
#define EEPROM_POSOPEN 14 //int
#define EEPROM_POSITION 500 //int[50]
#define EEPROM_COUNT 600 //int[50]

//motor
const uint8_t motorOverturn=10;
#define POST_RUN_TIME 1000
#define MOTOR_TIME_OUT 10000
#define NB_ITERATION 1100

//log config

const String logFileName = "moulog.txt";

//LCD special chars#define CHECK_CHAR 0
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
