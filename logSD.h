/*
 * logSD.h
 *
 *  Created on: 22 juil. 2016
 *      Author: guiguilours
 */

#ifndef LOGSD_H_
#define LOGSD_H_

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "ERRcodes.h"
#include "trapapoul_config.h"
#include "trapapoul_UI.h"

gdiTime_t logTime;


bool pushLog(String);

bool logSdInit(){

	bool result=0;
//initialisation de la carte SD
	pinMode(pinSDSS,OUTPUT);
	if (!SD.begin(pinSDSS)) {
		//should print error message here
		// don't do anything more:
		while(1);
	}
	File LogFile = SD.open(logFileName, FILE_WRITE);
	// if the file is available, write to it:
	if (LogFile) {
		LogFile.close();
		result=1;
	}else{
		result=0;
	}

	pushLog("\n");
	pushLog("\n");
	pushLog(INF_LOGNEW);
	pushLog(printDate());
	pushLog(";");
	updateTime();
	logTime.H=timeElements.Hour;
	logTime.M=timeElements.Minute;
	pushLog(printTime(logTime));
	pushLog("\n");
	return result;
}

bool pushLog (String pushee)
{
	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.
	File LogFile = SD.open(logFileName, FILE_WRITE);
	// if the file is available, write to it:
	if (LogFile) {
		LogFile.print(pushee);
		LogFile.close();
		return 1;
	}
	else{
		return 0;
	}
}


#endif /* LOGSD_H_ */
