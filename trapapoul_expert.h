/*
 * trapapoul_expert.h
 *
 *  Created on: 1 mai 2017
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_EXPERT_H_
#define TRAPAPOUL_EXPERT_H_

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
/*
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
	return 0;
}
extern float lever,coucher,meridien;
uint8_t expertEphemerides(){
	updateDateTime();
//	calculerEphemeride(timeElements.Day,timeElements.Month,timeElements.Year,45,-5,&lever,&meridien,&coucher);
	calculerEphemeride(RTC.getDay(),RTC.getMonth(),RTC.getYear(),-5,45,&lever,&meridien,&coucher);
	lcd.setCursor(0,0);
	//lcd.print(lever);
	waitButton();
	lcd.setCursor(0,1);
	//lcd.print(coucher);
	waitButton();
	lcd.clear();
	julianTranslate(&nowTime.H,&nowTime.M,&nowTime.S,lever);
	lcd.setCursor(0,1);
	lcdPrintTime(&nowTime,1);
	waitButton();
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



#endif /* TRAPAPOUL_EXPERT_H_ */
