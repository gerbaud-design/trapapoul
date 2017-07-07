/*
 * ephemeride.h
 *
 *  Created on: 8 juil. 2016
 *      Author: guiguilours
 */

#ifndef EPHEMERIDE_H_
#define EPHEMERIDE_H_


#include "Arduino.h"
#include "GDI_time.h"
#include <math.h>

// fonction principale :

void calculerEphemeride(uint8_t jour, uint8_t mois, uint8_t annee, int16_t longitude_ouest, int8_t latitude_nord, float *lever, float *meridien, float *coucher);
//Entrées :
//   jour
//   mois
//   annee : valeur 00 à 99 ou bien 2000 à 2099
//   longitude_ouest : nombre décimal, négatif si longitude est
//   latitude_nord : nombre décimal, négatif si latitude sud
//Sorties : lever, meridien, coucher sous forme de nombre décimal (julien)
//   -0.5 =>  0h00 UTC
//    0.0 => 12h00 UTC
//    0.5 => 24h00 UTC
//
//Les valeurs obtenues peuvent être vérifiées sur le site de l'Institut de Mécanique Céleste et de Calcul des Ephémérides (IMCCE)
//http://www.imcce.fr
//=> Ephémérides
//=> Phénomènes célestes
//=> Levers, couchers et passages au méridien des corps du système solaire
//=> Lieu géographique à saisir en coordonnées latitude / longitude
//=> Options : précision = seconde, Format = Sexagésimal



  //constantes précalculées par le compilateur
  PROGMEM const float M_2PI = 2.0 * M_PI;
  PROGMEM const float degres = 180.0 / M_PI;
  PROGMEM const float radians = M_PI / 180.0;
  PROGMEM const float radians2 = M_PI / 90.0;
  PROGMEM const float m0 = 357.5291;
  PROGMEM const float m1 = 0.98560028;
  PROGMEM const float l0 = 280.4665;
  PROGMEM const float l1 = 0.98564736;
  PROGMEM const float c0 = 0.01671;
  PROGMEM const float c1 = degres * (2.0*c0 - c0*c0*c0/4.0);
  PROGMEM const float c2 = degres * c0*c0 * 5.0 / 4.0;
  PROGMEM const float c3 = degres * c0*c0*c0 * 13.0 / 12.0;
  PROGMEM const float r1 = 0.207447644182976; // = tan(23.43929 / 180.0 * M_PI / 2.0)
  PROGMEM const float r2 = r1*r1;
  PROGMEM const float d0 = 0.397777138139599; // = sin(23.43929 / 180.0 * M_PI)
  PROGMEM const float o0 = -0.0106463073113138; // = sin(-36.6 / 60.0 * M_PI / 180.0)



void calculerCentreEtVariation(int16_t longitude_ouest, int8_t latitude_nord, float d, float *centre, float *variation)
{

  float tmp;

  //deux ou trois petites formules de calcul
  tmp = pgm_read_float(&radians) * fmod(pgm_read_float(&m0) + pgm_read_float(&m1) * d, 360.0);
  *variation = pgm_read_float(&c1)*sin(tmp) + pgm_read_float(&c2)*sin(2.0*tmp) + pgm_read_float(&c3)*sin(3.0*tmp);
  tmp = fmod(pgm_read_float(&l0) + pgm_read_float(&l1) * d + *variation, 360.0);
  *centre = pgm_read_float(&radians2) * tmp;
  tmp = -(pgm_read_float(&degres)) * atan((pgm_read_float(&r2)*sin(*centre))/(1+pgm_read_float(&r2)*cos(*centre)));
  *centre = (*variation + tmp + longitude_ouest)/360.0;

  tmp = fmod(pgm_read_float(&l0) + pgm_read_float(&l1) * d + *variation, 360.0);
  *variation = asin(pgm_read_float(&d0)*sin(pgm_read_float(&radians)*tmp));
  tmp = (pgm_read_float(&o0) - sin(*variation)*sin(latitude_nord))/(cos(*variation)*cos(latitude_nord));
  if ((tmp > -1.0) && (tmp < 1.0))
    *variation = acos(tmp) / pgm_read_float(&M_2PI);
  else
    *variation = 0.0;
}


void calculerEphemeride(uint8_t jour, uint8_t mois, uint8_t annee, int16_t longitude_ouest, int8_t latitude_nord, float *lever, float *meridien, float *coucher)
{
	uint8_t i;
  uint16_t nbjours;
  float x;

  //calcul nb jours écoulés depuis le 01/01/2000
  nbjours = (annee*365) + ((annee+3)>>2) + jour - 1;
  for (i=1;i<mois;++i){
	  nbjours+=daysInMonths[mois-1];
  }

  if ((mois > 2)&&((annee&3) == 0)) nbjours++;

  //calcul initial meridien & lever & coucher
  x = radians2 * latitude_nord;
  calculerCentreEtVariation(longitude_ouest, latitude_nord, nbjours + longitude_ouest/360.0, meridien, &x);
  *lever = *meridien - x;
  *coucher = *meridien + x;

  //seconde itération pour une meilleure précision de calcul du lever
  calculerCentreEtVariation(longitude_ouest, latitude_nord, nbjours + *lever, lever, &x);
  *lever = *lever - x;

  //seconde itération pour une meilleure précision de calcul du coucher
  calculerCentreEtVariation(longitude_ouest, latitude_nord, nbjours + *coucher, coucher, &x);
  *coucher = *coucher + x;
}


#endif /* EPHEMERIDE_H_ */
