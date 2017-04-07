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
#define M_2PI 2.0 * M_PI
#define degres 180.0 / M_PI
#define radians2 M_PI / 180.0
#define radians3 M_PI / 90.0
#define m0 357.5291
#define m1 0.98560028
#define l0 280.4665
#define l1 0.98564736
#define c0 0.01671
#define c1 degres * (2.0*c0 - c0*c0*c0/4.0)
#define c2 degres * c0*c0 * 5.0 / 4.0
#define c3 degres * c0*c0*c0 * 13.0 / 12.0
#define r1 0.207447644182976 // = tan(23.43929 / 180.0 * M_PI / 2.0)
#define r2 r1*r1
#define d0 0.397777138139599 // = sin(23.43929 / 180.0 * M_PI)
#define o0 -0.0106463073113138 // = sin(-36.6 / 60.0 * M_PI / 180.0)



// fonction interne :

void calculerCentreEtVariation(int16_t longitude_ouest, int8_t latitude_nord, float d, float *centre, float *variation)
{/*

  float M,C,L,R,dec,omega,x;

  //deux ou trois petites formules de calcul
  M = radians2 * fmod(m0 + m1 * d, 360.0);
  C = c1*sin(M) + c2*sin(2.0*M) + c3*sin(3.0*M);
  L = fmod(l0 + l1 * d + C, 360.0);
  x = radians3 * L;
  R = -degres * atan((r2*sin(x))/(1+r2*cos(x)));
  *centre = (C + R + longitude_ouest)/360.0;

  dec = asin(d0*sin(radians2*L));
  omega = (o0 - sin(dec)*sinlat)/(cos(dec)*coslat);
  if ((omega > -1.0) && (omega < 1.0))
    *variation = acos(omega) / M_2PI;
  else
    *variation = 0.0;*/
	//commented above has been RAM optimized into:
	  float tmp;

	  //deux ou trois petites formules de calcul
	  tmp = radians2 * fmod(m0 + m1 * d, 360.0);
	  *variation = c1*sin(tmp) + c2*sin(2.0*tmp) + c3*sin(3.0*tmp);
	  tmp = fmod(l0 + l1 * d + *variation, 360.0);
	  *centre = radians3 * tmp;
	  tmp = -degres * atan((r2*sin(*centre))/(1+r2*cos(*centre)));
	  *centre = (*variation + tmp + longitude_ouest)/360.0;

	  tmp = fmod(l0 + l1 * d + *variation, 360.0);
	  *variation = asin(d0*sin(radians2*tmp));
	  tmp = (o0 - sin(*variation)*sin(radians2 *latitude_nord))/(cos(*variation)*cos(radians2 *latitude_nord));
	  if ((tmp > -1.0) && (tmp < 1.0))
	    *variation = acos(tmp) / M_2PI;
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
/*
  switch (mois)
  {
    case  2 : nbjours +=  31; break;
    case  3 : nbjours +=  59; break;
    case  4 : nbjours +=  90; break;
    case  5 : nbjours += 120; break;
    case  6 : nbjours += 151; break;
    case  7 : nbjours += 181; break;
    case  8 : nbjours += 212; break;
    case  9 : nbjours += 243; break;
    case 10 : nbjours += 273; break;
    case 11 : nbjours += 304; break;
    case 12 : nbjours += 334; break;
  }*/
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
