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



void calculerEphemeride2(int jour, int mois, int annee, double longitude_ouest, double latitude_nord, double *lever, double *meridien, double *coucher);
//Entr�es :
//   jour
//   mois
//   annee : valeur 00 � 99 ou bien 2000 � 2099
//   longitude_ouest : nombre d�cimal, n�gatif si longitude est
//   latitude_nord : nombre d�cimal, n�gatif si latitude sud
//Sorties : lever, meridien, coucher sous forme de nombre d�cimal (julien)
//   -0.5 =>  0h00 UTC
//    0.0 => 12h00 UTC
//    0.5 => 24h00 UTC
//
//Les valeurs obtenues peuvent �tre v�rifi�es sur le site de l'Institut de M�canique C�leste et de Calcul des Eph�m�rides (IMCCE)
//http://www.imcce.fr
//=> Eph�m�rides
//=> Ph�nom�nes c�lestes
//=> Levers, couchers et passages au m�ridien des corps du syst�me solaire
//=> Lieu g�ographique � saisir en coordonn�es latitude / longitude
//=> Options : pr�cision = seconde, Format = Sexag�simal


//* fonction interne :


void calculerEphemeride(uint8_t jour, uint8_t mois, uint8_t annee, int16_t longitude_ouest, int8_t latitude_nord, float *lever, float *meridien, float *coucher){
	calculerEphemeride2((int )(jour),( int )(mois),( int )(annee),( double )(longitude_ouest),( double )(latitude_nord), (double *)(lever), (double *)(meridien), (double *)(coucher));
}


  const double M_2PI = 2.0 * M_PI;
  const double degres = 180.0 / M_PI;
  const double radians = M_PI / 180.0;
  const double radians2 = M_PI / 90.0;
  const double m0 = 357.5291;
  const double m1 = 0.98560028;
  const double l0 = 280.4665;
  const double l1 = 0.98564736;
  const double c0 = 0.01671;
  const double c1 = degres * (2.0*c0 - c0*c0*c0/4.0);
  const double c2 = degres * c0*c0 * 5.0 / 4.0;
  const double c3 = degres * c0*c0*c0 * 13.0 / 12.0;
  const double r1 = 0.207447644182976; // = tan(23.43929 / 180.0 * M_PI / 2.0)
  const double r2 = r1*r1;
  const double d0 = 0.397777138139599; // = sin(23.43929 / 180.0 * M_PI)
  const double o0 = -0.0106463073113138; // = sin(-36.6 / 60.0 * M_PI / 180.0)

  double M,C,L,R,dec,omega,x;


void calculerCentreEtVariation(double longitude_ouest, double sinlat, double coslat, double d, double *centre, double *variation)
{
  //constantes pr�calcul�es par le compilateur

  //deux ou trois petites formules de calcul
  M = radians * fmod(m0 + m1 * d, 360.0);
  C = c1*sin(M) + c2*sin(2.0*M) + c3*sin(3.0*M);
  L = fmod(l0 + l1 * d + C, 360.0);
  x = radians2 * L;
  R = -degres * atan((r2*sin(x))/(1+r2*cos(x)));
  *centre = (C + R + longitude_ouest)/360.0;

  dec = asin(d0*sin(radians*L));
  omega = (o0 - sin(dec)*sinlat)/(cos(dec)*coslat);
  if ((omega > -1.0) && (omega < 1.0))
    *variation = acos(omega) / M_2PI;
  else
    *variation = 0.0;
}

void calculerEphemeride2(int jour, int mois, int annee, double longitude_ouest, double latitude_nord, double *lever, double *meridien, double *coucher)
{
  int nbjours;
  const double radians = M_PI / 180.0;
  double d, x, sinlat, coslat;

  //calcul nb jours �coul�s depuis le 01/01/2000
  if (annee > 2000) annee -= 2000;
  nbjours = (annee*365) + ((annee+3)>>2) + jour - 1;
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
  }
  if ((mois > 2)&&((annee&3) == 0)) nbjours++;
  d = nbjours;

  //calcul initial meridien & lever & coucher
  x = radians * latitude_nord;
  sinlat = sin(x);
  coslat = cos(x);
  calculerCentreEtVariation(longitude_ouest, sinlat, coslat, d + longitude_ouest/360.0, meridien, &x);
  *lever = *meridien - x;
  *coucher = *meridien + x;

  //seconde it�ration pour une meilleure pr�cision de calcul du lever
  calculerCentreEtVariation(longitude_ouest, sinlat, coslat, d + *lever, lever, &x);
  *lever = *lever - x;

  //seconde it�ration pour une meilleure pr�cision de calcul du coucher
  calculerCentreEtVariation(longitude_ouest, sinlat, coslat, d + *coucher, coucher, &x);
  *coucher = *coucher + x;
}


#endif /* EPHEMERIDE_H_ */
