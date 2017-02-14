/*
 * ephemeride.h
 *
 *  Created on: 8 juil. 2016
 *      Author: guiguilours
 */

#ifndef EPHEMERIDE_H_
#define EPHEMERIDE_H_

//Code source de démonstration de calcul de lever / coucher du soleil
//Peut être compilé pour exécution sur Arduino ou pour exécution sur PC
//2014 Bricoleau

//************************************************************************************************
//* fonctions de calcul d'éphéméride
//************************************************************************************************
//optimisation du code à deux niveaux :
//1) précalculs lors de la compilation
//2) factorisation des appels aux fonctions trigonométriques, couteuses en CPU

#include <math.h>

//* fonction principale :

void calculerEphemeride(int jour, int mois, int annee, float longitude_ouest, float latitude_nord, float *lever, float *meridien, float *coucher);
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


//* fonction interne :

void calculerCentreEtVariation(float longitude_ouest, float sinlat, float coslat, float d, float *centre, float *variation)
{
  //constantes précalculées par le compilateur
  const float M_2PI = 2.0 * M_PI;
  const float degres = 180.0 / M_PI;
  const float radians = M_PI / 180.0;
  const float radians2 = M_PI / 90.0;
  const float m0 = 357.5291;
  const float m1 = 0.98560028;
  const float l0 = 280.4665;
  const float l1 = 0.98564736;
  const float c0 = 0.01671;
  const float c1 = degres * (2.0*c0 - c0*c0*c0/4.0);
  const float c2 = degres * c0*c0 * 5.0 / 4.0;
  const float c3 = degres * c0*c0*c0 * 13.0 / 12.0;
  const float r1 = 0.207447644182976; // = tan(23.43929 / 180.0 * M_PI / 2.0)
  const float r2 = r1*r1;
  const float d0 = 0.397777138139599; // = sin(23.43929 / 180.0 * M_PI)
  const float o0 = -0.0106463073113138; // = sin(-36.6 / 60.0 * M_PI / 180.0)

  float M,C,L,R,dec,omega,x;

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

void calculerEphemeride(int jour, int mois, int annee, float longitude_ouest, float latitude_nord, float *lever, float *meridien, float *coucher)
{
  int nbjours;
  const float radians = M_PI / 180.0;
  float d, x, sinlat, coslat;

  //calcul nb jours écoulés depuis le 01/01/2000
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

  //seconde itération pour une meilleure précision de calcul du lever
  calculerCentreEtVariation(longitude_ouest, sinlat, coslat, d + *lever, lever, &x);
  *lever = *lever - x;

  //seconde itération pour une meilleure précision de calcul du coucher
  calculerCentreEtVariation(longitude_ouest, sinlat, coslat, d + *coucher, coucher, &x);
  *coucher = *coucher + x;
}


float calculerCoordonneeDecimale(int degre, int minute, int seconde)
{
  if (degre > 0)
  {
    return (float) degre + minute / 60.0 + seconde / 3600.0;
  }
  else
  {
    return (float) degre - minute / 60.0 - seconde / 3600.0;
  }
}


/*

int nb_calculs = 0;
unsigned long int duree_calculs = 0;

void testerEphemeride(int jour, int mois, int annee, int nbjours,
                      int longitude_ouest_degres, int longitude_ouest_minutes, int longitude_ouest_secondes,
                      int latitude_nord_degres, int latitude_nord_minutes, int latitude_nord_secondes)
{
  double longitude_ouest, latitude_nord;
  double lever, meridien, coucher;
  int i;
  unsigned long int duree;

  afficherTexte("\nCalcul ephemeride au lieu ");
  afficherCoordonnee(longitude_ouest_degres, longitude_ouest_minutes, longitude_ouest_secondes, 'W');
  afficherTexte(" - ");
  afficherCoordonnee(latitude_nord_degres, latitude_nord_minutes, latitude_nord_secondes, 'N');
  afficherTexte("\n\nDate            Lever         Meridien      Coucher\n");

  longitude_ouest = calculerCoordonneeDecimale(longitude_ouest_degres, longitude_ouest_minutes, longitude_ouest_secondes);
  latitude_nord = calculerCoordonneeDecimale(latitude_nord_degres, latitude_nord_minutes, latitude_nord_secondes);

  for (i=0; i<nbjours; i++)
  {
    afficherDate(jour, mois, annee);
    afficherTexte(" ");

    duree = millis();
    calculerEphemeride(jour, mois, annee, longitude_ouest, latitude_nord, &lever, &meridien, &coucher);
    duree = millis() - duree;
    duree_calculs += duree;
    nb_calculs++;

    afficherHeure(lever);
    afficherTexte(" ");
    afficherHeure(meridien);
    afficherTexte(" ");
    afficherHeure(coucher);
    afficherTexte("\n");

    avancerDate(&jour, &mois, &annee);
  }
}

void setup()
{
  char texte[80];

  initialiserAffichage();

  //Ephéméride à la tour Eiffel en novembre 2014
  testerEphemeride(1, 11, 2014, 30, -2, 17, 40, 48, 51, 28);

  //Ephéméride à l'opéra de Sidney, sur 10 jours à compter du 1er décembre 2014
  testerEphemeride(1, 12, 2014, 10, -151, 12, 54, -33, 51, 25);

  //Ephéméride au pont Golden Gate de San Francisco, sur les 2 premières semaines de 2015
  testerEphemeride(1, 1, 2015, 14, 122, 28, 39, 37, 48, 50);

  //durée moyenne de calcul
  duree_calculs = duree_calculs * 1000 / nb_calculs;
  sprintf(texte,"\nDuree moyenne de calcul d'une ephemeride = %ld microsecondes\n", duree_calculs);
  afficherTexte(texte);
}


*/

#endif /* EPHEMERIDE_H_ */
