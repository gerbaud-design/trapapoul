/*
 * trapapoul_text.h
 *
 *  Created on: 19 avr. 2017
 *      Author: guiguilours
 */

#ifndef TRAPAPOUL_TEXT_H_
#define TRAPAPOUL_TEXT_H_

#include "Arduino.h"



const uint8_t BLANK           [16] PROGMEM = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
const uint8_t   SYSTEMRESET   [16] PROGMEM = {' ',' ','S','Y','S','T','E','M',' ','R','E','S','E','T',' ',' '};
const uint8_t UNKNOWN_WAKE_UP [16] PROGMEM = {'U','N','K','N','O','W','N',' ','W','A','K','E',' ','U','P',' '};
const uint8_t REGLAGE_DU_MODE [16] PROGMEM = {'R','E','G','L','A','G','E',' ','D','U',' ','M','O','D','E',' '};
const uint8_t HEURE_OUVERTURE [16] PROGMEM = {'H','E','U','R','E',' ','O','U','V','E','R','T','U','R','E',' '};
const uint8_t HEURE_FERMETURE [16] PROGMEM = {'H','E','U','R','E',' ','F','E','R','M','E','T','U','R','E',' '};
const uint8_t OUVERTURE       [16] PROGMEM = {'O','U','V','E','R','T','U','R','E',' ',' ',' ',' ',' ',' ',' '};
const uint8_t FERMETURE       [16] PROGMEM = {'F','E','R','M','E','T','U','R','E',' ',' ',' ',' ',' ',' ',' '};
const uint8_t ENREGISTREE     [16] PROGMEM = {'E','N','R','E','G','I','S','T','R','E','E',' ',' ',' ',' ',' '};
const uint8_t DELAI_FERMETURE [16] PROGMEM = {'D','E','L','A','I',' ','F','E','R','M','E','T','U','R','E',' '};
const uint8_t RETOUR          [16] PROGMEM = {'R','E','T','O','U','R',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

const uint8_t REGLAGE_DATE    [16] PROGMEM = {'R','E','G','L','A','G','E',' ','D','A','T','E',' ',' ',' ',' '};
const uint8_t ET_HEURE        [16] PROGMEM = {'E','T',' ','H','E','U','R','E',' ',' ',' ',' ',' ',' ',' ',' '};
const uint8_t HEURE           [16] PROGMEM = {'H','E','U','R','E',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
const uint8_t DATE            [16] PROGMEM = {'D','A','T','E',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

const uint8_t INSTALLATION    [16] PROGMEM = {'I','N','S','T','A','L','L','A','T','I','O','N',' ',' ',' ',' '};
const uint8_t DE_LA_TRAPPE    [16] PROGMEM = {'D','E',' ','L','A',' ','T','R','A','P','P','E',' ',' ',' ',' '};


//menu d'acceuil
const uint8_t OUVRIRA_A       [16] PROGMEM = {'O','U','V','R','I','R','A',' ','A',' ',' ',' ',' ',' ',' ',' '};
const uint8_t FERMERA_A       [16] PROGMEM = {'F','E','R','M','E','R','A',' ','A',' ',' ',' ',' ',' ',' ',' '};
const uint8_t OUVERT_INFINI   [16] PROGMEM = {'O','U','V','E','R','T',' ','I','N','F','I','N','I',' ',' ',' '};
const uint8_t FERME_INFINI    [16] PROGMEM = {'F','E','R','M','E',' ','I','N','F','I','N','I',' ',' ',' ',' '};
const uint8_t PARAMETREZ_MOI  [16] PROGMEM = {'P','A','R','A','M','E','T','R','E','Z',' ','M','O','I',' ',' '};

//menu expert
const uint8_t CHARGE          [16] PROGMEM = {'C','H','A','R','G','E',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
const uint8_t CYCLAGE         [16] PROGMEM = {'C','Y','C','L','A','G','E',' ',' ',' ',' ',' ',' ',' ',' ',' '};
const uint8_t EPHEMERIDES     [16] PROGMEM = {'E','P','H','E','M','E','R','I','D','E','S',' ',' ',' ',' ',' '};
const uint8_t DEBUG           [16] PROGMEM = {'D','E','B','U','G',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
const uint8_t ERREUR_MENU     [16] PROGMEM = {'E','R','R','E','U','R',' ','M','E','N','U',' ',' ',' ',' ',' '};

//installation
const uint8_t REGLAGE         [16] PROGMEM = {'R','E','G','L','A','G','E',' ',' ',' ',' ',' ',' ',' ',' ',' '};
const uint8_t POSITION_FERMEE [16] PROGMEM = {'P','O','S','I','T','I','O','N',' ','F','E','R','M','E','E',' '};
const uint8_t POSITION_OUVERTE[16] PROGMEM = {'P','O','S','I','T','I','O','N',' ','O','U','V','E','R','T','E'};
const uint8_t POSITION_GPS    [16] PROGMEM = {'P','O','S','I','T','I','O','N',' ','G','P','S',' ',' ',' ',' '};
const uint8_t PAR_DEPARTEMENT [16] PROGMEM = {'P','A','R',' ','D','E','P','A','R','T','E','M','E','N','T',' '};
const uint8_t MANUELLE        [16] PROGMEM = {'M','A','N','U','E','L','L','E',' ',' ',' ',' ',' ',' ',' ',' '};

const uint8_t HEURE_NON_VALIDE[16] PROGMEM = {'H','E','U','R','E',' ','N','O','N',' ','V','A','L','I','D','E'};
const uint8_t DATE_NON_VALIDE [16] PROGMEM = {'D','A','T','E',' ','N','O','N',' ','V','A','L','I','D','E',' '};

const uint8_t ENTREZ_VOTRE    [16] PROGMEM = {'E','N','T','R','E','Z',' ','V','O','T','R','E',' ',' ',' ',' '};
const uint8_t DEPARTEMENT     [16] PROGMEM = {'D','E','P','A','R','T','E','M','E','N','T',':',' ',' ',' ',' '};
const uint8_t LATITUDE_NORD   [16] PROGMEM = {'L','A','T','I','T','U','D','E',' ','N','O','R','D',' ',' ',' '};
const uint8_t LATITUDE_SUD    [16] PROGMEM = {'L','A','T','I','T','U','D','E',' ','S','U','D',' ',' ',' ',' '};
const uint8_t LONGITUDE_OUEST [16] PROGMEM = {'L','O','N','G','I','T','U','D','E',' ','O','U','E','S','T',' '};
const uint8_t LONGITUDE_EST   [16] PROGMEM = {'L','O','N','G','I','T','U','D','E',' ','E','S','T',' ',' ',' '};






enum menu_t{
	menu_ouverture,
	menu_fermeture,
	menu_dateheure,
	menu_installation,
	menu_expert,
	menu_quitter
}menuPointer=menu_ouverture;

enum menu_modeture_t{
	menu_modeture_soleil,
	menu_modeture_fixe,
	menu_modeture_minimum
}menuModeturePointer=menu_modeture_soleil;

const uint8_t menuModetureText[3][16] PROGMEM ={
	{'S','O','L','E','I','L',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '},
	{'H','E','U','R','E',' ','F','I','X','E',' ',' ',' ',' ',' ',' '},
	{'M','I','N','I','M','U','M',' ',' ',' ',' ',' ',' ',' ',' ',' '}
};


const uint8_t menuText[menu_quitter][2][16] PROGMEM = {
	{{'R','E','G','L','A','G','E',' ','D','U',' ','M','O','D','E',' '},
	{'D','\'','O','U','V','E','R','T','U','R','E',' ',' ',' ',' ',' '}},
	{{'R','E','G','L','A','G','E',' ','D','U',' ','M','O','D','E',' '},
	{'D','E',' ','F','E','R','M','E','T','U','R','E',' ',' ',' ',' '}},
	{{'R','E','G','L','A','G','E',' ','D','A','T','E',' ',' ',' ',' '},
	{'E','T',' ','H','E','U','R','E',' ',' ',' ',' ',' ',' ',' ',' '}},
	{{'I','N','S','T','A','L','L','A','T','I','O','N',' ',' ',' ',' '},
	{'D','E',' ','L','A',' ','T','R','A','P','P','E',' ',' ',' ',' '}},
	{{'R','E','G','L','A','G','E',' ','D','U',' ','M','O','D','E',' '},
	{'E','X','P','E','R','T',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '}}
};

#endif /* TRAPAPOUL_TEXT_H_ */
