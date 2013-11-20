#ifndef _PHOTON_H
#define _PHOTON_H

#include "Vect.h"

class Photon {
public:
	Vect position;
	Color power;
	char phi, theta;
	short flag;
	int used;
	int bounce;
	Vect dir;

	Photon();
	Photon(Vect pPos, Color pPower, char pPi, char pTheta, Vect pDir);
};

Photon::Photon(){
	position = Vect(0,0,0);
	power = Color(0,0,0,0);
	phi = 0;
	theta = 0;
	used = 0;
}


Photon::Photon(Vect pPos, Color pPower, char pPhi, char pTheta, Vect pDir){
	position = pPos;
	power = pPower;
	phi = pPhi;
	theta = pTheta;
	used = 0;
	dir = pDir;
}



#endif // !_PHTOTN_H
