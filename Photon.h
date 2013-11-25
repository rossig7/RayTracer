#ifndef _PHOTON_H
#define _PHOTON_H

#include "Vect.h"
#include "Color.h"

class Photon {
public:
	Vect position;
	Color power;
	char phi, theta;
	Vect dir;

	Photon(){
		position = Vect(0,0,0);
		power = Color(0,0,0,0);
		phi = 0;
		theta = 0;
	}

	Photon(Vect pPos, Color pPower, char pPhi, char pTheta, Vect pDir){
		position = pPos;
		power = pPower;
		phi = pPhi;
		theta = pTheta;
		dir = pDir;
	}
};





#endif // !_PHTOTN_H
