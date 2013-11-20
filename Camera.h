#ifndef _CAMERA_H
#define _CAMERA_H

#include "Vect.h"

class Camera{
	Vect camPos, camDir, camRight, camDown;
public:
	Camera();
	Camera(Vect, Vect, Vect, Vect);

	Vect getCameraPosition(){return camPos;}
	Vect getCameraDirection(){return camDir;}
	Vect getCameraRight(){return camRight;}
	Vect getCameraDown(){return camDown;}
};

Camera::Camera(){
	camPos = Vect(0,0,0);
	camDir = Vect(0,0,1);
	camRight = Vect(0,0,0);
	camDown = Vect(0,0,0);
}

Camera::Camera(Vect pos, Vect dir, Vect right, Vect down){
	camPos = pos;
	camDir = dir;
	camRight = right;
	camDown = down;
}

#endif