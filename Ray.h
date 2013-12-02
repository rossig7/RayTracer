#ifndef _RAY_H
#define _RAY_H

#include "Vect.h"


class Ray{
	Vect origin, direction;
public:
    Ray(){
        origin = Vect(0,0,0);
        direction = Vect(1,0,0);
    }

    Ray(Vect o, Vect d){
        origin = o;
        direction = d;
    }

	Vect getRayOrigin() const{return origin;}
    Vect getRayDirection () const{return direction;}
};



#endif