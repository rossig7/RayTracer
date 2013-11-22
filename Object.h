#ifndef _OBJECT_H
#define _OBJECT_H

#include "Ray.h"
#include "Vect.h"
#include "Color.h"

class Object{
public:
	Object();

	virtual Color getColor(){return Color(0.0,0.0,0.0,0);}

	virtual double findIntersection(Ray ray){
		return 0;
	}

	virtual Vect getNormalAt(Vect point) = 0;
	virtual float getRefraIdx() {return 1;}
};

Object::Object(){}

#endif