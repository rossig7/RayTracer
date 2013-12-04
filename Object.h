#ifndef _OBJECT_H
#define _OBJECT_H

#include "Ray.h"
#include "Vect.h"
#include "Color.h"
#include "BVH.h"

struct BBox;

class Object{
public:
    Object(){};

	virtual Color getColor(){return Color(0.0,0.0,0.0,0);}
	virtual Color getColor(const Vect& position){return Color(0.0,0.0,0.0,0);}
    virtual BBox getBBox() const = 0;
	virtual double findIntersection(const Ray& ray)  {
		return 0;
	}

	virtual bool isTextured()
	{
		return false;
	}
	virtual Vect getNormalAt(const Vect& point)  = 0;
	virtual Vect getTangentAt(const Vect& point)  = 0;
	virtual float getRefraIdx() {return 1;}
};



#endif