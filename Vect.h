#ifndef _VECT_H
#define _VECT_H

#include <math.h>


class Vect{
	 double x, y, z;
public:
	Vect(){
		x = 0;
		y = 0;
		z = 0;
	}

	Vect(double i , double j, double k){
		x = i;
		y = j;
		z = k;
	}

	inline double getVectX() const{return x;}
    inline double getVectY() const{return y;}
    inline double getVectZ() const{return z;}

    double operator[](int idx) const
    {
         switch(idx)
         {
             case 0: return x;
             case 1: return y;
             case 2: return z;
             default:
                 assert(0);
         }
        return 0;
    }

	double magnitude(){
		return sqrt(x*x+y*y+z*z);
	}
	
	Vect normalize(){
		double magnitude = sqrt(x*x+y*y+z*z);
		return Vect(x/magnitude, y/magnitude, z/magnitude);
	}

	Vect negtive(){
		return Vect(-x, -y, -z);
	}

	double dotProduct(Vect v){
		return x*v.getVectX() + y*v.getVectY() + z* v.getVectZ();
	}

	Vect crossProduct(Vect v){
		return Vect(y*v.getVectZ()-z*v.getVectY(), z*v.getVectX()-x*v.getVectZ(), x*v.getVectY()-y*v.getVectX());
	}

	Vect vectAdd(Vect v){
		return Vect (x+v.getVectX(), y+v.getVectY(), z+ v.getVectZ());
	}

    Vect vectMinus(Vect v){
        return Vect (x-v.getVectX(), y-v.getVectY(), z- v.getVectZ());
    }

	Vect vectMult (double scalar)const{
		return Vect (x*scalar, y *scalar, z*scalar);
	}

	double sqrDist(Vect v) const{
		return (x - v.x) * (x - v.x) + (y - v.y) * (y - v.y)
				+ (z - v.z) * (z - v.z);
	}
};



#endif