#ifndef _TRIANGLE_H
#define _TRIANGLE_H

#include <math.h>
#include "Object.h"
#include "Vect.h"
#include "Color.h"

class Triangle : public Object {
	Vect normal;
	double distance;
	Color color;
	Vect A, B, C;
public:
	Triangle();
	Triangle(Vect, Vect, Vect, Color);

	virtual Vect getTriangleNormal() {
		Vect CA (C.getVectX() - A.getVectX(), C.getVectY() - A.getVectY(), C.getVectZ() - A.getVectZ());
		Vect BA (B.getVectX() - A.getVectX(), B.getVectY() - A.getVectY(), B.getVectZ() - A.getVectZ());
		normal = CA.crossProduct(BA).normalize();
		//normal = normal.negtive();
		return normal;
	}
	virtual double getTriangleDistance() {
		normal = getTriangleNormal();
		distance = normal.dotProduct(A);
		return distance;
	}
	virtual Color getColor(){return color;}

	virtual Vect getNormalAt(Vect point){
		normal = getTriangleNormal();
		return normal;		
	}

	virtual double findIntersection(Ray ray){
		Vect ray_direction = ray.getRayDirection();
		Vect ray_origin = ray.getRayOrigin();

		double a = ray_direction.dotProduct(normal);
		normal = getTriangleNormal();
		distance = getTriangleDistance();

		if (a == 0){
			// ray parallel to plane
			return -1;
		}
		else{
			double b = normal.dotProduct(ray.getRayOrigin().vectAdd(normal.vectMult(distance).negtive())); //dafuq?
			double distance2Plane = -1*b/a;

			double Qx = ray_direction.vectMult(distance2Plane).getVectX() + ray_origin.getVectX();
			double Qy = ray_direction.vectMult(distance2Plane).getVectY() + ray_origin.getVectY();
			double Qz = ray_direction.vectMult(distance2Plane).getVectZ() + ray_origin.getVectZ();

			Vect Q (Qx, Qy, Qz);

			//[CAxQA]*n>=0   //dafuq
			Vect CA (C.getVectX() - A.getVectX(), C.getVectY() - A.getVectY(), C.getVectZ() - A.getVectZ());
			Vect QA (Q.getVectX() - A.getVectX(), Q.getVectY() - A.getVectY(), Q.getVectZ() - A.getVectZ());
			double test1 = CA.crossProduct(QA).dotProduct(normal);
			//[BCxQC]*n>=0
			Vect BC (B.getVectX() - C.getVectX(), B.getVectY() - C.getVectY(), B.getVectZ() - C.getVectZ());
			Vect QC (Q.getVectX() - C.getVectX(), Q.getVectY() - C.getVectY(), Q.getVectZ() - C.getVectZ());
			double test2 = BC.crossProduct(QC).dotProduct(normal);
			//[ABxQB]*n>=0
			Vect AB (A.getVectX() - B.getVectX(), A.getVectY() - B.getVectY(), A.getVectZ() - B.getVectZ());
			Vect QB (Q.getVectX() - B.getVectX(), Q.getVectY() - B.getVectY(), Q.getVectZ() - B.getVectZ());
			double test3 = AB.crossProduct(QB).dotProduct(normal);
			
			if (test1 >= 0 && test2 >= 0 && test3 >= 0) {
				//inside the triangle
			}
			else {
				return -1;
			}

			return -1*b/a;
		}
	}
};

Triangle::Triangle(){
	A = Vect (1,0,0);
	B = Vect (0,1,0);
	C = Vect (0,0,1);
	distance = 0;
	color = Color(0.5,0.5,0.5,0);
}

Triangle::Triangle(Vect pointA, Vect pointC, Vect pointB, Color colorValue){
	A = pointA;
	B = pointB;
	C = pointC;
	color = colorValue;
}

#endif