#ifndef _TRIANGLE_H
#define _TRIANGLE_H

#include "System.h"
#include "Object.h"
#include "Vect.h"
#include "Color.h"

class Triangle : public Object {
	Vect normal;
	double distance;
	Color color;
	Vect A, B, C;
	Vect NormalA, NormalB, NormalC;
	float refraIdx;
	bool hasSetNormal;
public:
    virtual BBox getBBox() const
    {
        BBox result;
        result.data[0][0] = min(A.getVectX(),min(B.getVectX(),C.getVectX()));
        result.data[0][1] = max(A.getVectX(),max(B.getVectX(),C.getVectX()));
        result.data[1][0] = min(A.getVectY(),min(B.getVectY(),C.getVectY()));
        result.data[1][1] = max(A.getVectY(),max(B.getVectY(),C.getVectY()));
        result.data[2][0] = min(A.getVectZ(),min(B.getVectZ(),C.getVectZ()));
        result.data[2][1] = max(A.getVectZ(),max(B.getVectZ(),C.getVectZ()));
        return result;
    };

    Triangle(){
        A = Vect (1,0,0);
        B = Vect (0,1,0);
        C = Vect (0,0,1);
        distance = 0;
        color = Color(0.5,0.5,0.5,0);
        refraIdx = 1;
    }

    Triangle(Vect pointA, Vect pointC, Vect pointB, Color colorValue, float refraIdxValue){
        A = pointA;
        B = pointB;
        C = pointC;
        NormalA = getTriangleNormal();
        NormalB = getTriangleNormal();
        NormalC = getTriangleNormal();
        color = colorValue;
        refraIdx = refraIdxValue;
        hasSetNormal = false;
    }

	virtual void setNormals(Vect normal_a,Vect normal_c,Vect normal_b)
	{
		NormalA=normal_a;
		NormalB=normal_b;
		NormalC=normal_c;
		hasSetNormal = true;
	}
	virtual Vect getProjectPosition(Vect above_position)
	{
		Vect line_ap=above_position.vectAdd(A.negtive());
		Vect temp_normal=getTriangleNormal();
		return temp_normal.vectMult(line_ap.dotProduct(temp_normal)).negtive().vectAdd(line_ap).vectAdd(A);
	}

	virtual Vect getTriangleSmoothNormal(Vect above_position)
	{
		Vect position=getProjectPosition(above_position);
		double AP_x,AP_y,AB_x,AB_y,AC_x,AC_y,c,b;
		Vect NormalAB,NormalAC;
		AP_x=position.getVectX()-A.getVectX();
		AP_y=position.getVectY()-A.getVectY();
		AB_x=B.getVectX()-A.getVectX();
		AB_y=B.getVectY()-A.getVectY();
		AC_x=C.getVectX()-A.getVectX();
		AC_y=C.getVectY()-A.getVectY();
		NormalAB=NormalB.vectAdd(NormalA.negtive());
		NormalAC=NormalC.vectAdd(NormalA.negtive());

		c=(AP_y-(AB_y/AB_x)*AP_x)/(AC_y-(AB_y/AB_x)*AC_x);
		b=(AP_x-c*AC_x)/AB_x;

		return NormalA.vectAdd(NormalAB.vectMult(b).vectAdd(NormalAC.vectMult(c))).normalize();
	}
	virtual Vect getTriangleNormal() {
		Vect CA (C.getVectX() - A.getVectX(), C.getVectY() - A.getVectY(), C.getVectZ() - A.getVectZ());
		Vect BA (B.getVectX() - A.getVectX(), B.getVectY() - A.getVectY(), B.getVectZ() - A.getVectZ());
		return CA.crossProduct(BA).normalize();
	}
	virtual double getTriangleDistance() {
		normal = getTriangleNormal();
		distance = normal.dotProduct(A);
		return distance;
	}
	virtual Color getColor(){return color;}

	virtual Vect getNormalAt(Vect point){
		if(!hasSetNormal)
			return getTriangleNormal();
		else
			return getTriangleSmoothNormal(point);
	}

	virtual Vect getTangentAt(Vect point){
		if(!hasSetNormal) {
			Vect CA (C.getVectX() - A.getVectX(), C.getVectY() - A.getVectY(), C.getVectZ() - A.getVectZ());
			return CA.normalize();
		}
		else {
			Vect Normal=getTriangleSmoothNormal(point);
			if(Normal.getVectY()==0&&Normal.getVectZ()==0)
				return Vect(0,1,0);
			else
			{
				return Normal.crossProduct(Vect(1,0,0)).normalize();
			}
		}
	}

	virtual float getRefraIdx() {return refraIdx;};

	virtual double findIntersection(Ray ray)  {
		Vect ray_direction = ray.getRayDirection();
		Vect ray_origin = ray.getRayOrigin();

        normal = getTriangleNormal();
        distance = getTriangleDistance();
		double a = ray_direction.dotProduct(normal);

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

			return -1*b/a - 0.000001;
		}
	}
};


#endif