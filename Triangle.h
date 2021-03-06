#ifndef _TRIANGLE_H
#define _TRIANGLE_H

#include "System.h"
#include "Object.h"
#include "Vect.h"
#include "Color.h"
#include "Texture.h"
#include "TextureMap.h"

class Triangle : public Object {
	Vect normal;
	double distance;
	Color color;
	Vect A, B, C;
	Vect NormalA, NormalB, NormalC;
	float refraIdx;
	bool hasSetNormal;
	bool hasTexture;
	MyTexture texture_A,texture_B,texture_C;
	TextureMap * my_map;
public:

	virtual bool isTextured()
	{
		return this->hasTexture;
	}

	virtual Color getColor(const Vect& above_position)
	{
		if(hasTexture)
		{
			Vect position=getProjectPosition(above_position);
			Color result=getTextureColor(position);

			//result.setColorRed(result.getColorRed()*3);
			//result.setColorBlue(result.getColorBlue()*3);
			//result.setColorGreen(result.getColorGreen()*3);
			return result;
		}
		else
		{
			return getColor();
		}
		
	}

	Color getTextureColor(const Vect& position)
	{
		//double AP_x,AP_y,AP_z,AB_x,AB_y,AB_z,AC_x,AC_y,AC_z,c,b;
		//Vect NormalAB,NormalAC;
		//AP_x=position.getVectX()-A.getVectX();
		//AP_y=position.getVectY()-A.getVectY();
		//AP_z=position.getVectZ()-A.getVectZ();
		//AB_x=B.getVectX()-A.getVectX();
		//AB_y=B.getVectY()-A.getVectY();
		//AB_z=B.getVectZ()-A.getVectZ();
		//AC_x=C.getVectX()-A.getVectX();
		//AC_y=C.getVectY()-A.getVectY();
		//AC_z=C.getVectZ()-A.getVectZ();

		//c=(AP_y-(AB_y/AB_x)*AP_x)/(AC_y-(AB_y/AB_x)*AC_x);
		//b=(AP_x-c*AC_x)/AB_x;

		double AP_x,AP_y,AP_z,AB_x,AB_y,AB_z,AC_x,AC_y,AC_z,c,b;
		Vect NormalAB,NormalAC;
		AP_x=position.getVectX()-A.getVectX();
		AP_y=position.getVectY()-A.getVectY();
		AP_z=position.getVectZ()-A.getVectZ();
		AB_x=B.getVectX()-A.getVectX();
		AB_y=B.getVectY()-A.getVectY();
		AB_z=B.getVectZ()-A.getVectZ();
		AC_x=C.getVectX()-A.getVectX();
		AC_y=C.getVectY()-A.getVectY();
		AC_z=C.getVectZ()-A.getVectZ();

		if(AB_y*AC_x-AC_y*AB_x!=0)
		{
			c=(AP_x*AB_y-AP_y*AB_x)/(AB_y*AC_x-AC_y*AB_x);
		}
		else if(AC_y*AB_z-AC_z*AB_y!=0)
		{
			c=(AP_y*AB_z-AP_z*AB_y)/(AC_y*AB_z-AC_z*AB_y);
		}
		else
		{
			c=(AP_x*AB_z-AP_z-AB_x)/(AC_x*AB_z-AC_z*AB_x);
		}

		if(AB_x!=0)
			b=(AP_x-c*AC_x)/AB_x;
		else if(AB_y!=0)
		{
			b=(AP_y-c*AC_y)/AB_y;
		}
		else
			b=(AP_z-c*AC_z)/AB_z;

		MyTexture AB(texture_B.Vect2DAdd(texture_A.Vect2DNegate()));
		MyTexture AC(texture_C.Vect2DAdd(texture_A.Vect2DNegate()));
		MyTexture result(AB.Vect2DMulti(b).Vect2DAdd(AC.Vect2DMulti(c)).Vect2DAdd(texture_A));

		return my_map->GetColor(result);
	}

	void setTexture(MyTexture texture_A,MyTexture texture_C, MyTexture texture_B,bool use_texture,TextureMap* map)
	{
		this->texture_A=texture_A;
		this->texture_B=texture_B;
		this->texture_C=texture_C;
		this->my_map=map;
		hasTexture=use_texture;
	}

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
		hasTexture=false;
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
		hasTexture=false;
    }

	virtual void setNormals(Vect normal_a,Vect normal_c,Vect normal_b)
	{
		NormalA=normal_a;
		NormalB=normal_b;
		NormalC=normal_c;
		hasSetNormal = true;
	}
	virtual Vect getProjectPosition(const Vect& above_position)
	{
		Vect line_ap=above_position.vectAdd(A.negtive());
		Vect temp_normal=getTriangleNormal();
		return temp_normal.vectMult(line_ap.dotProduct(temp_normal)).negtive().vectAdd(line_ap).vectAdd(A);
	}

	virtual Vect getTriangleSmoothNormal(const Vect&  above_position)
	{
		/*Vect position=getProjectPosition(above_position);
		double AP_x,AP_y,AB_x,AB_y,AC_x,AC_y,c,b;
		Vect NormalAB,NormalAC;
		AP_x=position.getVectX()-A.getVectX();
		AP_y=position.getVectY()-A.getVectY();
		AB_x=B.getVectX()-A.getVectX();
		AB_y=B.getVectY()-A.getVectY();
		AC_x=C.getVectX()-A.getVectX();
		AC_y=C.getVectY()-A.getVectY();
		NormalAB=NormalB.vectAdd(NormalA.negtive());
		NormalAC=NormalC.vectAdd(NormalA.negtive());*/

		Vect position=getProjectPosition(above_position);
		double AP_x,AP_y,AP_z,AB_x,AB_y,AB_z,AC_x,AC_y,AC_z,c,b;
		Vect NormalAB,NormalAC;
		NormalAB=NormalB.vectAdd(NormalA.negtive());
		NormalAC=NormalC.vectAdd(NormalA.negtive());
		AP_x=position.getVectX()-A.getVectX();
		AP_y=position.getVectY()-A.getVectY();
		AP_z=position.getVectZ()-A.getVectZ();
		AB_x=B.getVectX()-A.getVectX();
		AB_y=B.getVectY()-A.getVectY();
		AB_z=B.getVectZ()-A.getVectZ();
		AC_x=C.getVectX()-A.getVectX();
		AC_y=C.getVectY()-A.getVectY();
		AC_z=C.getVectZ()-A.getVectZ();

		if(AB_y*AC_x-AC_y*AB_x!=0)
		{
			c=(AP_x*AB_y-AP_y*AB_x)/(AB_y*AC_x-AC_y*AB_x);
		}
		else if(AC_y*AB_z-AC_z*AB_y!=0)
		{
			c=(AP_y*AB_z-AP_z*AB_y)/(AC_y*AB_z-AC_z*AB_y);
		}
		else
		{
			c=(AP_x*AB_z-AP_z-AB_x)/(AC_x*AB_z-AC_z*AB_x);
		}

		if(AB_x!=0)
			b=(AP_x-c*AC_x)/AB_x;
		else if(AB_y!=0)
		{
			b=(AP_y-c*AC_y)/AB_y;
		}
		else
			b=(AP_z-c*AC_z)/AB_z;

		//c=(AP_y-(AB_y/AB_x)*AP_x)/(AC_y-(AB_y/AB_x)*AC_x);
		//b=(AP_x-c*AC_x)/AB_x;

		Vect result=NormalA.vectAdd(NormalAB.vectMult(b).vectAdd(NormalAC.vectMult(c))).normalize();
		if(result.dotProduct(getTriangleNormal())<0)
			result=result.negtive();
		return result;
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

	virtual Vect getNormalAt(const Vect& point){
		if(!hasSetNormal)
			return getTriangleNormal();
		else
			return getTriangleSmoothNormal(point);
	}

	virtual Vect getTangentAt(const Vect& point){
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

	virtual double findIntersection(const Ray& ray)  {
		Vect d = ray.getRayDirection();
		Vect o = ray.getRayOrigin();
        Vect e1 = B.vectMinus(A);
        Vect e2 = C.vectMinus(A);
        Vect q = d.crossProduct(e2);
        double a = e1.dotProduct(q);
        if(abs(a) < 0.00001)
            return -1;
        double f = 1 / a;
        Vect s = o.vectMinus(A);
        double u = f * s.dotProduct(q);
        if(u < 0.)
            return -1;
        Vect r = s.crossProduct(e1);
        double v = f * d.dotProduct(r);
        if(v < 0. || u + v > 1.)
            return -1;
        return f * r.dotProduct(e2);
	}
};


#endif