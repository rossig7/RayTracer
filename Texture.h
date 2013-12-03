#ifndef _TEXTURE_H
#define _TEXTURE_H
class MyTexture
{
public:
	double x,y;
	MyTexture(double x,double y)
	{
		this->x=x;
		this->y=y;
	}
	MyTexture()
	{
		this->x=this->y=0;
	}

	//Texture(Texture t)
	//{
	//	this->x=t.x;
	//	this->y=t.y;
	//}

	MyTexture Vect2DMulti(double x)
	{
		return MyTexture(this->x*x,this->y*x);
	}

	MyTexture Vect2DAdd(MyTexture t)
	{
		return MyTexture(this->x+t.x,this->y+t.y);
	}

	MyTexture Vect2DNegate()
	{
		return MyTexture(this->x*(-1),(-1)*this->y);
	}

	double ReturnU()
	{
		return x;
	}

	double ReturnV()
	{
		return y;
	}

};

#endif