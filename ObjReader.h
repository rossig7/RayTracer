#include <string>
#include <vector>
#include "Triangle.h"
#include "Vect.h"
#include "Color.h"
#include "Object.h"
#include <fstream>
#include <iostream>
using namespace std;
class ObjReader
{
	string file_name;
	Color object_color;
	float refraIdxValue;
	float x_offset,y_offset,z_offset;
	vector <Vect> points;
public:
	ObjReader(string file_name,Color color, float refraIdxValue,float x_offset,float y_offset,float z_offset)
	{
		this->file_name=file_name;
		this->object_color=color;
		this->refraIdxValue=refraIdxValue;
		this->x_offset=x_offset;
		this->y_offset=y_offset;
		this->z_offset=z_offset;
	}

	ObjReader()
	{
		
	}

	void SetPara(string file_name,Color color, float refraIdxValue,float x_offset,float y_offset,float z_offset)
	{
		this->file_name=file_name;
		this->object_color=color;
		this->refraIdxValue=refraIdxValue;
		this->x_offset=x_offset;
		this->y_offset=y_offset;
		this->z_offset=z_offset;
	}

	bool ReadContent(vector<Object*>* scence_objects)
	{
		fstream input(file_name,ios::in);
		string head;
		float value_one,value_two,value_three;
		string string_one,string_two,string_three;
		if(!input)
			return false;

		while(!input.eof())
		{
			input>>head;
			if(head=="v")
			{
				input>>value_one>>value_two>>value_three;
				Vect point(value_one+x_offset,value_two+y_offset,value_three+z_offset);
				points.push_back(point);
			}
			else if(head=="f")
			{
				input>>string_one>>string_two>>string_three;
				string_one=string_one.substr(0,string_one.find("/"));
				string_two=string_two.substr(0,string_two.find("/"));
				string_three=string_three.substr(0,string_three.find("/"));

				scence_objects->push_back(new Triangle(points.at(atoi(string_one.c_str())-1),points.at(atoi(string_two.c_str())-1),points.at(atoi(string_three.c_str())-1),object_color,refraIdxValue));
			}
		}
		input.close();
		points.clear();
		return true;
	}
};