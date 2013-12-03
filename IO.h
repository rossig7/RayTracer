#ifndef _IO_H
#define _IO_H

#include "System.h"
#include "Triangle.h"
#include "Vect.h"
#include "Color.h"
#include "Object.h"
#include "Texture.h"
#include "TextureMap.h"

struct RGBType {
    double r;
    double g;
    double b;
};

void saveBmp(const char *filename, int w, int h, int dpi, RGBType *data);
void makeCube(vector<Object *>& objects,Vect corner1, Vect corner2, Color color);
void makeCornellBox(vector<Object *>& objects, Vect corner1, Vect corner2);

class ObjReader
{
	string file_name;
	Color object_color;
	float refraIdxValue;
	float x_offset,y_offset,z_offset;
	vector <Vect> points;
	vector <Vect> normals;
	vector <MyTexture> textures; 
	bool use_texture;
	TextureMap * my_map;
public:
	ObjReader(string file_name,Color color, float refraIdxValue,float x_offset,float y_offset,float z_offset,TextureMap * texture_map,bool use_texture)
	{
		this->file_name=file_name;
		this->object_color=color;
		this->refraIdxValue=refraIdxValue;
		this->x_offset=x_offset;
		this->y_offset=y_offset;
		this->z_offset=z_offset;
		this->my_map=texture_map;
		this->use_texture=use_texture;
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
		double value_one,value_two,value_three;
		char* string_one;
		char* string_two;
		char* string_three;
		string_one=new char[20];
		string_two=new char[20];
		string_three=new char[20];
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
				int normal_x,normal_y,normal_z;
				int texture_x,texture_y,texture_z;

				value_one=atoi(strtok(string_one,"/"))-1;
				texture_x=atoi(strtok(NULL,"/"))-1;
				normal_x=atoi(strtok(NULL,"/"))-1;

				value_two=atoi(strtok(string_two,"/"))-1;
				texture_y=atoi(strtok(NULL,"/"))-1;
				normal_y=atoi(strtok(NULL,"/"))-1;

				value_three=atoi(strtok(string_three,"/"))-1;
				texture_z=atoi(strtok(NULL,"/"))-1;
				normal_z=atoi(strtok(NULL,"/"))-1;

				Triangle* triangle=new Triangle(points.at(value_one),points.at(value_two),points.at(value_three),object_color,refraIdxValue);
				triangle->setNormals(normals.at(normal_x),normals.at(normal_y),normals.at(normal_z));
				triangle->setTexture(textures.at(texture_x),textures.at(texture_y),textures.at(texture_z),this->use_texture,my_map);
				scence_objects->push_back(triangle);
			}
			else if(head=="vn")
			{
				input>>value_one>>value_two>>value_three;
				Vect normal(value_one,value_two,value_three);
				normals.push_back(normal);
			}
			else if(head=="vt")
			{
				input>>value_one>>value_two;
				MyTexture new_texture(value_one,value_two);
				textures.push_back(new_texture);
			}
		}
		input.close();
		points.clear();
		return true;
	}
};

#endif