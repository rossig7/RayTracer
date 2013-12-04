#ifndef _TEXTUREMAP_H
#define _TEXTUREMAP_H

#include <string>
#include <fstream>
#include "Color.h"

using namespace std;
class TextureMap
{
public:
	int x_res,y_res;
	Color** ColorMap;

	TextureMap()
	{

	}

	void TextureMapRead(string file_name)
	{
		FILE *fp;      
		if (!(fp = fopen(file_name.c_str(), "rb")))        
			return;             
		unsigned char header[54];  
		fread(header, sizeof(unsigned char), 54, fp);        
		x_res=y_res=0;
		for(int i=18;i<=21;i++)
		{
			y_res+=header[i]*pow(256,i-18);
		}
		for(int i=22;i<=25;i++)
		{
			x_res+=header[i]*pow(256,i-22);
		}
		
		ColorMap=(Color**)malloc(sizeof(Color*)*x_res+sizeof(Color)*x_res*y_res);
		Color* start_position=(Color*)(ColorMap+x_res);
		for(int i=0;i<x_res;i++)
			ColorMap[i]=start_position+i*y_res;
		//(Color*)malloc(sizeof(Color)*y_res);

		//unsigned char* image=(unsigned char*)malloc(sizeof(unsigned char)*x_res*y_res*3);
		//fread(image, sizeof(unsigned char), (size_t)(long)x_res * y_res * 3, fp);   
		for(int i=0;i<x_res;i++)
		{        
			for(int j=0;j<y_res;j++)
			{
				unsigned char image[3];
				fread(image, sizeof(unsigned char), 3, fp);
				ColorMap[i][j].setColorRed(image[0]/255.0);
				ColorMap[i][j].setColorGreen(image[1]/255.0);
				ColorMap[i][j].setColorBlue(image[2]/255.0);
				//if(image!=NULL)
					//delete(image);
				//int k=(i*y_res+j)*3;
				//ColorMap[i][j].setColorRed(image[k]/255.0);
				//ColorMap[i][j].setColorGreen(image[k+1]/255.0);
				//ColorMap[i][j].setColorBlue(image[k+2]/255.0);
			}
		}

		//free(image);
		//fclose(fp);
		//fclose(fp); 
		//delete(header);
	//int k=0;
		//fstream input(file_name,ios::in);
		//unsigned char temp_byte;
		//x_res=0;
		//y_res=0;
		//for(int i=0;i<54;i++)
		//{
		//	if(i==18)
		//	{
		//		for(;i<=21;i++)
		//		{
		//			input>>temp_byte;
		//			y_res=y_res+temp_byte*pow(256,i-18);
		//		}
		//		cout<<y_res<<" ";
		//		i--;
		//	}
		//	else if(i==22)
		//	{
		//		for(;i<=25;i++)
		//		{
		//			input>>temp_byte;
		//			x_res=x_res+temp_byte*pow(256,i-22);
		//		}
		//		cout<<x_res<<" texture bmp\n";
		//		i--;
		//	}
		//	else
		//	{
		//		input>>temp_byte;
		//	}
		//}

		//ColorMap=new Color*[x_res];
		//for(int i=0;i<x_res;i++)
		//	ColorMap[i]=new Color[y_res];

		//for(int i=0;i<x_res;i++)
		//{
		//	for(int j=0;j<y_res;j++)
		//	{
		//		Color new_color;
		//		unsigned char temp_red_byte,temp_blue_byte,temp_green_byte;
		//		input>>temp_red_byte>>temp_green_byte>>temp_blue_byte;
		//		new_color.setColorRed(temp_red_byte/255.0);
		//		//input>>temp_byte;
		//		new_color.setColorGreen(temp_green_byte/255.0);
		//		//input>>temp_byte;
		//		new_color.setColorBlue(temp_blue_byte/255.0);
		//		new_color.setColorSpecial(0);
		//		if(temp_red_byte==31&&temp_green_byte==31&&temp_blue_byte==39)
		//			int k=0;
		//		if(i==(x_res-1)&&j==(y_res-1))
		//			int m=0;
		//		ColorMap[i][j]=new_color;
		//	}
		//}
		//input.close();
	}

	double getBilinearInterpolate(double left_up,double left_down, double right_up, double right_down, double s, double t)
	{
		return left_up*(1-s)*(1-t)+left_down*s*(1-t)+right_up*(1-s)*t+right_down*s*t;
	}

	Color GetColor(double x, double y)
	{
		double dot_x=x;
		double dot_y=y;
		if(dot_x>0)
		{
			while(dot_x-1>0)
				dot_x-=1;
		}
		else if(dot_x<0)
		{
			do 
			{
				dot_x+=1;
			} while (dot_x<0);
		}
		
		if(dot_y>0)
		{
			while(dot_y-1>0)
				dot_y-=1;
		}
		else if(dot_y<0)
		{
			do 
			{
				dot_y+=1;
			} while (dot_y<0);
		}

		dot_x=dot_x*(x_res-1);
		dot_y=dot_y*(y_res-1);
		int floor_x=int(dot_x);
		int ceil_x=(floor_x+1)>=x_res?x_res-1:floor_x+1;
		int floor_y=int(dot_y);
		int ceil_y=(floor_y+1)>=y_res?y_res-1:floor_y+1;

		Color left_up=ColorMap[floor_x][floor_y];
		Color left_down=ColorMap[ceil_x][floor_y];
		Color right_up=ColorMap[floor_x][ceil_y];
		Color right_down=ColorMap[ceil_x][ceil_y];

		Color result_color;
		double s=dot_x-floor_x;
		double t=dot_y-floor_y;
		result_color.setColorRed(getBilinearInterpolate(left_up.getColorRed(),left_down.getColorRed(),right_up.getColorRed(),right_down.getColorRed(),s,t));
		result_color.setColorGreen(getBilinearInterpolate(left_up.getColorGreen(),left_down.getColorGreen(),right_up.getColorGreen(),right_down.getColorGreen(),s,t));
		result_color.setColorBlue(getBilinearInterpolate(left_up.getColorBlue(),left_down.getColorBlue(),right_up.getColorBlue(),right_down.getColorBlue(),s,t));
		result_color.setColorSpecial(0);
		return result_color;
	}

	Color GetColor(MyTexture t)
	{
		return GetColor(t.ReturnU(),t.ReturnV());
	}
};

#endif