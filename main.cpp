#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <queue>

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>

#include <random>

#include "Vect.h"
#include "Ray.h"
#include "Camera.h"
#include "Color.h"
#include "Light.h"
#include "Object.h"
#include "Source.h"
#include "Sphere.h"
#include "Plane.h"
#include "Triangle.h"
#include "Photon.h"
#include "Kdtree.h"

#include <omp.h>

/*NOTICE: a good value pair would be (50000,1200), too large PHOTONMUM/PHOTONUSE will produce spotty result, too small will produce over-blurred shadow*/
#define PHOTONMUM 50000
#define PHOTONUSE 1200

#define PI 3.1415926
#define BOUNCE 3
#define NATUREE 2.71828
#define FRESNEL
#define TRACING_DEPTH 4  // Depth must >= 4
//#define GLOSSY
/*define GLOSSY will enable glossy reflection and refraction for both balls in the scene. VERY SLOW*/
#define GLOSSY_SAMPLE 16

using namespace std;

vector<Photon *> photons;
vector<Object *> scene_objects;

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis_rand(0, 1);

struct RGBType {
	double r;
	double g;
	double b;
};

void saveBmp(const char *filename, int w, int h, int dpi, RGBType *data)
{
	FILE *f;
	int k = w * h;
	int s = 4 * k;
	int filesize = 54 + s;

	double factor = 39.375;
	int m = static_cast<int>(factor);

	int ppm = dpi * m;

	unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
	unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0};

	bmpFileHeader[2] = (unsigned char) (filesize);
	bmpFileHeader[3] = (unsigned char) (filesize >> 8);
	bmpFileHeader[4] = (unsigned char) (filesize >> 16);
	bmpFileHeader[5] = (unsigned char) (filesize >> 24);

	bmpInfoHeader[4] = (unsigned char) (w);
	bmpInfoHeader[5] = (unsigned char) (w >> 8);
	bmpInfoHeader[6] = (unsigned char) (w >> 16);
	bmpInfoHeader[7] = (unsigned char) (w >> 24);

	bmpInfoHeader[8] = (unsigned char) (h);
	bmpInfoHeader[9] = (unsigned char) (h >> 8);
	bmpInfoHeader[10] = (unsigned char) (h >> 16);
	bmpInfoHeader[11] = (unsigned char) (h >> 24);

	bmpInfoHeader[21] = (unsigned char) (s);
	bmpInfoHeader[22] = (unsigned char) (s >> 8);
	bmpInfoHeader[23] = (unsigned char) (s >> 16);
	bmpInfoHeader[24] = (unsigned char) (s >> 24);

	bmpInfoHeader[25] = (unsigned char) (ppm);
	bmpInfoHeader[26] = (unsigned char) (ppm >> 8);
	bmpInfoHeader[27] = (unsigned char) (ppm >> 16);
	bmpInfoHeader[28] = (unsigned char) (ppm >> 24);

	bmpInfoHeader[29] = (unsigned char) (ppm);
	bmpInfoHeader[30] = (unsigned char) (ppm >> 8);
	bmpInfoHeader[31] = (unsigned char) (ppm >> 16);
	bmpInfoHeader[32] = (unsigned char) (ppm >> 24);

	f = fopen(filename, "wb");

	fwrite(bmpFileHeader, 1, 14, f);
	fwrite(bmpInfoHeader, 1, 40, f);

	for (int i = 0; i < k; i++) {
		RGBType rgb = data[i];

		double red = data[i].r * 255;
		double green = data[i].g * 255;
		double blue = data[i].b * 255;

		unsigned char color[3] = {(unsigned char) floor(blue), (unsigned char) floor(green), (unsigned char) floor(red)};

		fwrite(color, 1, 3, f);
	}

	fclose(f);
}

int winningObjectIndex(vector<double> object_intersections)
{
	//return the index of winning intersection
	int index_of_minimum_value;

	if (object_intersections.size() == 0) {
		return -1;
	}
	else if (object_intersections.size() == 1) {
		if (object_intersections.at(0) > 0) {
			return 0;
		}
		else {
			return -1;
		}
	}
	else {
		double max = 0;
		for (int i = 0; i < object_intersections.size(); i++) {
			if (max < object_intersections.at(i))
				max = object_intersections.at(i);
		}

		if (max > 0) {
			for (int index = 0; index < object_intersections.size(); index++) {
				if (object_intersections.at(index) > 0 && object_intersections.at(index) <= max) {
					max = object_intersections.at(index);
					index_of_minimum_value = index;
				}
			}
			return index_of_minimum_value;
		}
		else {
			return -1;
		}
	}
}

Color storePhoton(Vect intersection_position, Vect intersecting_ray_direction, vector<Object *> scene_objects, int index_of_winning_object, double accuracy, double ambientLight, Color lightColor, int bounce)
{
	Color winning_object_color = scene_objects.at(index_of_winning_object)->getColor();

	Vect winning_object_normal = scene_objects.at(index_of_winning_object)->getNormalAt(intersection_position);

	//Color final_color = winning_object_color/*.colorScalar(ambientLight)*/;

	Color final_color;

	final_color.setColorRed(min(winning_object_color.getColorRed(), lightColor.getColorRed()));
	final_color.setColorGreen(min(winning_object_color.getColorGreen(), lightColor.getColorGreen()));
	final_color.setColorBlue(min(winning_object_color.getColorBlue(), lightColor.getColorBlue()));

	//char phi = 255 * (atan2(intersecting_ray_direction.getVectX(), intersecting_ray_direction.getVectY()) + PI) / (2 * PI);
	//char theta = 255 * acos(intersecting_ray_direction.getVectX()) / PI;

	char phi = 0;
	char theta = 0;

	Photon* currentPhoton  = new Photon(intersection_position, lightColor, phi, theta, intersecting_ray_direction);
#pragma omp critical
	{
		photons.push_back(currentPhoton);
	}
	return final_color.clip();
}

Vect computeReflectionDir(Vect winning_object_normal, Vect intersecting_ray_direction) {
	double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
	Vect scalar1 = winning_object_normal.vectMult(dot1);
	Vect add1 = scalar1.vectAdd(intersecting_ray_direction);
	Vect scalar2 = add1.vectMult(2);
	Vect add2 = intersecting_ray_direction.negtive().vectAdd(scalar2);

	return add2.normalize();
}

Vect computeGlossyRay(Vect reflection_direction, Vect winning_object_normal, double glossiness) {
	double a = dis_rand(gen);
	double b = dis_rand(gen);

	double theta = acos(pow(1-a, glossiness));
	double phi = 2 * PI * b;

	double x = sin(theta) * cos(phi);
	double z = sin(theta) * sin(phi);
	//double z = cos(theta);		

	Vect u = reflection_direction.crossProduct(winning_object_normal);
	Vect v = reflection_direction.crossProduct(u);

	Vect world_ref_ray_dir = u.vectMult(x).vectAdd(v.vectMult(z).vectAdd(reflection_direction));
	
	return world_ref_ray_dir.normalize();
}

Color getColorAt(Vect intersection_position, Vect intersecting_ray_direction, vector<Object *> scene_objects, int index_of_winning_object, double accuracy, double ambientLight, int depth, KDTree* kdtree)
{
	Color winning_object_color = scene_objects.at(index_of_winning_object)->getColor();
	Vect winning_object_normal = scene_objects.at(index_of_winning_object)->getNormalAt(intersection_position);

	Color final_color = winning_object_color.colorScalar(ambientLight);
	if (depth > TRACING_DEPTH) return final_color.clip();


	if (winning_object_color.getColorSpecial() > 0) {  // have reflection
		Vect reflection_direction = computeReflectionDir(winning_object_normal, intersecting_ray_direction);

		double sample;
		double sampledGlossyReflectColor = 0;
		Color sampleGlossyReflectColor(0,0,0,0);
		double colorSpecial = winning_object_color.getColorSpecial();

		if (colorSpecial*100.0 - floor(colorSpecial*100.0) - 0.7654 < 0.0001 && colorSpecial*100.0 - floor(colorSpecial*100.0) - 0.7654 > 0) 
			sample = GLOSSY_SAMPLE;
		else sample = 1;

		for (int i = 0; i < sample; i ++) {
			Vect world_ref_ray_dir;
			if (sample != 1)
				world_ref_ray_dir = computeGlossyRay(reflection_direction, winning_object_normal, 0.2);
			else
				world_ref_ray_dir = reflection_direction;

			Ray reflection_ray(intersection_position, world_ref_ray_dir);
			vector<double> reflection_intersections;

			for (int reflection_index = 0; reflection_index < scene_objects.size(); reflection_index++) {
				reflection_intersections.push_back(scene_objects.at(reflection_index)->findIntersection(reflection_ray));
			}

			int index_of_winning_object_reflection = winningObjectIndex(reflection_intersections);
			if (index_of_winning_object_reflection != -1) {
				//no miss
				if (reflection_intersections.at(index_of_winning_object_reflection) > accuracy) {
					Vect reflection_intersection_position = intersection_position.vectAdd(reflection_direction.vectMult(reflection_intersections.at(index_of_winning_object_reflection)));
					Vect reflection_intersection_direction = reflection_direction;

					Color reflection_intersection_color = getColorAt(reflection_intersection_position, reflection_intersection_direction, scene_objects, index_of_winning_object_reflection, accuracy, ambientLight, depth+1, kdtree);

					if (winning_object_color.getColorSpecial() > 1) 
						sampleGlossyReflectColor = sampleGlossyReflectColor.colorAdd(reflection_intersection_color.colorScalar(2 - winning_object_color.getColorSpecial()));
					else
						sampleGlossyReflectColor = sampleGlossyReflectColor.colorAdd(reflection_intersection_color);
					sampledGlossyReflectColor++;
				}
			}
		}
		sampleGlossyReflectColor = sampleGlossyReflectColor.colorScalar(1.0/sample);
		final_color = final_color.colorAdd(sampleGlossyReflectColor);

		if (winning_object_color.getColorSpecial() > 1) { // have reflection and refraction
			double sampledGlossyRefractColor = 0;
			Color sampleGlossyRefractColor(0,0,0,0);

			for (int i = 0; i < sample; i ++) {
				Vect world_ref_ray_dir;

				double refractance = winning_object_color.getColorSpecial() - 1;

				double n1n2 = 1.0 / scene_objects.at(index_of_winning_object)->getRefraIdx();
				double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());  // NL

				if (dot1 < 0) {
					winning_object_normal = winning_object_normal.negtive();
					dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
					n1n2 = 1.0 / n1n2;
				}

				double nNL = n1n2 * dot1;  //n*NL
				double underSQRT = 1 - n1n2 * n1n2 * (1 - dot1 * dot1); // 1-n^2*(1-(NL)^2)

				if (underSQRT > 0) {
					double coeffN = nNL - sqrt(underSQRT);   //n*NL - sqrt(1-n^2*(1-(NL)^2))
					Vect reflection_direction = winning_object_normal.vectMult(coeffN).vectAdd((intersecting_ray_direction.negtive().vectMult(n1n2)).negtive());

					if (sample != 1)
						world_ref_ray_dir = computeGlossyRay(reflection_direction, winning_object_normal, 0.2);
					else
						world_ref_ray_dir = reflection_direction;

					Ray reflection_ray(intersection_position, world_ref_ray_dir);

					vector<double> reflection_intersections;
					for (int reflection_index = 0; reflection_index < scene_objects.size(); reflection_index++) {
						reflection_intersections.push_back(scene_objects.at(reflection_index)->findIntersection(reflection_ray));
					}
					int index_of_winning_object_reflection = winningObjectIndex(reflection_intersections);

					if (index_of_winning_object_reflection != -1) {
						//no miss
						double intersect = reflection_intersections.at(index_of_winning_object_reflection);
						if (intersect > accuracy) {
							Vect reflection_intersection_position = intersection_position.vectAdd(reflection_direction.vectMult(reflection_intersections.at(index_of_winning_object_reflection)));
							Vect reflection_intersection_direction = reflection_direction;

							Color reflection_intersection_color = getColorAt(reflection_intersection_position, reflection_intersection_direction, scene_objects, index_of_winning_object_reflection, accuracy, ambientLight, depth+1, kdtree);

							sampleGlossyRefractColor = sampleGlossyRefractColor.colorAdd(reflection_intersection_color.colorScalar(refractance));
							sampledGlossyRefractColor++;
						}
					}
				}
				sampleGlossyRefractColor = sampleGlossyRefractColor.colorScalar(1.0/sample);
				final_color = final_color.colorAdd(sampleGlossyRefractColor);
			}
		}
	}
	else {
		vector<Photon *> photon_find = kdtree->findKNN(PHOTONUSE, intersection_position);
		float maxDistSqr = -1;
		int k = 1.5;

		for(int i = 0; i < photon_find.size(); i++)
		{
			float distanceSqr = photon_find[i]->position.sqrDist(intersection_position);
			if (distanceSqr > maxDistSqr) maxDistSqr = distanceSqr;
		}

		//cout<<photon_find.size()<<" "<< maxDistSqr <<endl;

		for(int i = 0; i < photon_find.size(); i++)
		{
			float distanceSqr = photon_find[i]->position.sqrDist(intersection_position);

//#define CONE
#ifdef CONE  // cone filter
			float weight = 1 - sqrt(distanceSqr) / (k * sqrt(maxDistSqr));  // cone filter
#else  // gaussian filter
			float weight = 0.918 * (1 - (1 - pow(NATUREE, -1.953*(distanceSqr)/(2*maxDistSqr)))/(1 - pow(NATUREE, -1.953)));
#endif
			Vect light_direction = photon_find[i]->dir.negtive();

			float cosine_angle = winning_object_normal.dotProduct(light_direction);
			if (cosine_angle < 0) cosine_angle = 0;

			final_color = final_color.colorAdd(winning_object_color.colorMultiply(photon_find[i]->power).colorScalar(cosine_angle * weight));
		}
		if (photon_find.size() != 0) {
			//final_color = final_color.colorScalar(1.0 / (photon_find.size()));
#ifdef CONE
			final_color = final_color.colorScalar(1.0/(PHOTONMUM/16*PI*maxDistSqr * (1 - 2/(3*k))));
#else
			final_color = final_color.colorScalar(1.0/(PHOTONMUM/16*PI*maxDistSqr));
#endif
		}
	}
	return final_color.clip();
}

double FresnelEquation(float cosThetaI, float n1, float n2, int type){
	// type: 0:reflectance, 1:refractance
	float sin2ThetaT = (n1/n2)*(n1/n2)*(1-cosThetaI*cosThetaI);
	float cosThetaT = sqrt(1-sin2ThetaT);

	float Rspolarize = (n1*cosThetaI - n2*cosThetaT) / (n1*cosThetaI + n2*cosThetaT);
	float Rppolarize = (n2*cosThetaI - n1*cosThetaT) / (n2*cosThetaI + n1*cosThetaT);

	if (type == 0)
		return (Rspolarize*Rspolarize + Rppolarize*Rppolarize) / 2;
	else
		return 1-(Rspolarize*Rspolarize + Rppolarize*Rppolarize) / 2;
}

void photonEmission(Ray photon_ray, Vect photon_ray_direction, vector<Object *> scene_objects, double accuracy, double ambientLight, Color lightColor, int bounce) {
	//cout << bounce << endl;
	if (bounce> BOUNCE)
		return;
	
	vector<double> intersections;

	for (int index = 0; index < scene_objects.size(); index++) {
		intersections.push_back(scene_objects.at(index)->findIntersection(photon_ray));
	}

	int index_of_winning_object = winningObjectIndex(intersections);

	if (index_of_winning_object == -1) {
		//if (bounce == 0) pn--;
		return;
	}

	if (intersections.at(index_of_winning_object) > accuracy) {
		Vect intersection_position = photon_ray.getRayOrigin().vectAdd(photon_ray_direction.vectMult(intersections.at(index_of_winning_object)));
		Vect intersecting_ray_direction = photon_ray_direction;
		Vect winning_object_normal = scene_objects.at(index_of_winning_object)->getNormalAt(intersection_position);

		float reflectance = 1;
		int refractMask[3] = {1,1,1};
		bool canTransmit = true;

		if (scene_objects.at(index_of_winning_object)->getColor().getColorSpecial() == 0) {
			lightColor = storePhoton(intersection_position, intersecting_ray_direction, scene_objects, index_of_winning_object, accuracy, ambientLight, lightColor, bounce);
			lightColor = lightColor.colorScalar(0.8);

			// not use Russian Roulette for lambert model 
			/*
			float cosTheta = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
			float n2 = scene_objects.at(index_of_winning_object)->getRefraIdx();

			reflectance = FresnelEquation(cosTheta, 1, n2, 0);
			
			int roll = rand() % 100;
			if (roll > reflectance * 100) {   // absorb rate of 0.2. TODO: add absorb/refraction/refraction rate
				canTransmit = false;
			}
			*/
		}
		else if (scene_objects.at(index_of_winning_object)->getColor().getColorSpecial() < 1) {
			float cosTheta = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
			float n2 = scene_objects.at(index_of_winning_object)->getRefraIdx();

			reflectance = FresnelEquation(cosTheta, 1, n2, 0);
			int roll = rand() % 100;
			if (roll > reflectance * 100) {   // absorb rate of 0.2. TODO: add absorb/refraction/refraction rate
				canTransmit = false;
			}
		}
		else if (scene_objects.at(index_of_winning_object)->getColor().getColorSpecial() < 2) {
			for (int i = 0; i < 3; i++) {
				float cosTheta = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
				float n2 = scene_objects.at(index_of_winning_object)->getRefraIdx()-0.01 + i/100.0;

				if (cosTheta > 0) 
					reflectance = FresnelEquation(cosTheta, 1, n2, 1);
				else {
					cosTheta = winning_object_normal.negtive().dotProduct(intersecting_ray_direction.negtive());
					reflectance = FresnelEquation(cosTheta, n2, 1, 1);
				}
				int roll = rand() % 100;
				if (roll > reflectance * 100) {   // absorb rate of 0.2. TODO: add absorb/refraction/refraction rate
					refractMask[i] = 0;
				}
			}
		}

		if (canTransmit) {
			if (scene_objects.at(index_of_winning_object)->getColor().getColorSpecial() == 0) {  // Lambert model
				double Zeta1 = dis_rand(gen);
				double Zeta2 = dis_rand(gen);
				
				double theta = acos(sqrt(Zeta1));
				double phi = 2 * PI * Zeta2;

				double x = sin(theta) * cos(phi);
				double z = sin(theta) * sin(phi);
				double y = cos(theta);		

				//Vect u = reflection_direction.crossProduct(winning_object_normal);
				Vect u = scene_objects.at(index_of_winning_object)->getTangentAt(intersection_position);
				Vect v = winning_object_normal.crossProduct(u);

				Vect world_ref_ray_dir = u.vectMult(x).vectAdd(v.vectMult(z).vectAdd(winning_object_normal).vectMult(y));
				world_ref_ray_dir = world_ref_ray_dir.normalize();
				//Vect reflection_dir(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));
				//reflection_dir = reflection_dir.normalize();

				photon_ray = Ray(intersection_position, world_ref_ray_dir);
				photon_ray_direction = world_ref_ray_dir;
				photonEmission (photon_ray, photon_ray_direction, scene_objects, accuracy, ambientLight, lightColor, bounce+1);		
			}
			else if (scene_objects.at(index_of_winning_object)->getColor().getColorSpecial() < 1) {
				//lightColor = lightColor.colorScalar(0.7);
				double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive()); // N*L
				Vect scalar1 = winning_object_normal.vectMult(dot1); // (N*L)*N
				Vect add1 = scalar1.vectAdd(intersecting_ray_direction);
				Vect scalar2 = add1.vectMult(2);
				Vect add2 = intersecting_ray_direction.negtive().vectAdd(scalar2);
				Vect reflection_dir = add2.normalize();

				photon_ray = Ray(intersection_position, reflection_dir);
				photon_ray_direction = reflection_dir;
				photonEmission (photon_ray, photon_ray_direction, scene_objects, accuracy, ambientLight, lightColor, bounce+1);
			}
			else {
				for (int i = 0; i < 3; i++) {
					if (refractMask == 0) continue;
					double n1n2 = 1.0 / (scene_objects.at(index_of_winning_object)->getRefraIdx()-0.01 + i/100.0);
					double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());  // NL
	
					if (dot1 < 0) {
						winning_object_normal = winning_object_normal.negtive();
						dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
						n1n2 = 1.0 / n1n2;
					}
	
					double nNL = n1n2 * dot1;  //n*NL
					double underSQRT = 1 - n1n2 * n1n2 * (1 - dot1 * dot1); // 1-n^2*(1-(NL)^2)
	
					if (underSQRT > 0) {
						Color refractColor;
						double coeffN = nNL - sqrt(underSQRT);   //n*NL - sqrt(1-n^2*(1-(NL)^2))
						Vect refraction_direction = winning_object_normal.vectMult(coeffN).vectAdd((intersecting_ray_direction.negtive().vectMult(n1n2)).negtive());
	
						Ray refraction_ray(intersection_position, refraction_direction);
	
						photon_ray = refraction_ray;
						photon_ray_direction = refraction_direction;
						//bounce--;
						//lightColor = white_light;
						if (i == 0) {
							refractColor = lightColor;
							refractColor.setColorGreen(0);
							refractColor.setColorBlue(0);
						}
						else if (i == 1) {
							refractColor = lightColor;
							refractColor.setColorRed(0);
							refractColor.setColorBlue(0);
						}
						else if (i == 2) {
							refractColor = lightColor;
							refractColor.setColorRed(0);
							refractColor.setColorGreen(0);
						}
						if (!(refractColor.getColorRed() == 0 && refractColor.getColorGreen() == 0 && refractColor.getColorBlue() == 0))
							photonEmission (photon_ray, photon_ray_direction, scene_objects, accuracy, ambientLight, refractColor, bounce+1);
					}		
				}
			}			
		}		
	}
}


void makeCube(Vect corner1, Vect corner2, Color color)
{
	double c1x = corner1.getVectX();
	double c1y = corner1.getVectY();
	double c1z = corner1.getVectZ();

	double c2x = corner2.getVectX();
	double c2y = corner2.getVectY();
	double c2z = corner2.getVectZ();

	Vect A (c2x, c1y, c1z);
	Vect B (c2x, c1y, c2z);
	Vect C (c1x, c1y, c2z);

	Vect D (c2x, c2y, c1z);
	Vect E (c1x, c2y, c1z);
	Vect F (c1x, c2y, c2z);

	//left side
	scene_objects.push_back(new Triangle(D, A, corner1, color, 2));
	scene_objects.push_back(new Triangle(corner1, E, D, color, 2));
	//far side
	scene_objects.push_back(new Triangle(corner2, B, A, color, 2));
	scene_objects.push_back(new Triangle(A, D, corner2, color, 2));
	//right side
	scene_objects.push_back(new Triangle(F, C, B, color, 2));
	scene_objects.push_back(new Triangle(B, corner2, F, color, 2));
	//front side
	scene_objects.push_back(new Triangle(E, corner1, C, color, 2));
	scene_objects.push_back(new Triangle(C, F, E, color, 2));
	//top
	scene_objects.push_back(new Triangle(D, E, F, color, 2));
	scene_objects.push_back(new Triangle(F, corner2, D, color, 2));
	//bottom
	scene_objects.push_back(new Triangle(corner1, A, B, color, 2));
	scene_objects.push_back(new Triangle(B, C, corner1, color, 2));
}

void makeCornellBox(Vect corner1, Vect corner2)
{
	double c1x = corner1.getVectX();
	double c1y = corner1.getVectY();
	double c1z = corner1.getVectZ();

	double c2x = corner2.getVectX();
	double c2y = corner2.getVectY();
	double c2z = corner2.getVectZ();

	Vect A (c2x, c1y, c1z);
	Vect B (c2x, c1y, c2z);
	Vect C (c1x, c1y, c2z);

	Vect D (c2x, c2y, c1z);
	Vect E (c1x, c2y, c1z);
	Vect F (c1x, c2y, c2z);


	Color red(1, 0.25, 0.25, 0);
	Color green(0.25, 1, 0.25, 0);
	Color white(1.0, 1.0, 1.0, 0);


	//left side
	scene_objects.push_back(new Triangle(D, A, corner1, green, 20));
	scene_objects.push_back(new Triangle(corner1, E, D, green, 20));
	//far side
	scene_objects.push_back(new Triangle(corner2, B, A, white, 20));
	scene_objects.push_back(new Triangle(A, D, corner2, white, 20));
	//right side
	scene_objects.push_back(new Triangle(F, C, B, red, 20));
	scene_objects.push_back(new Triangle(B, corner2, F, red, 20));
	//front side
	//scene_objects.push_back(new Triangle(E, corner1, C, white, 20));
	//scene_objects.push_back(new Triangle(C, F, E, white, 20));
	//top
	scene_objects.push_back(new Triangle(D, E, F, white, 20));
	scene_objects.push_back(new Triangle(F, corner2, D, white, 20));
	//bottom
	scene_objects.push_back(new Triangle(corner1, A, B, white, 20));
	scene_objects.push_back(new Triangle(B, C, corner1, white, 20));
}


int main(int argc, char *argv[])
{
	cout << "rendering..." << endl << "Preparing Scene..." << endl;

	clock_t t1, t2, tPrep, tPhoton, tTracing;
	t1 = clock();

	int dpi = 72;
	int width = 512;
	int height = 512;
	int n = width * height;

	int aadepth = 1;
	double aathreshold = .1;
	double aspectRatio = (double) width / (double) height;
	double ambientLight = 0;
	double accuracy = 0.000001;

	RGBType *pixels = new RGBType[n];

	Vect O (0, 0, 0);
	Vect X (1, 0, 0);
	Vect Y (0, 1, 0);
	Vect Z (0, 0, 1);

	Vect new_sphere_pos (0.3, -0.7, -0.4);
	Vect new_sphere_pos2 (0.3, -0.7, 0.4);

	Vect camPos(2.8, 0, 0);

	Vect look_at(0, 0, 0);
	Vect diff_btw(camPos.getVectX() - look_at.getVectX(), camPos.getVectY() - look_at.getVectY(), camPos.getVectZ() - look_at.getVectZ());

	Vect camDir = diff_btw.negtive().normalize();
	Vect camRight = Y.crossProduct(camDir).normalize();
	Vect camDown = camRight.crossProduct(camDir);

	Camera scene_cam (camPos, camDir, camRight, camDown);

	Color white_light (1.0, 1.0, 1.0, 0.0);
	//Color pretty_green (0.5,1.0,0.5,0.3);
#ifdef GLOSSY 
	Color reflectWhite (1.0, 1.0, 1.0, 0.987654);
	Color refractWhite(1.0, 1.0, 1.0, 1.887654);
#else
	Color reflectWhite (1.0, 1.0, 1.0, 0.9);
	Color refractWhite (1.0, 1.0, 1.0, 1.8);
#endif
	Color maroon(0.5, 0.25, 0.25, 2);
	Color pretty_maroon(0.5, 0.25, 0.25, 0.6);
	Color gray(0.5, 0.5, 0.5, 0);
	Color black(0.0, 0.0, 0.0, 0);
	Color orange(0.94, 0.75, 0.31, 0);

	Vect light_position(0.1, 0.2, 0);
	Light scene_light (light_position, white_light);
	vector<Source *> light_sources;
	light_sources.push_back(dynamic_cast<Source *>(&scene_light));

	Sphere scene_sphere (new_sphere_pos, 0.3, reflectWhite, 220);
	Sphere scene_sphere2 (new_sphere_pos2, 0.3, refractWhite, 1.5);
	//Plane scene_plane(Y, -1, maroon);
	Triangle scene_triangle (Vect(3, 0, 0), Vect(0, 3, 0), Vect(0, 0, 3), orange, 20);

	scene_objects.push_back(dynamic_cast<Object *>(&scene_sphere));
	scene_objects.push_back(dynamic_cast<Object *>(&scene_sphere2));

	makeCornellBox(Vect(1, 1, 1), Vect(-1, -1, -1));

	tPrep = clock();
	float diffPrep = ((float) tPrep - (float) t1) / CLOCKS_PER_SEC;
	cout << diffPrep << "seconds" << endl;
	cout << "start emit photons..." << endl;

	srand(0);

	std::uniform_real_distribution<> dis(-1, 1);
	//Color lightColor = white_light.colorScalar(32.0/PHOTONMUM);
	Color lightColor = white_light;

	int procNum = omp_get_num_procs();

#pragma omp parallel for
	for (int t = 0; t < procNum; t++) {
		int pn = 0;
		while (pn < PHOTONMUM/procNum) {
			double x, y, z;
			do {
				x = dis(gen);
				y = dis(gen);
				z = dis(gen);
			} while (x * x + y * y + z * z > 1);
	
			Vect photon_ray_direction(x, y, z);
			photon_ray_direction = photon_ray_direction.normalize();
	
			Ray photon_ray (light_position, photon_ray_direction);
	
			int bounce = 0;
			photonEmission(photon_ray, photon_ray_direction, scene_objects, accuracy, ambientLight, lightColor, bounce);
	
			pn++;
		}
	}

	//kdtree = new KDTree(photons);
	vector<KDTree*> KDTreeSet;

	for (int t = 0; t < procNum; t++) {
		KDTreeSet.push_back(new KDTree(photons));
	}

	tPhoton = clock();
	float diffPhoton = ((float) tPhoton - (float) tPrep) / CLOCKS_PER_SEC;
	cout << diffPhoton << "seconds" << endl;
	cout << "start ray tracing..." << endl;

	for (int x = 0; x < width; x++) {
#pragma omp parallel for
		for (int y = 0; y < height; y++) {
			int thisone = y * width + x;
			int aa_index;
			double xamnt, yamnt;  //dafuq?

			//start with black pix
			double *tempRed = new double [aadepth * aadepth];
			double *tempGreen = new double [aadepth * aadepth];
			double *tempBlue = new double [aadepth * aadepth];

			for (int aax = 0; aax < aadepth; aax++) {
				for (int aay = 0; aay < aadepth; aay++) {

					aa_index = aay * aadepth + +aax;

					if (aadepth == 1) {
						// no anti aliasing
						if (width > height) {
							// img is wider than tall
							xamnt = ((x + 0.5) / width) * aspectRatio - (((width - height) / (double) height) / 2);
							yamnt = ((height - y) + 0.5) / height;
						}
						else if (height > width) {
							xamnt = (x + 0.5) / width;
							yamnt = (((height - y) + 0.5) / height) / aspectRatio - (((height - width) / (double) width) / 2);
						}
						else {
							//square image
							xamnt = (x + 0.5) / width;
							yamnt = ((height - y) + 0.5) / height;
						}
					}

					else {
						// AA
						if (width > height) {
							// img is wider than tall
							xamnt = ((x + (double) aax / ((double) aadepth - 1)) / width) * aspectRatio - (((width - height) / (double) height) / 2);
							yamnt = ((height - y) + (double) aax / ((double) aadepth - 1)) / height;
						}
						else if (height > width) {
							xamnt = (x + (double) aax / ((double) aadepth - 1)) / width;
							yamnt = (((height - y) + (double) aax / ((double) aadepth - 1)) / height) / aspectRatio - (((height - width) / (double) width) / 2);
						}
						else {
							//square image
							xamnt = (x + (double) aax / ((double) aadepth - 1)) / width;
							yamnt = ((height - y) + (double) aax / ((double) aadepth - 1)) / height;
						}
					}

					Vect cam_ray_origin = scene_cam.getCameraPosition();
					Vect cam_ray_direction = camDir.vectAdd(camRight.vectMult(xamnt - 0.5).vectAdd(camDown.vectMult(yamnt - 0.5))).normalize();

					Ray cam_ray (cam_ray_origin, cam_ray_direction);

					vector<double> intersections;

					for (int index = 0; index < scene_objects.size(); index++) {
						intersections.push_back(scene_objects.at(index)->findIntersection(cam_ray));
					}

					int index_of_winning_object = winningObjectIndex(intersections);

					//cout << index_of_winning_object;

					if (index_of_winning_object == -1) {
						tempRed[aa_index] = 0;
						tempGreen[aa_index] = 0;
						tempBlue[aa_index] = 0;
					}
					else {
						if (intersections.at(index_of_winning_object) > accuracy) {
							Vect intersection_position = cam_ray.getRayOrigin().vectAdd(cam_ray_direction.vectMult(intersections.at(index_of_winning_object)));
							Vect intersecting_ray_direction = cam_ray_direction;

							Color intersection_color = getColorAt(intersection_position, intersecting_ray_direction, scene_objects, index_of_winning_object, accuracy, ambientLight, 1, KDTreeSet.at(omp_get_thread_num()));

							tempRed[aa_index] = intersection_color.getColorRed();
							tempGreen[aa_index] = intersection_color.getColorGreen();
							tempBlue[aa_index] = intersection_color.getColorBlue();
						}
					}
				}
				//delete tempRed, tempGreen, tempBlue;
			}
			// return color

			double totalRed = 0;
			double totalGreen = 0;
			double totalBlue = 0;

			for (int iRed = 0; iRed < aadepth * aadepth; iRed++) {
				totalRed += tempRed[iRed];
			}
			for (int iGreen = 0; iGreen < aadepth * aadepth; iGreen++) {
				totalGreen += tempGreen[iGreen];
			}
			for (int iBlue = 0; iBlue < aadepth * aadepth; iBlue++) {
				totalBlue += tempBlue[iBlue];
			}

			double avgRed = totalRed / (aadepth * aadepth);
			double avgGreen = totalGreen / (aadepth * aadepth);
			double avgBlue = totalBlue / (aadepth * aadepth);

			pixels[thisone].r = avgRed;
			pixels[thisone].g = avgGreen;
			pixels[thisone].b = avgBlue;
		}
	}

	tTracing = clock();
	float diffTracing = ((float) tTracing - (float) tPrep) / CLOCKS_PER_SEC;
	cout << diffTracing << "seconds" << endl << "saving file" << endl;

	saveBmp("scene.bmp", width, height, dpi, pixels);
	cout << "Finished" << endl;

	//delete pixels, tempRed, tempGreen, tempBlue;;

	t2 = clock();
	float diff = ((float) t2 - (float) t1) / CLOCKS_PER_SEC;

	//delete kdtree;
	cout << diff << "seconds" << endl;

	system("pause");
	return 0;
}