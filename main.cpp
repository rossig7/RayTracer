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

#define PHOTONMUM 100000
#define PHOTONUSE 200
#define PHOTONDIST 0.2

#define PI 3.1415926
#define BOUNCE 3

using namespace std;

Photon photonMap[PHOTONMUM* BOUNCE* BOUNCE* BOUNCE* BOUNCE];
int storedPhotonMum = 0;

const int max_elements = 3;

struct KDNode {
	// 0:x ; 1:y ; 2:z
	short axis;
	Photon *self;
	vector<Photon *> left, right;
	KDNode *left_branch, *right_branch;

	KDNode()
	{
		left_branch = right_branch = NULL;
		axis = -1;
	}
};

void createKD(vector<Photon *> &photons, KDNode *parent)
{
	assert(photons.size() > max_elements);
	switch (parent->axis) {
		case 0:
			sort(photons.begin(), photons.end(),
					[](const Photon *a, const Photon *b) -> bool
							{
								return a->position.getVectX() < b->position.getVectX();
							});
	        break;
		case 1:
			sort(photons.begin(), photons.end(),
					[](const Photon *a, const Photon *b) -> bool
							{
								return a->position.getVectY() < b->position.getVectY();
							});
	        break;
		case 2:
			sort(photons.begin(), photons.end(),
					[](const Photon *a, const Photon *b) -> bool
							{
								return a->position.getVectZ() < b->position.getVectZ();
							});
	        break;
	}

	int mid = photons.size() / 2;
	parent->self = photons[mid];
	/*cout << "Axis: " << parent->axis << " Pos:" << parent->self->position.getVectX()
			<< " " << parent->self->position.getVectY()
			<< " " << parent->self->position.getVectZ()
			<< " Mid: " << mid << endl ;  */
	vector<Photon *> left(photons.begin(), photons.begin() + mid),
			right(photons.begin() + mid + 1, photons.end());
	if (left.size() <= max_elements) {
		parent->left = left;
	}
	else {
		parent->left_branch = new KDNode();
		parent->left_branch->axis = (parent->axis + 1) % 3;
		createKD(left, parent->left_branch);
	}
	if (right.size() <= max_elements) {
		parent->right = right;
	}
	else {
		parent->right_branch = new KDNode();
		parent->right_branch->axis = (parent->axis + 1) % 3;
		createKD(right, parent->right_branch);
	}

	return;
}

KDNode *Root;
class distanceComparsion {
	Vect center;
public:
	distanceComparsion(Vect center_)
	{
		center = center_;
	}

	bool operator ()(const Photon *lhs, const Photon *rhs) const
	{
		return lhs->position.sqrDist(center) < rhs->position.sqrDist(center);
	}
};

priority_queue<Photon *, vector<Photon *>, distanceComparsion> *KNN_queue;

int perf_count = 0;
void findKNN_(int k, Vect center, KDNode *root)
{
	/*
	Photon * topPhoton = KNN_queue->top();
	float farDistance = topPhoton->position.sqrDist(center);
    */

	for (int i = 0; i < root->left.size(); i++) {
		perf_count++;
		if (KNN_queue->size() < k &&
				root->left[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue->push(root->left[i]);
			continue;
		}
		float dis = 1e3;
		if(KNN_queue->size() > 0)
		{
			Photon *topPhoton = KNN_queue->top();
			dis = topPhoton->position.sqrDist(center);
		}
		if (root->left[i]->position.sqrDist(center) < dis
				&& root->left[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue->push(root->left[i]);
			KNN_queue->pop();
		}
	}

	for (int i = 0; i < root->right.size(); i++) {
		perf_count++;
		if (KNN_queue->size() < k &&
				root->right[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue->push(root->right[i]);
			continue;
		}
		float dis = 1e3;
		if(KNN_queue->size() > 0)
		{
			Photon *topPhoton = KNN_queue->top();
			dis = topPhoton->position.sqrDist(center);
		}
		if (root->right[i]->position.sqrDist(center) < dis
			&& root->left[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue->push(root->right[i]);
			KNN_queue->pop();
		}
	}

	// Pivot Self
	perf_count++;
	if (KNN_queue->size() < k &&
			root->self->position.sqrDist(center) < PHOTONDIST) {
		KNN_queue->push(root->self);
	}
	else {
		float dis = 1e3;
		if(KNN_queue->size() > 0)
		{
			Photon *topPhoton = KNN_queue->top();
			dis = topPhoton->position.sqrDist(center);
		}
		if (root->self->position.sqrDist(center) < dis &&
				root->self->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue->push(root->self);
			KNN_queue->pop();
		}
	}

	// Order Left First
	int is_left_first = 0;
	int need_other = 1;
	switch (root->axis) {
		case 0:
			if (center.getVectX() < root->self->position.getVectX())
				is_left_first = 1;
	        break;
		case 1:
			if (center.getVectY() < root->self->position.getVectY())
				is_left_first = 1;
	        break;
		case 2:
			if (center.getVectZ() < root->self->position.getVectZ())
				is_left_first = 1;
	        break;
	}

	if (is_left_first) {
		if (root->left_branch) {
			findKNN_(k, center, root->left_branch);
		}
		float dis = 1e3;
		if(KNN_queue->size() > 0)
		{
			Photon *topPhoton = KNN_queue->top();
			dis = topPhoton->position.sqrDist(center);
		}
		float farDistance = min(PHOTONDIST * PHOTONDIST * 1.,dis * 1.);
		switch (root->axis) {
			case 0:
				if (abs(center.getVectX() - root->self->position.getVectX()) > sqrt(farDistance))
					need_other = 0;
		        break;
			case 1:
				if (abs(center.getVectY() - root->self->position.getVectY()) > sqrt(farDistance))
					need_other = 0;
		        break;
			case 2:
				if (abs(center.getVectZ() - root->self->position.getVectZ()) > sqrt(farDistance))
					need_other = 0;
		        break;
		}
		if (need_other && root->right_branch) {
			findKNN_(k, center, root->right_branch);
		}
	}
	else {
		if (root->right_branch) {
			findKNN_(k, center, root->right_branch);
		}

		float dis = 1e3;
		if(KNN_queue->size() > 0)
		{
			Photon *topPhoton = KNN_queue->top();
			dis = topPhoton->position.sqrDist(center);
		}
		float farDistance = min(PHOTONDIST * PHOTONDIST * 1.,dis * 1.);
		switch (root->axis) {
			case 0:
				if (abs(center.getVectX() - root->self->position.getVectX()) > sqrt(farDistance))
					need_other = 0;
		        break;
			case 1:
				if (abs(center.getVectY() - root->self->position.getVectY()) > sqrt(farDistance))
					need_other = 0;
		        break;
			case 2:
				if (abs(center.getVectZ() - root->self->position.getVectZ()) > sqrt(farDistance))
					need_other = 0;
		        break;
		}

		if (need_other && root->left_branch) {
			findKNN_(k, center, root->left_branch);
		}

	}
}


vector<Photon *> findKNN(int k, Vect center, KDNode *root)
{
	KNN_queue = new priority_queue<Photon *, vector<Photon *>, distanceComparsion>
			(distanceComparsion(center));

	findKNN_(k, center, root);

	vector<Photon *> result;
	while (KNN_queue->size() > 0) {
		result.push_back(KNN_queue->top());
		KNN_queue->pop();
	}
	reverse(result.begin(), result.end());
	delete KNN_queue;
	return result;
}


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

Color storePhoton(Vect intersection_position, Vect intersecting_ray_direction, vector<Object *> scene_objects, int index_of_winning_object, vector<Source *> light_sources, double accuracy, double ambientLight, Color lightColor, int bounce)
{
	Color winning_object_color = scene_objects.at(index_of_winning_object)->getColor();

	Vect winning_object_normal = scene_objects.at(index_of_winning_object)->getNormalAt(intersection_position);

	//Color final_color = winning_object_color/*.colorScalar(ambientLight)*/;

	Color final_color;
	final_color.setColorRed(min(winning_object_color.getColorRed(), lightColor.getColorRed()));
	final_color.setColorGreen(min(winning_object_color.getColorGreen(), lightColor.getColorGreen()));
	final_color.setColorBlue(min(winning_object_color.getColorBlue(), lightColor.getColorBlue()));



	/*
	for (int light_index = 0; light_index < light_sources.size(); light_index++){
		Vect light_direction = light_sources.at(light_index)->getLightPosition().vectAdd(intersection_position.negtive()).normalize();
				
		float cosine_angle = winning_object_normal.dotProduct(light_direction);

		if (cosine_angle > 0){
			//test shadow
			bool shadowed = false;

			Vect distance_to_light = light_sources.at(light_index)->getLightPosition().vectAdd(intersection_position.negtive()); // no normalize
			float distance_to_light_magnitude = distance_to_light.magnitude();

			Ray shadow_ray(intersection_position, light_sources.at(light_index)->getLightPosition().vectAdd(intersection_position.negtive()).normalize());

			vector<double> secondary_intersections;
			for (int object_index = 0; object_index < scene_objects.size() && shadowed == false; object_index++){
				secondary_intersections.push_back(scene_objects.at(object_index)->findIntersection(shadow_ray));
			}

			for (int c = 0; c < secondary_intersections.size(); c++){
				if (secondary_intersections.at(c) > accuracy){
					if (secondary_intersections.at(c) <= distance_to_light_magnitude){
						shadowed = true;
					}
					break;
				}
			}

			if (shadowed == false){
				final_color = final_color.colorAdd(winning_object_color.colorMultiply(light_sources.at(light_index)->getLightColor()).colorScalar(cosine_angle));

				if (winning_object_color.getColorSpecial() > 0 && winning_object_color.getColorSpecial() <= 1){
					double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive()); // cos of objNormal and rayDir
					Vect scalar1 = winning_object_normal.vectMult(dot1); // 
					Vect add1 = scalar1.vectAdd(intersecting_ray_direction);
					Vect scalar2 = add1.vectMult(2);
					Vect add2 = intersecting_ray_direction.negtive().vectAdd(scalar2);
					Vect reflection_dir = add2.normalize();

					double specular = reflection_dir.dotProduct(light_direction);
					if (specular > 0){
						specular = pow(specular, 10);
						final_color = final_color.colorAdd(light_sources.at(light_index)->getLightColor().colorScalar(specular*winning_object_color.getColorSpecial()));
					}
 				}
			}
		}
	}
	*/

	char phi = 255 * (atan2(intersecting_ray_direction.getVectX(), intersecting_ray_direction.getVectY()) + PI) / (2 * PI);
	char theta = 255 * acos(intersecting_ray_direction.getVectX()) / PI;

	//photonMap[storedPhotonMum++] = Photon(intersection_position, final_color.clip(), phi, theta, intersecting_ray_direction);
	photonMap[storedPhotonMum++] = Photon(intersection_position, lightColor, phi, theta, intersecting_ray_direction);

	photonMap[storedPhotonMum].bounce = bounce;
	return final_color.clip();
}

Color getColorAt(Vect intersection_position, Vect intersecting_ray_direction, vector<Object *> scene_objects, int index_of_winning_object, vector<Source *> light_sources, double accuracy, double ambientLight)
{
	Color winning_object_color = scene_objects.at(index_of_winning_object)->getColor();
	Vect winning_object_normal = scene_objects.at(index_of_winning_object)->getNormalAt(intersection_position);

	Color final_color = winning_object_color.colorScalar(ambientLight);

	if (winning_object_color.getColorSpecial() > 0 && winning_object_color.getColorSpecial() <= 1) {
		//reflection from object with specular intensity

		double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
		Vect scalar1 = winning_object_normal.vectMult(dot1);
		Vect add1 = scalar1.vectAdd(intersecting_ray_direction);
		Vect scalar2 = add1.vectMult(2);
		Vect add2 = intersecting_ray_direction.negtive().vectAdd(scalar2);
		Vect reflection_direction = add2.normalize();

		Ray reflection_ray(intersection_position, reflection_direction);

		//determine what the ray intersects first
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

				Color reflection_intersection_color = getColorAt(reflection_intersection_position, reflection_intersection_direction, scene_objects, index_of_winning_object_reflection, light_sources, accuracy, ambientLight);

				final_color = final_color.colorAdd(reflection_intersection_color.colorScalar(winning_object_color.getColorSpecial()));
			}
		}
	}

	else if (winning_object_color.getColorSpecial() > 1) {
		double n1n2 = 1.0 / 1.5;
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

			//Ray reflection_ray(intersection_position, reflection_direction);
			Ray reflection_ray(intersection_position.vectAdd(intersecting_ray_direction.vectMult(0)), reflection_direction);

			//determine what the ray intersects first
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

					Color reflection_intersection_color = getColorAt(reflection_intersection_position, reflection_intersection_direction, scene_objects, index_of_winning_object_reflection, light_sources, accuracy, ambientLight);

					final_color = final_color.colorAdd(reflection_intersection_color.colorScalar(/*winning_object_color.getColorSpecial()*/1));
				}
			}
		}
	}

	else {

		vector<Photon *> photon_find = findKNN(PHOTONUSE, intersection_position, Root);


		for(int i = 0; i < photon_find.size(); i++)
		{
			Vect light_direction = photon_find[i]->dir.negtive();

			float cosine_angle = winning_object_normal.dotProduct(light_direction);

			if (cosine_angle > 0) {
				//test shadow
				bool shadowed = false;

				final_color = final_color.colorAdd(winning_object_color.colorMultiply(photon_find[i]->power).colorScalar(cosine_angle));
			}

		}
		if (photon_find.size() != 0) {
			final_color = final_color.colorScalar(1.0 / (photon_find.size()));
			//final_color = final_color.colorScalar(1.0/(4*PI));
		}
	}
	return final_color.clip();
}

int thisone;
vector<Object *> scene_objects;

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
	scene_objects.push_back(new Triangle(D, A, corner1, color));
	scene_objects.push_back(new Triangle(corner1, E, D, color));
	//far side
	scene_objects.push_back(new Triangle(corner2, B, A, color));
	scene_objects.push_back(new Triangle(A, D, corner2, color));
	//right side
	scene_objects.push_back(new Triangle(F, C, B, color));
	scene_objects.push_back(new Triangle(B, corner2, F, color));
	//front side
	scene_objects.push_back(new Triangle(E, corner1, C, color));
	scene_objects.push_back(new Triangle(C, F, E, color));
	//top
	scene_objects.push_back(new Triangle(D, E, F, color));
	scene_objects.push_back(new Triangle(F, corner2, D, color));
	//bottom
	scene_objects.push_back(new Triangle(corner1, A, B, color));
	scene_objects.push_back(new Triangle(B, C, corner1, color));
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
	scene_objects.push_back(new Triangle(D, A, corner1, green));
	scene_objects.push_back(new Triangle(corner1, E, D, green));
	//far side
	scene_objects.push_back(new Triangle(corner2, B, A, white));
	scene_objects.push_back(new Triangle(A, D, corner2, white));
	//right side
	scene_objects.push_back(new Triangle(F, C, B, red));
	scene_objects.push_back(new Triangle(B, corner2, F, red));
	//front side
	//scene_objects.push_back(new Triangle(E, corner1, C, white));
	//scene_objects.push_back(new Triangle(C, F, E, white));
	//top
	scene_objects.push_back(new Triangle(D, E, F, white));
	scene_objects.push_back(new Triangle(F, corner2, D, white));
	//bottom
	scene_objects.push_back(new Triangle(corner1, A, B, white));
	scene_objects.push_back(new Triangle(B, C, corner1, white));
}


int main(int argc, char *argv[])
{
	cout << "rendering..." << endl << "Preparing Scene..." << endl;

	clock_t t1, t2, tPrep, tTracing;
	t1 = clock();

	int dpi = 72;
	int width = 480;
	int height = 480;
	int n = width * height;

	int aadepth = 1;
	double aathreshold = .1;
	double aspectRatio = (double) width / (double) height;
	double ambientLight = 0.2;
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
	Color pretty_green (1.0, 1.0, 1.0, 0.9);
	Color maroon(0.5, 0.25, 0.25, 2);
	Color pretty_maroon(0.5, 0.25, 0.25, 0.6);
	Color gray(0.5, 0.5, 0.5, 0);
	Color black(0.0, 0.0, 0.0, 0);
	Color orange(0.94, 0.75, 0.31, 0);
	Color refractWhite(1.0, 1.0, 1.0, 1.9);

	Vect light_position(0, 0.9, 0);
	Light scene_light (light_position, white_light);
	vector<Source *> light_sources;
	light_sources.push_back(dynamic_cast<Source *>(&scene_light));

	Sphere scene_sphere (new_sphere_pos, 0.3, pretty_green);
	Sphere scene_sphere2 (new_sphere_pos2, 0.3, orange);
	Plane scene_plane(Y, -1, maroon);
	Triangle scene_triangle (Vect(3, 0, 0), Vect(0, 3, 0), Vect(0, 0, 3), orange);

	scene_objects.push_back(dynamic_cast<Object *>(&scene_sphere));
	//scene_objects.push_back(dynamic_cast<Object*>(&scene_plane));
	scene_objects.push_back(dynamic_cast<Object *>(&scene_sphere2));
	//scene_objects.push_back(dynamic_cast<Object*>(&scene_triangle));

	makeCornellBox(Vect(1, 1, 1), Vect(-1, -1, -1));
	//makeCube(Vect (0.5,0,0.5), Vect (-0.5,-1,-0.5), pretty_green);

	int thisone, aa_index;
	double xamnt, yamnt;  //dafuq?
	double tempRed, tempGreen, tempBlue;

	tPrep = clock();
	float diffPrep = ((float) tPrep - (float) t1) / CLOCKS_PER_SEC;
	cout << diffPrep << "seconds" << endl;

	cout << "start emit photons..." << endl;


	int pn = 0;
	float x, y, z;

	srand(0);

	while (pn < PHOTONMUM) {
		do {
			x = (rand() % 200) / 100.0f - 1;
			y = (rand() % 200) / 100.0f - 1;
			z = (rand() % 200) / 100.0f - 1;
		} while (x * x + y * y + z * z > 1);

		Vect photon_ray_direction(x, y, z);
		photon_ray_direction = photon_ray_direction.normalize();

		Ray photon_ray (light_position, photon_ray_direction);

		Color lightColor = white_light;

		for (int bounce = 0; bounce < BOUNCE; bounce++) {
			vector<double> intersections;

			for (int index = 0; index < scene_objects.size(); index++) {
				intersections.push_back(scene_objects.at(index)->findIntersection(photon_ray));
			}

			int index_of_winning_object = winningObjectIndex(intersections);

			//cout << index_of_winning_object;
			if (index_of_winning_object == -1) {
				if (bounce == 0) pn--;
				break;
			}

			if (intersections.at(index_of_winning_object) > accuracy) {
				Vect intersection_position = photon_ray.getRayOrigin().vectAdd(photon_ray_direction.vectMult(intersections.at(index_of_winning_object)));
				Vect intersecting_ray_direction = photon_ray_direction;

				if (scene_objects.at(index_of_winning_object)->getColor().getColorSpecial() == 0) {
					lightColor = storePhoton(intersection_position, intersecting_ray_direction, scene_objects, index_of_winning_object, light_sources, accuracy, ambientLight, lightColor, bounce);
				}
				//lightColor = lightColor.colorScalar(0.8);

				//srand(intersection_position.getVectX()*100);
				//int roll = rand() % 100;
				//if (roll < 40) {
				//	break;
				//}
				//else {
				if (scene_objects.at(index_of_winning_object)->getColor().getColorSpecial() < 1) {
					lightColor = lightColor.colorScalar(0.7);
					Vect winning_object_normal = scene_objects.at(index_of_winning_object)->getNormalAt(intersection_position);
					double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive()); // N*L
					Vect scalar1 = winning_object_normal.vectMult(dot1); // (N*L)*N
					Vect add1 = scalar1.vectAdd(intersecting_ray_direction);
					Vect scalar2 = add1.vectMult(2);
					Vect add2 = intersecting_ray_direction.negtive().vectAdd(scalar2);
					Vect reflection_dir = add2.normalize();


					photon_ray = Ray(intersection_position, reflection_dir);
					photon_ray_direction = reflection_dir;
				}
				else {
					lightColor = lightColor.colorScalar(1);
					Vect winning_object_normal = scene_objects.at(index_of_winning_object)->getNormalAt(intersection_position);
					double n1n2 = 1.0 / 1.5;
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
						Vect refraction_direction = winning_object_normal.vectMult(coeffN).vectAdd((intersecting_ray_direction.negtive().vectMult(n1n2)).negtive());

						//Ray reflection_ray(intersection_position, reflection_direction);
						Ray refraction_ray(intersection_position, refraction_direction);

						photon_ray = refraction_ray;
						photon_ray_direction = refraction_direction;
						bounce--;
						lightColor = white_light;
					}
				}
				//srand(0);
				//}		
			}
		}
		pn++;
	}

	vector<Photon *> photons;
	Root = new KDNode();
	Root->axis = 0;
	for (int i = 0; i < storedPhotonMum; i++)
		photons.push_back(photonMap + i);
	createKD(photons, Root);


	const Vect Goal(1,0.5,0.4);
	perf_count = 0;
	vector<Photon *> testfind = findKNN(20, Goal, Root);

	/*
	for (int i = 0; i < testfind.size(); i++) {
		cout << testfind[i]->position.getVectX() << " "
				<< testfind[i]->position.getVectY() << " "
				<< testfind[i]->position.getVectZ() << " " << endl;
	}
    */

	/*
	vector<Photon *> copy_test = photons;
	sort(copy_test.begin(), copy_test.end(),
			[Goal](const Photon *a, const Photon *b) -> bool
					{
						return a->position.sqrDist(Goal) <
								b->position.sqrDist(Goal);
					});

	cout << "Total..." << perf_count << endl;
	for (int i = 0; i < testfind.size(); i++) {
		//cout << testfind[i]->position.sqrDist(copy_test[i]->position) << endl;
		if(testfind[i]->position.sqrDist(Goal) -
				copy_test[i]->position.sqrDist(Goal) > 1e-7)
		{
			cout << testfind[i]->position.getVectX() << " "
				<< testfind[i]->position.getVectY() << " "
					<< testfind[i]->position.getVectZ() << " "
					<< endl;
			cout << copy_test[i]->position.getVectX() << " "
					<< copy_test[i]->position.getVectY() << " "
					<< copy_test[i]->position.getVectZ() << " "
					<< endl;
			cout << endl;
		}
	}
	*/
	cout << "start ray tracing..." << endl;

	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			thisone = y * width + x;

			//start with black pix
			double *tempRed = new double [aadepth * aadepth];
			double *tempGreen = new double [aadepth * aadepth];
			double *tempBlue = new double [aadepth * aadepth];

			for (int aax = 0; aax < aadepth; aax++) {
				for (int aay = 0; aay < aadepth; aay++) {

					aa_index = aay * aadepth + +aax;

					srand(time(0));

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

							Color intersection_color = getColorAt(intersection_position, intersecting_ray_direction, scene_objects, index_of_winning_object, light_sources, accuracy, ambientLight);

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

	delete pixels, tempRed, tempGreen, tempBlue;;

	t2 = clock();
	float diff = ((float) t2 - (float) t1) / CLOCKS_PER_SEC;

	cout << diff << "seconds" << endl;

	system("pause");
	return 0;
}