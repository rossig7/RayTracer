#include "System.h"

#include "IO.h"
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
#include "BVH.h"

std::mutex photon_mtx;

/*NOTICE: a good value pair would be (50000,1200), too large PHOTONMUM/PHOTONUSE will produce spotty result, too small will produce over-blurred shadow*/
#define PHOTONMUM 50000
#define PHOTONUSE 1000

#define PI 3.1415926
#define BOUNCE 5
#define NATUREE 2.71828
#define FRESNEL
#define TRACING_DEPTH 4  // Depth must >= 4

/*define GLOSSY will enable glossy reflection and refraction for both balls in the scene. VERY SLOW*/
//#define GLOSSY
#define GLOSSY_SAMPLE 16

/*set DOF_SAMPLE to 1 to disable depth of field. To enable set a value larger than 128 is recommended*/
#define DOF_SAMPLE 1
#define FOCAL_LEN 2.6
#define APERTURE_SIZE 0.2

#define LOADOBJ

BVH* bvh;
KDTree* kdtree;
KDTree* volumeKDTree;
vector<Photon *> photons;
vector<Photon *> volumePhotons;
vector<Object *> scene_objects;

std::uniform_real_distribution<> dis_rand(0, 1);
std::uniform_real_distribution<> dis_rand_100(0, 100);
std::uniform_real_distribution<> dis_rand_dof(-APERTURE_SIZE, APERTURE_SIZE);

void CalcTime(string message)
{
    using namespace std::chrono;

    static high_resolution_clock::time_point t[20];
    static int current_ptr = 0;
    t[current_ptr] = high_resolution_clock::now();
    if(current_ptr > 0)
    {
       cout << "Elapsed: " << (duration_cast<duration<double>>(t[current_ptr] - t[current_ptr-1])).count();
       cout << "  Total: " << (duration_cast<duration<double>>(t[current_ptr] - t[0])).count() << endl;
    }
    cout << message << endl;
    current_ptr++;

}

Color storePhoton(Vect intersection_position, Vect intersecting_ray_direction, Object * intersect_obj, double accuracy, double ambientLight, Color lightColor, int bounce)
{
	Color winning_object_color = intersect_obj->getColor(intersection_position);

	Vect winning_object_normal = intersect_obj->getNormalAt(intersection_position);

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
	{
        photon_mtx.lock();
		photons.push_back(currentPhoton);
        photon_mtx.unlock();
	}
	return final_color.clip();
}

void storeVolumePhoton(Vect pos, Vect dir, Color lightColor)
{
	char phi = 0;
	char theta = 0;

	Photon* currentPhoton  = new Photon(pos, lightColor, phi, theta, dir);
	{
		photon_mtx.lock();
		volumePhotons.push_back(currentPhoton);
		photon_mtx.unlock();
	}
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

Vect computeReflectionDir(Vect normal, Vect ray_in) {
    double dot1 = normal.dotProduct(ray_in.negtive());
    if (dot1 <= 0.02) {
        return ray_in;
    }
    Vect scalar1 = normal.vectMult(dot1);
    Vect add1 = scalar1.vectAdd(ray_in);
    Vect scalar2 = add1.vectMult(2);
    Vect add2 = ray_in.negtive().vectAdd(scalar2);

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

Color getColorAt(Vect hit_pos, Vect hit_dir, Object *hit_obj, double accuracy, double ambientLight, int depth)
{
	Color final_color(0,0,0,0);
	if (depth > TRACING_DEPTH) return final_color.clip();
	hit_pos = hit_pos.vectAdd(hit_dir.vectMult(accuracy));

	Color hit_color = hit_obj->getColor(hit_pos);
	Vect hit_normal = hit_obj->getNormalAt(hit_pos);

	double colorSpecial = hit_color.getColorSpecial();

	if (colorSpecial > 0) {  // have reflection
		Vect reflection_direction = computeReflectionDir(hit_normal, hit_dir);

		double sample;
		double sampledGlossyReflectColor = 0;
		Color sampleGlossyReflectColor(0,0,0,0);
		double colorSpecial = hit_color.getColorSpecial();

		if (colorSpecial*100.0 - floor(colorSpecial*100.0) - 0.7654 < 0.0001 && colorSpecial*100.0 - floor(colorSpecial*100.0) - 0.7653 > 0) 
			sample = GLOSSY_SAMPLE;
		else sample = 1;

		for (int i = 0; i < sample; i ++) {
			Vect world_ref_ray_dir;
			if (sample != 1)
				world_ref_ray_dir = computeGlossyRay(reflection_direction, hit_normal, 0.2);
			else
				world_ref_ray_dir = reflection_direction;

			Ray reflection_ray(hit_pos, world_ref_ray_dir);

            double intersect_distance = 1e9;
            Object * intersect_obj = bvh->Shoot(reflection_ray, intersect_distance);

			if (hit_obj != nullptr && intersect_distance > accuracy) {
				//no miss
					Vect reflection_intersection_position = hit_pos.vectAdd(reflection_direction.vectMult(intersect_distance));
					Vect reflection_intersection_direction = reflection_direction;

					Color reflection_intersection_color = getColorAt(reflection_intersection_position, reflection_intersection_direction, hit_obj, accuracy, ambientLight, depth+1);

					if (colorSpecial > 1) 
						sampleGlossyReflectColor = sampleGlossyReflectColor.colorAdd(reflection_intersection_color.colorScalar(2 - hit_color.getColorSpecial()));
					else
						sampleGlossyReflectColor = sampleGlossyReflectColor.colorAdd(reflection_intersection_color);
					sampledGlossyReflectColor++;
			}
		}
		sampleGlossyReflectColor = sampleGlossyReflectColor.colorScalar(1.0/sample);
		final_color = final_color.colorAdd(sampleGlossyReflectColor);

		if (colorSpecial > 1) { // have reflection and refraction
			double sampledGlossyRefractColor = 0;
			Color sampleGlossyRefractColor(0,0,0,0);

			for (int i = 0; i < sample; i ++) {
				Vect world_ref_ray_dir;

				double refractance = hit_color.getColorSpecial() - 1;

				double n1n2 = 1.0 / hit_obj->getRefraIdx();
				double dot1 = hit_normal.dotProduct(hit_dir.negtive());  // NL

				if (dot1 < 0) {
					hit_normal = hit_normal.negtive();
					dot1 = hit_normal.dotProduct(hit_dir.negtive());
					n1n2 = 1.0 / n1n2;
				}

				double nNL = n1n2 * dot1;  //n*NL
				double underSQRT = 1 - n1n2 * n1n2 * (1 - dot1 * dot1); // 1-n^2*(1-(NL)^2)

				if (underSQRT > 0) {
					double coeffN = nNL - sqrt(underSQRT);   //n*NL - sqrt(1-n^2*(1-(NL)^2))
					Vect reflection_direction = hit_normal.vectMult(coeffN).vectAdd((hit_dir.negtive().vectMult(n1n2)).negtive());

					if (sample != 1)
						world_ref_ray_dir = computeGlossyRay(reflection_direction, hit_normal, 0.2);
					else
						world_ref_ray_dir = reflection_direction;

					Ray reflection_ray(hit_pos, world_ref_ray_dir);

                    double intersect_distance = 1e9;
                    Object * intersect_obj = bvh->Shoot(reflection_ray, intersect_distance);

                    if (hit_obj != nullptr && intersect_distance > accuracy) {
							Vect reflection_intersection_position = hit_pos.vectAdd(reflection_direction.vectMult(accuracy + intersect_distance));
							Vect reflection_intersection_direction = reflection_direction;

							Color reflection_intersection_color = getColorAt(reflection_intersection_position, reflection_intersection_direction, hit_obj, accuracy, ambientLight, depth+1);

							sampleGlossyRefractColor = sampleGlossyRefractColor.colorAdd(reflection_intersection_color.colorScalar(refractance));
							sampledGlossyRefractColor++;
					}
				}
				sampleGlossyRefractColor = sampleGlossyRefractColor.colorScalar(1.0/sample);
				final_color = final_color.colorAdd(sampleGlossyRefractColor);
			}
		}
	}
	else if (colorSpecial <= 0) {
		vector<Photon *> photon_find = kdtree->findKNN(PHOTONUSE, hit_pos);
		float maxDistSqr = -1;

		Color sssColor(0,0,0,0);

        if (photon_find.size() != 0)
            maxDistSqr = photon_find[photon_find.size() - 1]->position.sqrDist(hit_pos);

		//cout<<photon_find.size()<<" "<< maxDistSqr <<endl;

		for(int i = 0; i < photon_find.size(); i++)
		{
			float distanceSqr = photon_find[i]->position.sqrDist(hit_pos);

//#define CONE
#ifdef CONE  // cone filter
			float weight = 1 - sqrt(distanceSqr) / (k * sqrt(maxDistSqr));  // cone filter
#else  // gaussian filter
			float weight = 0.918 * (1 - (1 - pow(NATUREE, -1.953*(distanceSqr)/(2*maxDistSqr)))/(1 - pow(NATUREE, -1.953)));
#endif
			Vect light_direction = photon_find[i]->dir.negtive();

			float cosine_angle = hit_normal.dotProduct(light_direction);
			if (cosine_angle < 0) /*cosine_angle = 0;*/  cosine_angle = -cosine_angle;

			final_color = final_color.colorAdd(hit_color.colorMultiply(photon_find[i]->power).colorScalar(cosine_angle * weight));
		}
		if (photon_find.size() != 0) {
			//final_color = final_color.colorScalar(1.0 / (photon_find.size()));
#ifdef CONE
			final_color = final_color.colorScalar(1.0/(PHOTONMUM/16*PI*maxDistSqr * (1 - 2/(3*k))));
#else
			final_color = final_color.colorScalar(1.0/(PHOTONMUM/16*PI*maxDistSqr));
#endif
		}

		if (colorSpecial < 0) {   //sss
			double n1n2 = 1.0 / hit_obj->getRefraIdx();
			double dot1 = hit_normal.dotProduct(hit_dir.negtive());  // NL

			if (dot1 < 0) {
				hit_normal = hit_normal.negtive();
				dot1 = hit_normal.dotProduct(hit_dir.negtive());
				n1n2 = 1.0 / n1n2;
			}

			double nNL = n1n2 * dot1;  //n*NL
			double underSQRT = 1 - n1n2 * n1n2 * (1 - dot1 * dot1); // 1-n^2*(1-(NL)^2)

			vector<Photon *> volume_photon_find = volumeKDTree->findKNN(PHOTONUSE, hit_pos);
			float maxDistSqr = -1;
			Color firstStep(0,0,0,0);

			//cout << volume_photon_find.size() << endl;

            if (volume_photon_find.size() != 0)
                maxDistSqr = volume_photon_find[volume_photon_find.size() - 1]->position.sqrDist(hit_pos);

			for(int i = 0; i < volume_photon_find.size(); i++)
			{
				float distanceSqr = volume_photon_find[i]->position.sqrDist(hit_pos);

				float weight = 0.918 * (1 - (1 - pow(NATUREE, -1.953*(distanceSqr)/(2*maxDistSqr)))/(1 - pow(NATUREE, -1.953)));
				if (weight < 0) weight = 0;

				Vect light_direction = volume_photon_find[i]->dir.negtive();

				//currentStep = currentStep.colorAdd(hit_color.colorMultiply(photon_find[i]->power));
				firstStep = firstStep.colorAdd(hit_color.colorMultiply(volume_photon_find[i]->power).colorScalar(/*cosine_angle*/0.2));
			}
			if (volume_photon_find.size() != 0) {
				firstStep = firstStep.colorScalar(1.0/(pow(NATUREE, 0)*PHOTONMUM/8*PI*maxDistSqr*sqrt(maxDistSqr)));
			}
			sssColor = sssColor.colorAdd(firstStep);


			if (underSQRT > 0) {
				double coeffN = nNL - sqrt(underSQRT);   //n*NL - sqrt(1-n^2*(1-(NL)^2))
				Vect reflection_direction = hit_normal.vectMult(coeffN).vectAdd((hit_dir.negtive().vectMult(n1n2)).negtive());

				Vect pos = hit_pos.vectAdd(reflection_direction.vectMult(0.02));

				double pass;

				for (pass = 1; pass < 3; pass++) {
					vector<Photon *> volume_photon_find = volumeKDTree->findKNN(PHOTONUSE, pos);
					float maxDistSqr = -1;
					Color currentStep(0,0,0,0);

					//cout << volume_photon_find.size() << endl;

                    if (volume_photon_find.size() != 0)
                        maxDistSqr = volume_photon_find[volume_photon_find.size() - 1]->position.sqrDist(hit_pos);

					for(int i = 0; i < volume_photon_find.size(); i++)
					{
						float distanceSqr = volume_photon_find[i]->position.sqrDist(pos);

						float weight = 0.918 * (1 - (1 - pow(NATUREE, -1.953*(distanceSqr)/(2*maxDistSqr)))/(1 - pow(NATUREE, -1.953)));
						if (weight < 0) weight = 0;

						Vect light_direction = volume_photon_find[i]->dir.negtive();

						float cosine_angle = reflection_direction.dotProduct(light_direction);
						if (cosine_angle < 0) cosine_angle = 0;

						//currentStep = currentStep.colorAdd(hit_color.colorMultiply(photon_find[i]->power));
						currentStep = currentStep.colorAdd(hit_color.colorMultiply(volume_photon_find[i]->power).colorScalar(/*cosine_angle*/0.2));
					}
					if (volume_photon_find.size() != 0) {
						currentStep = currentStep.colorScalar(1.0/(pow(NATUREE, pass)*PHOTONMUM/8*PI*maxDistSqr*sqrt(maxDistSqr)));
					}
					sssColor = sssColor.colorAdd(currentStep);
					double d = -log(dis_rand(gen))/200;

					pos = pos.vectAdd(reflection_direction.vectMult(d));
					/*
					if (pass >= 3) {
						double stopScatter = dis_rand(gen);
						if (stopScatter > (1 - (pass/5)))
							break;
					}
					*/

				}
				//sssColor = sssColor.colorScalar(1.0/pass); 
			}
		}
		final_color = final_color.colorAdd(sssColor);
	}
	return final_color.clip();
}



void photonEmission(Ray photon_ray, Vect photon_ray_direction, vector<Object *> scene_objects, double accuracy, double ambientLight, Color lightColor, int bounce) {
	//cout << bounce << endl;
	if (bounce> BOUNCE)
		return;

    double intersect_dist = 1e9;
    Object * intersect_obj = bvh->Shoot(photon_ray, intersect_dist);

    if (intersect_obj == nullptr)
        return;

	if (intersect_dist > accuracy) {
		Vect intersection_position = photon_ray.getRayOrigin().vectAdd(photon_ray_direction.vectMult(accuracy + intersect_dist));
		Vect intersecting_ray_direction = photon_ray_direction;
		Vect winning_object_normal = intersect_obj->getNormalAt(intersection_position);

		float reflectance = 1;
		int refractMask[3] = {1,1,1};
		bool canTransmit = true;

		double colorSpecial = intersect_obj->getColor(intersection_position).getColorSpecial();

		if (colorSpecial <= 0) {
			lightColor = storePhoton(intersection_position, intersecting_ray_direction, intersect_obj, accuracy, ambientLight, lightColor, bounce);
			lightColor = lightColor.colorScalar(0.8);

			// not use Russian Roulette for lambert model 
			/*
			float cosTheta = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
			float n2 = scene_objects.at(index_of_winning_object)->getRefraIdx();

			reflectance = FresnelEquation(cosTheta, 1, n2, 0);
			
			int roll = dis_rand_100(gen);
			if (roll > reflectance * 100) {   // absorb rate of 0.2. TODO: add absorb/refraction/refraction rate
				canTransmit = false;
			}
			*/
		}
		else if (colorSpecial < 1 && colorSpecial > 0) {
			float cosTheta = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
			float n2 = intersect_obj->getRefraIdx();

			reflectance = FresnelEquation(cosTheta, 1, n2, 0);
			int roll = dis_rand_100(gen);
			if (roll > reflectance * 100) {   // absorb rate of 0.2. TODO: add absorb/refraction/refraction rate
				canTransmit = false;
			}
		}
		else if (colorSpecial < 2 && colorSpecial > 0) {
			for (int i = 0; i < 3; i++) {
				float cosTheta = winning_object_normal.dotProduct(intersecting_ray_direction.negtive());
				float n2 = intersect_obj->getRefraIdx()-0.01 + i/100.0;

				if (cosTheta > 0) 
					reflectance = FresnelEquation(cosTheta, 1, n2, 1);
				else {
					cosTheta = winning_object_normal.negtive().dotProduct(intersecting_ray_direction.negtive());
					reflectance = FresnelEquation(cosTheta, n2, 1, 1);
				}
				int roll = dis_rand_100(gen);
				if (roll > reflectance * 100) {   // absorb rate of 0.2. TODO: add absorb/refraction/refraction rate
					refractMask[i] = 0;
				}
			}
		}

		if (canTransmit) {
			if (intersect_obj->getColor(intersection_position).getColorSpecial() == 0) {  // Lambert model
				double Zeta1 = dis_rand(gen);
				double Zeta2 = dis_rand(gen);
				
				double theta = acos(sqrt(Zeta1));
				double phi = 2 * PI * Zeta2;

				double x = sin(theta) * cos(phi);
				double z = sin(theta) * sin(phi);
				double y = cos(theta);		

				Vect u = intersect_obj->getTangentAt(intersection_position);
				Vect v = winning_object_normal.crossProduct(u);

				Vect world_ref_ray_dir = u.vectMult(x).vectAdd(v.vectMult(z).vectAdd(winning_object_normal).vectMult(y));
				world_ref_ray_dir = world_ref_ray_dir.normalize();

				photon_ray = Ray(intersection_position, world_ref_ray_dir);
				photon_ray_direction = world_ref_ray_dir;
				photonEmission (photon_ray, photon_ray_direction, scene_objects, accuracy, ambientLight, lightColor, bounce+1);		
			}
			else if (colorSpecial <= 1 && colorSpecial > 0) {
				double dot1 = winning_object_normal.dotProduct(intersecting_ray_direction.negtive()); // N*L
				Vect scalar1 = winning_object_normal.vectMult(dot1); // (N*L)*N
				Vect add1 = scalar1.vectAdd(intersecting_ray_direction);
				Vect scalar2 = add1.vectMult(2);
				Vect add2 = intersecting_ray_direction.negtive().vectAdd(scalar2);
				Vect reflection_dir = add2.normalize();

				photon_ray = Ray(intersection_position, reflection_dir);
				photon_ray_direction = reflection_dir;

				Color winning_object_color = intersect_obj->getColor(intersection_position);

				lightColor.setColorRed(min(winning_object_color.getColorRed(), lightColor.getColorRed()));
				lightColor.setColorGreen(min(winning_object_color.getColorGreen(), lightColor.getColorGreen()));
				lightColor.setColorBlue(min(winning_object_color.getColorBlue(), lightColor.getColorBlue()));

				photonEmission (photon_ray, photon_ray_direction, scene_objects, accuracy, ambientLight, lightColor, bounce+1);
			}
			else if (colorSpecial <= 2 && colorSpecial > 1) {
				for (int i = 0; i < 1; i++) {
					if (refractMask[i] == 0) continue;
					double n1n2 = 1.0 / (intersect_obj->getRefraIdx()-0.01 + i/100.0);
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
							//refractColor.setColorGreen(0);
							//refractColor.setColorBlue(0);
						}
						else if (i == 1) {
							refractColor = lightColor;
							//refractColor.setColorRed(0);
							//refractColor.setColorBlue(0);
						}
						else if (i == 2) {
							refractColor = lightColor;
							//refractColor.setColorRed(0);
							//refractColor.setColorGreen(0);
						}
						if (!(refractColor.getColorRed() == 0 && refractColor.getColorGreen() == 0 && refractColor.getColorBlue() == 0)) {
							Color winning_object_color = intersect_obj->getColor(intersection_position);

							refractColor.setColorRed(min(winning_object_color.getColorRed(), lightColor.getColorRed()));
							refractColor.setColorGreen(min(winning_object_color.getColorGreen(), lightColor.getColorGreen()));
							refractColor.setColorBlue(min(winning_object_color.getColorBlue(), lightColor.getColorBlue()));

							photonEmission (photon_ray, photon_ray_direction, scene_objects, accuracy, ambientLight, refractColor, bounce+1);

						}
					}		
				}
			}	

			if (colorSpecial < 0) { //sss
				// calc spherical coord
				double d = 0.005;
				Vect pos = intersection_position.vectAdd(intersecting_ray_direction.vectMult(d));
				Vect sssDir = intersecting_ray_direction;

				std::uniform_real_distribution<> phi_rand(-PI, PI);
				//d = 0.02;
				for (int s = 0; s < 4; s++) {
					double theta = acos(sssDir.getVectZ());
					//double phi = atan2(sssDir.getVectY(), sssDir.getVectX());
					double phi = phi_rand(gen);
					double g = 0.3;

					double newCosTheta = 1/abs(2*g)*(1+g*g-pow(((1-g*g)/(1-g+2*g*dis_rand(gen))),2));
					double newTheta = /*theta + */acos(newCosTheta);

					double newX = sin(newTheta) * cos(phi);
					double newY = sin(newTheta) * sin(phi);
					double newZ = cos(newTheta);

					sssDir = Vect(newX, newY, newZ).normalize();

					Ray scatterRay(pos, sssDir);

                    double intersect_distance = d;
                    Object * intersect_obj = bvh->Shoot(scatterRay, intersect_distance);

                    if (intersect_obj == nullptr) {
							pos = pos.vectAdd(sssDir.vectMult(d));
							storeVolumePhoton(pos, sssDir, lightColor);
							lightColor = lightColor.colorScalar(0.6);
					}
					else
						break;
				}
			}
		}		
	}
}


int main(int argc, char *argv[])
{
    CalcTime("Preparing Scene...");

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

	Vect new_sphere_pos (0.3, -0.4, -0.4);
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
	Color refractWater (0.26, 0.84, 1.0, 1.8);
#endif
	Color maroon(0.5, 0.25, 0.25, 2);
	Color pretty_maroon(0.5, 0.25, 0.25, 0.6);
	Color gray(0.5, 0.5, 0.5, 0);
	Color black(0.0, 0.0, 0.0, 0);
	Color orange(0.94, 0.75, 0.31, 0);
	Color sssWhite(1.0, 1.0, 1.0, -0.9);

	Vect light_position(0.1, 0.2, 0);
	//Vect light_position(-0.8, 0.2, -0.4);
	Light scene_light (light_position, white_light);
	vector<Source *> light_sources;
	light_sources.push_back(dynamic_cast<Source *>(&scene_light));

#ifdef LOADOBJ
	TextureMap* map=new TextureMap();
    ObjReader* objReader = new ObjReader("bunny_normal.obj", sssWhite, 1.6, 0.2, -1.0, 0.0, map, false);
	objReader->ReadContent(&scene_objects);
#endif

	//Sphere scene_sphere (new_sphere_pos, 0.3, reflectWhite, 220);
	Sphere scene_sphere (new_sphere_pos, 0.3, reflectWhite, 1.5);
	Sphere scene_sphere2 (new_sphere_pos2, 0.3, refractWhite, 1.5);
	Triangle scene_triangle (Vect(3, 0, 0), Vect(0, 3, 0), Vect(0, 0, 3), orange, 20);

	//scene_objects.push_back(dynamic_cast<Object *>(&scene_sphere));
	//scene_objects.push_back(dynamic_cast<Object *>(&scene_sphere2));

	makeCornellBox(scene_objects, Vect(1, 1, 1), Vect(-1, -1, -1));

    CalcTime("start build BVH...");

    bvh = new BVH(scene_objects);
    BVHSelfTest(scene_objects, bvh);

    CalcTime("start emit photons...");

	std::uniform_real_distribution<> dis(-1, 1);
	//Color lightColor = white_light.colorScalar(32.0/PHOTONMUM);
	Color lightColor = white_light;

    std::vector<std::thread> threads;

#if MULTI_THREAD == 1
    unsigned num_threads = 2 * std::thread::hardware_concurrency();
#else
    unsigned num_threads = 1;
#endif

    assert( (width % num_threads) == 0);

for(int i = 0; i < num_threads; i++)
    threads.push_back(thread([&](){
		int pn = 0;
		while (pn < PHOTONMUM/num_threads) {
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
	}));
    for (auto& th : threads) th.join();
    threads.clear();

    CalcTime("start build kdtree...");

	kdtree = new KDTree(photons);

	if (volumePhotons.size() > 3) {
		volumeKDTree = new KDTree(volumePhotons);
	}

    KDTreeSelfTest(kdtree, photons);

    CalcTime("start raytracing...");


    for(int xdiv = 0; xdiv < num_threads; xdiv++)
    threads.push_back(thread([&](int x1,int x2){
        for (int x = x1; x < x2; x++)
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

					Vect focalPoint = cam_ray_origin.vectAdd(cam_ray_direction.vectMult(FOCAL_LEN));
					Vect worldUp(0, 1, 0);
					Vect camUp = worldUp.vectAdd(cam_ray_direction.vectMult(worldUp.dotProduct(cam_ray_direction)).negtive());
					Vect camSide = camUp.crossProduct(cam_ray_direction);

					double dofColor[3] = {0,0,0};

					for (int d = 0; d < DOF_SAMPLE; d++) {
						Ray cam_ray (cam_ray_origin, cam_ray_direction);

						if (DOF_SAMPLE != 1) {
							double x = dis_rand_dof(gen);
							double y = dis_rand_dof(gen);
						
							Vect camPosDoF = cam_ray_origin.vectAdd(camUp.vectMult(x)).vectAdd(camSide.vectMult(y));
							Vect camDirDoF = focalPoint.vectAdd(camPosDoF.negtive()).normalize();

							cam_ray = Ray(camPosDoF, camDirDoF);
						}

                        double intersect_distance = 1e9;
                        Object * intersect_obj = bvh->Shoot(cam_ray, intersect_distance);

						if (intersect_obj == nullptr) {
							dofColor[0] += 0;
							dofColor[1] += 0;
							dofColor[2] += 0;
						}
						else if (intersect_distance > accuracy) {
								Vect intersection_position = cam_ray.getRayOrigin().vectAdd(cam_ray_direction.vectMult(intersect_distance));
								Vect intersecting_ray_direction = cam_ray_direction;
	
								Color intersection_color = getColorAt(intersection_position, intersecting_ray_direction, intersect_obj, accuracy, ambientLight, 1);
	
								dofColor[0] += intersection_color.getColorRed();
								dofColor[1] += intersection_color.getColorGreen();
								dofColor[2] += intersection_color.getColorBlue();
						}
					}
					tempRed[aa_index] = dofColor[0]/DOF_SAMPLE;
					tempGreen[aa_index] = dofColor[1]/DOF_SAMPLE;
					tempBlue[aa_index] = dofColor[2]/DOF_SAMPLE;
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
    },width / num_threads * xdiv, width / num_threads * (xdiv + 1)));
    for (auto& th : threads) th.join();
    threads.clear();

    CalcTime("start saving file...");

	saveBmp("scene.bmp", width, height, dpi, pixels);

	//delete pixels, tempRed, tempGreen, tempBlue;;

    CalcTime("All Done...");

	//delete kdtree;
#ifdef LOADOBJ
	delete objReader;
#endif
	system("pause");
	return 0;
}