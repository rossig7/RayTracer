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
#include "Photon.h"
#include "Kdtree.h"

#define PHOTONDIST 0.5

#define PI 3.1415926
#define BOUNCE 3

using namespace std;


void KDTree::createKD_(vector<Photon *> &photons, KDNode *parent)
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
		createKD_(left, parent->left_branch);
	}
	if (right.size() <= max_elements) {
		parent->right = right;
	}
	else {
		parent->right_branch = new KDNode();
		parent->right_branch->axis = (parent->axis + 1) % 3;
		createKD_(right, parent->right_branch);
	}

	return;
}

void KDTree::createKD(vector<Photon *> &photons)
{
	if(Root)
		destroyKD_(Root);
	Root = new KDNode();
	Root->axis = 0;
	createKD_(photons,Root);
}

void KDTree::destroyKD_(KDNode *parent)
{
	if(parent->left_branch)
		destroyKD_(parent->left_branch);

	if(parent->right_branch)
		destroyKD_(parent->right_branch);

	delete parent;
}

void KDTree::findKNN_(int k, Vect center, KDNode *root,
        priority_queue<Photon *, vector<Photon *>, distanceComparsion>& KNN_queue)
{
	/*
	Photon * topPhoton = KNN_queue->top();
	float farDistance = topPhoton->position.sqrDist(center);
    */

	for (int i = 0; i < root->left.size(); i++) {
		//perf_count++;
		if (KNN_queue.size() < k &&
				root->left[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue.push(root->left[i]);
			continue;
		}
		float dis = 1e3;
		if (KNN_queue.size() > 0) {
			Photon *topPhoton = KNN_queue.top();
			dis = topPhoton->position.sqrDist(center);
		}
		if (root->left[i]->position.sqrDist(center) < dis
				&& root->left[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue.push(root->left[i]);
			KNN_queue.pop();
		}
	}

	for (int i = 0; i < root->right.size(); i++) {
		//perf_count++;
		if (KNN_queue.size() < k &&
				root->right[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue.push(root->right[i]);
			continue;
		}
		float dis = 1e3;
		if (KNN_queue.size() > 0) {
			Photon *topPhoton = KNN_queue.top();
			dis = topPhoton->position.sqrDist(center);
		}
		if (root->right[i]->position.sqrDist(center) < dis
				&& root->right[i]->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue.push(root->right[i]);
			KNN_queue.pop();
		}
	}

	// Pivot Self
	// perf_count++;
	if (KNN_queue.size() < k &&
			root->self->position.sqrDist(center) < PHOTONDIST) {
		KNN_queue.push(root->self);
	}
	else {
		float dis = 1e3;
		if (KNN_queue.size() > 0) {
			Photon *topPhoton = KNN_queue.top();
			dis = topPhoton->position.sqrDist(center);
		}
		if (root->self->position.sqrDist(center) < dis &&
				root->self->position.sqrDist(center) < PHOTONDIST) {
			KNN_queue.push(root->self);
			KNN_queue.pop();
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
			findKNN_(k, center, root->left_branch, KNN_queue);
		}
		float dis = 1e3;
		if (KNN_queue.size() > 0) {
			Photon *topPhoton = KNN_queue.top();
			dis = topPhoton->position.sqrDist(center);
		}
		float farDistance = min(PHOTONDIST * PHOTONDIST * 1., dis * 1.);
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
			findKNN_(k, center, root->right_branch, KNN_queue);
		}
	}
	else {
		if (root->right_branch) {
			findKNN_(k, center, root->right_branch, KNN_queue);
		}

		float dis = 1e3;
		if (KNN_queue.size() > 0) {
			Photon *topPhoton = KNN_queue.top();
			dis = topPhoton->position.sqrDist(center);
		}
		float farDistance = min(PHOTONDIST * PHOTONDIST * 1., dis * 1.);
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
			findKNN_(k, center, root->left_branch, KNN_queue);
		}

	}
}

vector<Photon *> KDTree::findKNN(int k, Vect center)
{
    auto KNN_queue = priority_queue<Photon *, vector<Photon *>, distanceComparsion>
			(distanceComparsion(center));

	findKNN_(k, center, Root, KNN_queue);

	vector<Photon *> result;
	while (KNN_queue.size() > 0) {
		result.push_back(KNN_queue.top());
		KNN_queue.pop();
	}
	reverse(result.begin(), result.end());
	return result;
}

// Test kd tree
/*
const Vect Goal(1,0.5,0.4);
perf_count = 0;
vector<Photon *> testfind = findKNN(20, Goal, Root);
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