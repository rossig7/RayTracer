#ifndef _KDTREE_H
#define _KDTREE_H

#include "Vect.h"
#include <vector>
#include "Photon.h"

using namespace std;

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

class KDTree{
private:
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

	int perf_count;
	static const int max_elements = 3;
	KDNode *Root;
	priority_queue<Photon *, vector<Photon *>, distanceComparsion> *KNN_queue;

	void destroyKD_(KDNode *parent);
	void createKD_(vector<Photon *> &photons, KDNode *parent);
	void findKNN_(int k, Vect center, KDNode *root);
	KDTree()
	{
		perf_count = 0;
		Root = NULL;
	}

public:
	KDTree(vector<Photon *> &photons)
	{
		perf_count = 0;
		Root = NULL;
		createKD(photons);
	}
	~KDTree()
	{
		destroyKD_(Root);
	}
	vector<Photon *> findKNN(int k, Vect center);
	void createKD(vector<Photon *> &photons);
};


#endif // !_PHTOTN_H
