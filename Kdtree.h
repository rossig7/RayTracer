#ifndef _KDTREE_H
#define _KDTREE_H

#include "Vect.h"
#include "BVH.h"
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
	class distanceComparison {
		Vect center;
	public:
		distanceComparison(Vect center_)
		{
			center = center_;
		}
		bool operator ()(const Photon *lhs, const Photon *rhs) const
		{
			return lhs->position.sqrDist(center) < rhs->position.sqrDist(center);
		}
	};

#if KD_PERF_TEST == 1
    mutable int performance_counter;
    mutable int total_obj;
#endif
	static const int max_elements = 8;
	KDNode *Root;

    void updateQueue( int k, Photon * photon,const Vect& center,
            priority_queue<Photon *, vector<Photon *>, distanceComparison>& KNN_queue, double &current_best_sqr) const;
	void destroyKD_(KDNode *parent);
	void createKD_(vector<Photon *> &photons, KDNode *parent);
	void findKNN_(int k, Vect center, KDNode *root,
            priority_queue<Photon *, vector<Photon *>, distanceComparison>& KNN_queue,double &current_best_sqr) const;
	KDTree()
	{
		Root = NULL;
	}

public:
	KDTree(vector<Photon *> &photons)
	{
		Root = NULL;
#if KD_PERF_TEST == 1
        total_obj = photons.size();
#endif
		createKD(photons);
	}
	~KDTree()
	{
		destroyKD_(Root);
	}
	vector<Photon *> findKNN(int k, Vect center) const;
	void createKD(vector<Photon *> &photons);
};

void KDTreeSelfTest(KDTree* kdtree, vector<Photon *> photons);
#endif // !_PHTOTN_H
