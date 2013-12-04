#include "System.h"
#include "Vect.h"
#include "Photon.h"
#include "Kdtree.h"

#define PHOTONDIST 0.28

#define PI 3.1415926
#define BOUNCE 3

void KDTree::createKD_(vector<Photon *> &photons, KDNode *parent)
{
	assert(photons.size() > max_elements);
    double span_axis[3];
    for(int i = 0; i < 3; i++)
    {
        auto span = minmax_element(photons.begin(), photons.end(),
                [&](const Photon *a, const Photon *b) -> bool
                        {return a->position[i] < b->position[i];});
        span_axis[i] = (*span.second)->position[i] - (*span.first)->position[i] ;
    }
    parent->axis = max_element(span_axis, span_axis + 3) - span_axis;

	sort(photons.begin(), photons.end(),
		[&](const Photon *a, const Photon *b) -> bool
            {return a->position[parent->axis] < b->position[parent->axis];});

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
		createKD_(left, parent->left_branch);
	}
	if (right.size() <= max_elements) {
		parent->right = right;
	}
	else {
		parent->right_branch = new KDNode();
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
void KDTree::updateQueue( int k, Photon * photon, const Vect& center ,
        priority_queue<Photon *, vector<Photon *>, distanceComparison>& KNN_queue, double &current_best_sqr) const
{
    double dist = photon->position.sqrDist(center);
    if (KNN_queue.size() < k && dist < PHOTONDIST * PHOTONDIST) {
        KNN_queue.push(photon);
        return;
    }
    float dis = 1e6;
    if (dist < current_best_sqr && dist < PHOTONDIST * PHOTONDIST) {
        Photon *prev_topPhoton = KNN_queue.top();
        KNN_queue.push(photon);
        Photon *topPhoton = KNN_queue.top();
        if(prev_topPhoton != topPhoton)
            current_best_sqr = topPhoton->position.sqrDist(center);
        KNN_queue.pop();
    }
}

void KDTree::findKNN_(int k, Vect center, KDNode *root,
        priority_queue<Photon *, vector<Photon *>, distanceComparison>& KNN_queue, double &current_best_sqr) const
{
    if(root == nullptr)
        return;

    double dis = 1e6;
    float farDistance = min(PHOTONDIST * PHOTONDIST * 1., current_best_sqr);

	for (int i = 0; i < root->left.size(); i++) {
#if KD_PERF_TEST == 1
        performance_counter++;
#endif
        updateQueue(k,root->left[i],center,KNN_queue,current_best_sqr);
	}

	for (int i = 0; i < root->right.size(); i++) {
#if KD_PERF_TEST == 1
        performance_counter++;
#endif
        updateQueue(k,root->right[i],center,KNN_queue,current_best_sqr);
	}

	// Pivot Self
#if KD_PERF_TEST == 1
    performance_counter++;
#endif
    updateQueue(k,root->self,center,KNN_queue,current_best_sqr);

	if (center[root->axis] < root->self->position[root->axis]) {
			findKNN_(k, center, root->left_branch, KNN_queue,current_best_sqr);
		if ((center[root->axis] - root->self->position[root->axis])
                * (center[root->axis] - root->self->position[root->axis]) <= farDistance)
            findKNN_(k, center, root->right_branch, KNN_queue,current_best_sqr);
	}
	else {
		findKNN_(k, center, root->right_branch, KNN_queue,current_best_sqr);
        if ((center[root->axis] - root->self->position[root->axis])
                * (center[root->axis] - root->self->position[root->axis]) <= farDistance)
        findKNN_(k, center, root->left_branch, KNN_queue,current_best_sqr);
	}
}

vector<Photon *> KDTree::findKNN(int k, Vect center) const
{
    double current_best_sqr = 1e9;
#if KD_PERF_TEST == 1
    performance_counter = 0;
#endif
    auto KNN_queue = priority_queue<Photon *, vector<Photon *>, distanceComparison>
			(distanceComparison(center));

	findKNN_(k, center, Root, KNN_queue,current_best_sqr);

	vector<Photon *> result;
	while (KNN_queue.size() > 0) {
		result.push_back(KNN_queue.top());
		KNN_queue.pop();
	}
	reverse(result.begin(), result.end());
#if KD_PERF_TEST == 1
    if(performance_counter * 1. / total_obj > 0.1)
        cout <<"POINT:" << center[0] << "," << center[1] << "," << center[2] << endl;
    cout << "KD: " << k << ":" << performance_counter << ":" << total_obj << endl;
#endif
	return result;
}


void KDTreeSelfTest(KDTree* kdtree, vector<Photon *> photons){
    std::uniform_real_distribution<> rg(-1, 1);
    for(int i_case = 0; i_case < KD_SELF_TEST_TIMES; i_case++)
    {
        int k;
        const Vect Goal(rg(gen),rg(gen),rg(gen));
        vector<Photon *> testfind = kdtree->findKNN(k, Goal);
        vector<Photon *> copy_test = photons;
        sort(copy_test.begin(), copy_test.end(),
                [Goal](const Photon *a, const Photon *b) -> bool
                        {
                            return a->position.sqrDist(Goal) <
                                    b->position.sqrDist(Goal);
                        });

        for (int i = 0; i < testfind.size(); i++) {
            if(testfind[i]->position.sqrDist(Goal) -
                    copy_test[i]->position.sqrDist(Goal) > 1e-9)
            {
                cout << "KD TEST FAILED AT case: " << i_case << endl;
                cout << Goal.getVectX() << " "
                        << Goal.getVectY() << " "
                        << Goal.getVectZ() << " "
                        << endl;
                cout << "IDX: " << i << " Real Distance: "
                        << copy_test[i]->position.sqrDist(Goal)
                        << " Wrong Distance: "
                        << testfind[i]->position.sqrDist(Goal) << endl;
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
        if(testfind.size() > 0)
        assert(copy_test[testfind.size() - 1]->position.sqrDist(Goal) <= PHOTONDIST * PHOTONDIST);
        assert(copy_test[testfind.size()]->position.sqrDist(Goal) > PHOTONDIST * PHOTONDIST
        ||testfind.size() == k);
    }
#if KD_SELF_TEST_TIMES > 0
    cout << "KD Self Test Passed\n";
#endif
}
