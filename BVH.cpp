#include "System.h"

#include "BVH.h"

double BBox::Intersect(const Ray & ray, double current_best) const
{
    // will return 0, if inside
    // Grab from PBRT 2 P194
    double t0 = 0. , t1 = current_best;
    for(int i = 0; i < 3; i++)
    {
        double inv_ray = 1. / ray.getRayDirection()[i];
        double far_t   = (data[i][0] - ray.getRayOrigin()[i]) * inv_ray;
        double near_t  = (data[i][1] - ray.getRayOrigin()[i]) * inv_ray;
        if(near_t > far_t) swap(near_t, far_t);
        // come in when cross all 3 axises
        // out when cross 1 axis, also clipped by known_best
        t0 = near_t > t0 ? near_t : t0   ;
        t1 = far_t  > t1 ? t1     : far_t;
        // out before in, no intersect
        if(t0 > t1) return -1;
    }
    return t0;
}

bool BBox::centerComparison::operator ()(const Object *lhs, const Object *rhs) const
{
    return lhs->getBBox().Center()[axis] < rhs->getBBox().Center()[axis];
}

BBox BBox::Union(const BBox &other)
{
    BBox result;
    for(int i = 0; i < 3; i++)
    {
        result.data[i][0] = min(data[i][0],other.data[i][0]);
        result.data[i][1] = max(data[i][1],other.data[i][1]);
    }
    return result;
}

void BVHNode::calculateBBox()
{
    if(objects.size() > 0)
    {
        bbox = objects[0]->getBBox();
        for(auto object: objects)
            bbox = bbox.Union(object->getBBox());
    }
    else
    {
        bbox = child[0]->bbox;
        bbox = bbox.Union(child[1]->bbox);
    }
}

void BVHNode::splitNode()
{
    if(objects.size() < max_elements)
        return;
    double span_axis[3];
    for(int i = 0; i < 3; i++)
    {
        auto span = minmax_element(objects.begin(), objects.end(),BBox::centerComparison(i));
        span_axis[i] = (*span.second)->getBBox().Center()[i] - (*span.first)->getBBox().Center()[i];
    }
    int split_axis = max_element(span_axis, span_axis + 3) - span_axis;
    sort(objects.begin(), objects.end(),BBox::centerComparison(split_axis));
    vector<BBox> left_box(objects.size(),objects[0]->getBBox()),
            right_box(objects.size(),objects[objects.size() - 1]->getBBox());
    for(int i = 1; i < objects.size() ; i++)
    {
        left_box[i] = left_box[i - 1].Union(objects[i]->getBBox());
        right_box[i] = right_box[i - 1].Union(objects[objects.size() - 1 - i]->getBBox());
    }
    vector<double> split_quality(objects.size(),1e9);
    int bad_zone = max(1,(int)objects.size()/3);
    for(int i = 0; i < objects.size() ; i++)
    {
        if(i < bad_zone || i > objects.size() - 1 - bad_zone)
            continue;
        split_quality[i] = left_box[i].Size() +
                right_box[objects.size() - 1 - i].Size();
    }
    int split_pos = min_element(split_quality.begin(), split_quality.end()) - split_quality.begin();
    child[0] = new BVHNode();
    child[0]->objects.assign(objects.begin(), objects.begin() + split_pos + 1);
    child[0]->calculateBBox();
    child[0]->splitNode();
    child[1] = new BVHNode();
    child[1]->objects.assign(objects.begin() + split_pos + 1, objects.end());
    child[1]->calculateBBox();
    child[1]->splitNode();
    objects.clear();
    calculateBBox();
}

BVH::BVH(vector<Object *> & in_objects)
{
#if BVH_PERF_TEST == 1
    total_obj = in_objects.size();
#endif
    root.objects = in_objects;
    root.calculateBBox();
    root.splitNode();
}

Object * BVH::Shoot(const Ray &ray, double &distance) const
{
#ifdef __APPLE__
    if(isnan(ray.getRayDirection()[0]))
#else
    if(_isnan(ray.getRayDirection()[0]))
#endif
        return NULL;

#if BVH_PERF_TEST == 1
    performance_counter = 0;
    performance_counter_ch = 0;
#endif
    if(root.bbox.Intersect(ray, distance) < 0)
        return nullptr;
    Object * r = Shoot(root, ray, distance);
#if BVH_PERF_TEST == 1
    if(performance_counter > 10000)
    {
        cout << ray.getRayOrigin()[0] << ","  << ray.getRayOrigin()[1] << ","<< ray.getRayOrigin()[2] << "," <<endl;
        cout << ray.getRayDirection()[0] << ","  << ray.getRayDirection()[1] << ","<< ray.getRayDirection()[2] << "," <<endl;
    }
    // cout << performance_counter_ch << ":" << performance_counter << ":" << total_obj << endl;
#endif
    return r;
}

Object * BVH::Shoot(const BVHNode &node, const Ray &ray, double &distance) const
{
    if(node.bbox.Intersect(ray, distance) < 0)
        return nullptr;
    Object * result = nullptr;
    for(auto object: node.objects)
    {
        double t = object->findIntersection(ray);
#if BVH_PERF_TEST == 1
        performance_counter++;
#endif
        if(t >= 0 && t < distance)
        {
#if BVH_PERF_TEST == 1
            performance_counter_ch++;
#endif
            result = object;
            distance = min(distance , t);
        }
    }
    double test_distance[2] = {1e6,1e6};

    if(node.child[0] == nullptr)
        return result;

    for(int i = 0; i < 2; i++)
        test_distance[i] = node.child[i]->bbox.Intersect(ray, distance);

    Object * result_child = nullptr;
    if(test_distance[0] < test_distance[1])
    {
        result_child = Shoot(*(node.child[0]), ray, distance);
        if(result_child)
            result = result_child;
        result_child = Shoot(*(node.child[1]), ray, distance);
        if(result_child)
            result = result_child;
    }
    else
    {
        result_child = Shoot(*(node.child[1]), ray, distance);
        if(result_child)
            result = result_child;
        result_child = Shoot(*(node.child[0]), ray, distance);
        if(result_child)
            result = result_child;
    }

    return result;
}

// Only used for testing
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

void BVHSelfTest(vector<Object *> objects, BVH *bvh)
{
    // BVH self check
    for(int i = 0; i < BVH_SELF_TEST_TIMES; i++)
    {
        std::uniform_real_distribution<> rg(-1, 1);

        vector<double> intersections;
        Ray r(Vect(rg(gen),rg(gen),rg(gen)),Vect(rg(gen),rg(gen),rg(gen)));
        for (int index = 0; index < objects.size(); index++) {
            intersections.push_back(objects.at(index)->findIntersection(r));
        }

        int index_of_winning_object = winningObjectIndex(intersections);
        Object * ref = NULL;
        if(index_of_winning_object >= 0)
            ref = objects.at(index_of_winning_object);

        double intersect_dist = 1e9;
        Object * intersect_obj = bvh->Shoot(r, intersect_dist);

        assert(intersect_obj == ref);
    }

#if BVH_SELF_TEST_TIMES > 0
    cout << "BVH self test passed \n";
#endif
}