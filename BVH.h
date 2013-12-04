#ifndef _BVH_H
#define _BVH_H

#include "System.h"

#include "Ray.h"
#include "Object.h"

class Object;

struct BBox
{
    // x,y,z | min,max
    double data[3][2];

    class centerComparison {
        int axis;
    public:
        centerComparison(int axis_):axis(axis_){};
        bool operator ()(const Object *lhs, const Object *rhs) const;
    };

    Vect Center()
    {
        return Vect((data[0][1] + data[0][0]) / 2,
                (data[1][1] + data[1][0]) / 2,
                (data[2][1] + data[2][0]) / 2);
    }
    BBox()
    {
        //memset(data, 0, sizeof(double) * 6);
    }
    double Size() const
    {
        return data[0][1] - data[0][0]
            + data[1][1] - data[1][0]
            + data[2][1] - data[2][0];
    }
    BBox Union(const BBox &other);
    double Intersect(const Ray& ray, double current_best) const;
};

struct BVHNode
{
    static const int max_elements = 16;
    BVHNode* child[2];
    BBox bbox;
    vector<Object*> objects;
    void splitNode();
    void calculateBBox();
    BVHNode()
    {
        child[0] = child[1] = nullptr;
    }
};

class BVH
{
private:
#if BVH_PERF_TEST == 1
    mutable int performance_counter;
    mutable int performance_counter_ch;
    mutable int total_obj;
#endif
    BVHNode root;
    Object *Shoot(const BVHNode &node, const Ray &ray, double &distance) const;

public:
    BVH(vector<Object *> &in_objects);
    // return the nearest intersect object
    // and the distance to the intersect point
    // which below the distance passed in
    Object *Shoot(const Ray &ray, double &distance) const;
};

void BVHSelfTest(vector<Object *> objects, BVH *bvh);

#endif