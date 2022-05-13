//
// Created by monica on 2021/3/15.
//

#include "fcl/BV/SPHERE.h"
#include "fcl/BVH/BVH_utility.h"
#include <iostream>
#include <math.h>
#include <limits>

namespace fcl {

SPHERE::SPHERE() : Tr(0,0,0),
                   r(-1)
{
    axis[0].setValue(1,0,0);
    axis[1].setValue(0,1,0);
    axis[2].setValue(0,0,1);
}

SPHERE::SPHERE(const Vec3f &v) : Tr(v.data.vs[0], v.data.vs[1], v.data.vs[2]),
                                 r(std::numeric_limits<fcl::FCL_REAL>::epsilon())
{
    axis[0].setValue(1,0,0);
    axis[1].setValue(0,1,0);
    axis[2].setValue(0,0,1);
}

bool overlap(const Vec3f &T0, const SPHERE &b1, const SPHERE &b2) {
    Vec3f d = b2.Tr - b1.Tr;
    if ((d.dot(d) - pow((b1.r + b2.r), 2)) > 0) return false;
    else return true;
}

SPHERE &SPHERE::operator += (const Vec3f &p) {
    // create a SHPERE other at position p with zero size
    SPHERE other(p);
    // return *this + other
    SPHERE res(*this);
    return res += other;
}

SPHERE& SPHERE::operator += (const SPHERE &other) {
    // Computer the squared distance between the sphere Trs
    Vec3f  d = Tr - other.Tr;
    FCL_REAL dist2 = d.data.vs[0] * d.data.vs[0] + d.data.vs[1] * d.data.vs[1] + d.data.vs[2] * d.data.vs[2];

    SPHERE s;
    if (r + other.r >= dist2)
    {
        // The sphere with the larger radius encloses the other;
        // just set s to be the larger of the two spheres
        if (r > other.r)
            s = *this;
        else
            s = other;
    } else {
        // Spheres partially overlapping or disjoint
        float dist = sqrt(dist2);
        s.r = (dist + r + other.r) * 0.5;
        s.Tr = Tr;
        if(dist > 0)    // epsilon = 0
            s.Tr += ((s.r - r) / dist) * d;
    }

    return *this;
}

FCL_REAL SPHERE::distance(const SPHERE &other, Vec3f *P, Vec3f *Q) const {
    Vec3f d = this->Tr - other.Tr;
    std::cout << "return distance of two spheres, but not return nearest points";
    return sqrt(d.dot(d) - pow((this->r - other.r),2));
}

FCL_REAL distance(const Matrix3f &R0, const Vec3f &T0, const SPHERE &b1, const SPHERE &b2, Vec3f *P, Vec3f *Q) {
    Vec3f d = b1.Tr - (T0 + b2.Tr);
    std::cout << "return distance of two spheres, but not return nearest points";
    return sqrt(d.dot(d) - pow((b1.r - b2.r),2));
}

SPHERE translate(const SPHERE& bv, const Vec3f& t)
{
    SPHERE res(bv);
    res.Tr += t;
    return res;
}

}
