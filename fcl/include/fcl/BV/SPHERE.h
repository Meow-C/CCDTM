//
// Created by monica on 2021/3/15.
//

#ifndef FCL_SPHERE_H
#define FCL_SPHERE_H

#include "fcl/math/constants.h"
#include "fcl/math/vec_3f.h"
#include "fcl/math/matrix_3f.h"

namespace fcl
{

/// @brief A class for sphere bounding volume
class SPHERE
{
public:
    /// @brief The SPHERE center
    Vec3f Tr;

    /// @brief Radius of SPHERE
    FCL_REAL r;

    /// @brief Orientation of SPHERE.
    /// The Orientation has no practical meaning for the sphere, it is only used for BVSplit.
    Vec3f axis[3];

    /// @brief Creating a SPHERE with zero size (r +inf)
    SPHERE();

    /// @brief Creating a SPHERE at position v with zero size
    SPHERE(const Vec3f& v);

    /// @brief Creating a SPHERE at position v with radius radius
    SPHERE(const Vec3f& v, const FCL_REAL radius) : Tr(v.data.vs[0], v.data.vs[1], v.data.vs[2]),
                                                    r(radius)
    {
        Tr.setValue(v.data.vs[0], v.data.vs[1], v.data.vs[2]);
        r = radius;
        axis[0].setValue(1,0,0);
        axis[1].setValue(0,1,0);
        axis[2].setValue(0,0,1);
    }

    /// @brief Check collision between two SPHERE
    bool overlap(const SPHERE& other) const
    {
        Vec3f  d = Tr - other.Tr;
        if (d.data.vs[0] * d.data.vs[0] + d.data.vs[1] * d.data.vs[1] + d.data.vs[2] * d.data.vs[2] > r * r)
            return false;

        return true;
    }

    /// @brief Check collision between two SPHERE and return the overlap part.
    /// For SPHERE, we return nothing, as the overlap part of two SPHEREs usually is not a SPHERE.
    bool overlap(const SPHERE& other, SPHERE& overlap_part) const
    {
        return overlap(other);
    }

    /// @brief Check whether the SPHERE contains a point
    inline bool contain(const Vec3f& p) const
    {
        Vec3f  d = Tr - p;
        if (d.data.vs[0] * d.data.vs[0] + d.data.vs[1] * d.data.vs[1] + d.data.vs[2] * d.data.vs[2] > r * r)
            return false;
        return true;
    }

    /// @brief A simple way to merge the SPHERE and a point, not compact.
    SPHERE& operator += (const Vec3f& p);

    /// @brief Merge the SPHERE and another SPHERE
    SPHERE& operator += (const SPHERE& other);

    /// @brief Return the merged SPHERE of current SPHERE and the other one
    SPHERE operator + (const SPHERE& other) const
    {
        SPHERE res(*this);
        return res += other;
    }

    /// @brief Width of the SPHERE
    inline FCL_REAL width() const
    {
        return 2 * r;
    }

    /// @brief Height of the SPHERE
    inline FCL_REAL height() const
    {
        return 2 * r;
    }

    /// @brief Depth of the SPHERE
    inline FCL_REAL depth() const
    {
        return 2 * r;
    }
    
    /// @brief Volume of the SPHERE
    inline FCL_REAL volume() const
    {
        return (4/3 * M_PI * r * r * r);
    }

    /// @brief Size of the SPHERE (used in BV_Splitter to order two SPHEREs)
    inline FCL_REAL size() const
    {
        return (2 * r);
    }

    /// @brief The RSS center
    inline const Vec3f& center() const
    {
        return Tr;
    }

    /// @brief the distance between two SPHERE; P and Q, if not NULL, return the nearest points
    FCL_REAL distance(const SPHERE& other, Vec3f* P = NULL, Vec3f* Q = NULL) const;

};

/// @brief distance between two SPHERE bounding volumes
/// P and Q (optional return values) are the closest points in the rectangles, not the SPHERE. But the direction P - Q is the correct direction for cloest points
/// Notice that P and Q are both in the local frame of the first SPHERE (not global frame and not even the local frame of object 1)
FCL_REAL distance(const Vec3f& T0, const SPHERE& b1, const SPHERE& b2, Vec3f* P = NULL, Vec3f* Q = NULL);


/// @brief Check collision between two SPHEREs, b1 is in configuration (R0, T0) and b2 is in identity.
bool overlap(const Vec3f& T0, const SPHERE& b1, const SPHERE& b2);

/// @brief Translate the SPHERE bv
SPHERE translate(const SPHERE& bv, const Vec3f& t);

}

#endif // FCL_SPHERE
