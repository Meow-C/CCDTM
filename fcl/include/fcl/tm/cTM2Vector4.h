#ifndef SRC_CTM2VECTOR4_H
#define SRC_CTM2VECTOR4_H

#include "fcl/tm/cTaylorModel2.h"
#include "fcl/tm/cTM2Matrix44.h"

class cTM2Vector4 {
public:
    cTaylorModel2 P[4];

    cTM2Vector4();
    cTM2Vector4(cTaylorModel2 p0, cTaylorModel2 p1, cTaylorModel2 p2, cTaylorModel2 p3);
    cTM2Vector4(double p0, double p1, double p2, double p3);
    cTM2Vector4(double p[4]);
};

inline cTM2Vector4 M44multV14(cTM2Matrix44 &M, cTM2Vector4 &V)
{
    cTM2Vector4 res_V;
    res_V.P[0] = M.M[0][0] * V.P[0] + M.M[0][1] * V.P[1] + M.M[0][2] * V.P[2] + M.M[0][3] * V.P[3];
    res_V.P[1] = M.M[1][0] * V.P[0] + M.M[1][1] * V.P[1] + M.M[1][2] * V.P[2] + M.M[1][3] * V.P[3];
    res_V.P[2] = M.M[2][0] * V.P[0] + M.M[2][1] * V.P[1] + M.M[2][2] * V.P[2] + M.M[2][3] * V.P[3];
    res_V.P[3] = M.M[3][0] * V.P[0] + M.M[3][1] * V.P[1] + M.M[3][2] * V.P[2] + M.M[3][3] * V.P[3];
    return res_V;
}

// calculate the distance
inline cTaylorModel2 distance_square_cal(cTM2Vector4& p, cTM2Vector4& q)
{
    cTaylorModel2 x = p.P[0] - q.P[0];
    cTaylorModel2 y = p.P[1] - q.P[1];
    cTaylorModel2 z = p.P[2] - q.P[2];

    cTaylorModel2 dis_square = x * x + y * y + z * z;

    return dis_square;
}

#endif //SRC_CTM2VECTOR4_H
