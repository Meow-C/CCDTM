#ifndef __CTM2MATRIX44_HPP__
#define __CTM2MATRIX44_HPP__

#define CTM3_PI 3.1415926535897932
#define CTM3_HALF_PI 1.57079632679489661

#include "cInterval.h"
#include "cMatrix33.h"
#include "cTaylorModel2.h"

class cTM2Matrix44 {
public:
    cTaylorModel2 M[4][4];

    cTM2Matrix44();
    cTM2Matrix44(cTaylorModel2& a00, cTaylorModel2& a01, cTaylorModel2& a02, cTaylorModel2& a03,
                 cTaylorModel2& a10, cTaylorModel2& a11, cTaylorModel2& a12, cTaylorModel2& a13,
                 cTaylorModel2& a20, cTaylorModel2& a21, cTaylorModel2& a22, cTaylorModel2& a23,
                 cTaylorModel2& a30, cTaylorModel2& a31, cTaylorModel2& a32, cTaylorModel2& a33);
    cTM2Matrix44(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13, double a20, double a21, double a22, double a23, double a30, double a31, double a32, double a33);
    cTM2Matrix44(cMatrix33& a, double xyz[3]);

    cTM2Matrix44 operator*(const cTM2Matrix44& in);

    void init()
    {
        cTaylorModel2 zero;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                M[i][j] = zero;
            }
        }
    }
};


#endif  // __CTM2MATRIX44_HPP__