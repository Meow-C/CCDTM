#include "fcl/tm/cTaylorModel2.h"
#include "fcl/tm/cTM2Matrix44.h"
#include <iostream>
#include <math.h>
#include <memory.h>

cTM2Matrix44::cTM2Matrix44() { ; }
cTM2Matrix44::cTM2Matrix44(cTaylorModel2& a00, cTaylorModel2& a01, cTaylorModel2& a02, cTaylorModel2& a03,
                           cTaylorModel2& a10, cTaylorModel2& a11, cTaylorModel2& a12, cTaylorModel2& a13,
                           cTaylorModel2& a20, cTaylorModel2& a21, cTaylorModel2& a22, cTaylorModel2& a23,
                           cTaylorModel2& a30, cTaylorModel2& a31, cTaylorModel2& a32, cTaylorModel2& a33) {
    M[0][0] = a00;
    M[0][1] = a01;
    M[0][2] = a02;
    M[0][3] = a03;
    M[1][0] = a10;
    M[1][1] = a11;
    M[1][2] = a12;
    M[1][3] = a13;
    M[2][0] = a20;
    M[2][1] = a21;
    M[2][2] = a22;
    M[2][3] = a23;
    M[3][0] = a30;
    M[3][1] = a31;
    M[3][2] = a32;
    M[3][3] = a33;
}
cTM2Matrix44::cTM2Matrix44(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13, double a20, double a21, double a22, double a23, double a30, double a31, double a32, double a33) {
    cTaylorModel2 b00(a00);
    cTaylorModel2 b01(a01);
    cTaylorModel2 b02(a02);
    cTaylorModel2 b03(a03);
    cTaylorModel2 b10(a10);
    cTaylorModel2 b11(a11);
    cTaylorModel2 b12(a12);
    cTaylorModel2 b13(a13);
    cTaylorModel2 b20(a20);
    cTaylorModel2 b21(a21);
    cTaylorModel2 b22(a22);
    cTaylorModel2 b23(a23);
    cTaylorModel2 b30(a30);
    cTaylorModel2 b31(a31);
    cTaylorModel2 b32(a32);
    cTaylorModel2 b33(a33);
    M[0][0] = b00;
    M[0][1] = b01;
    M[0][2] = b02;
    M[0][3] = b03;
    M[1][0] = b10;
    M[1][1] = b11;
    M[1][2] = b12;
    M[1][3] = b13;
    M[2][0] = b20;
    M[2][1] = b21;
    M[2][2] = b22;
    M[2][3] = b23;
    M[3][0] = b30;
    M[3][1] = b31;
    M[3][2] = b32;
    M[3][3] = b33;
}
cTM2Matrix44::cTM2Matrix44(cMatrix33& a, double xyz[3]) {
    cTaylorModel2 b00(a.m[0][0]);
    cTaylorModel2 b01(a.m[0][1]);
    cTaylorModel2 b02(a.m[0][2]);
    cTaylorModel2 b03(xyz[0]);
    cTaylorModel2 b10(a.m[1][0]);
    cTaylorModel2 b11(a.m[1][1]);
    cTaylorModel2 b12(a.m[1][2]);
    cTaylorModel2 b13(xyz[1]);
    cTaylorModel2 b20(a.m[2][0]);
    cTaylorModel2 b21(a.m[2][1]);
    cTaylorModel2 b22(a.m[2][2]);
    cTaylorModel2 b23(xyz[2]);
    cTaylorModel2 b30(0);
    cTaylorModel2 b31(0);
    cTaylorModel2 b32(0);
    cTaylorModel2 b33(1);
    M[0][0] = b00;
    M[0][1] = b01;
    M[0][2] = b02;
    M[0][3] = b03;
    M[1][0] = b10;
    M[1][1] = b11;
    M[1][2] = b12;
    M[1][3] = b13;
    M[2][0] = b20;
    M[2][1] = b21;
    M[2][2] = b22;
    M[2][3] = b23;
    M[3][0] = b30;
    M[3][1] = b31;
    M[3][2] = b32;
    M[3][3] = b33;
}

cTM2Matrix44 cTM2Matrix44::operator*(const cTM2Matrix44& in){
    register cTaylorModel2 c00, c01, c02, c03, c10, c11, c12, c13, c20, c21, c22, c23, c30, c31, c32, c33;
    c00 = M[0][0] * in.M[0][0] + M[0][1] * in.M[1][0] + M[0][2] * in.M[2][0] + M[0][3] * in.M[3][0];
    c01 = M[0][0] * in.M[0][1] + M[0][1] * in.M[1][1] + M[0][2] * in.M[2][1] + M[0][3] * in.M[3][1];
    c02 = M[0][0] * in.M[0][2] + M[0][1] * in.M[1][2] + M[0][2] * in.M[2][2] + M[0][3] * in.M[3][2];
    c03 = M[0][0] * in.M[0][3] + M[0][1] * in.M[1][3] + M[0][2] * in.M[2][3] + M[0][3] * in.M[3][3];
    c10 = M[1][0] * in.M[0][0] + M[1][1] * in.M[1][0] + M[1][2] * in.M[2][0] + M[1][3] * in.M[3][0];
    c11 = M[1][0] * in.M[0][1] + M[1][1] * in.M[1][1] + M[1][2] * in.M[2][1] + M[1][3] * in.M[3][1];
    c12 = M[1][0] * in.M[0][2] + M[1][1] * in.M[1][2] + M[1][2] * in.M[2][2] + M[1][3] * in.M[3][2];
    c13 = M[1][0] * in.M[0][3] + M[1][1] * in.M[1][3] + M[1][2] * in.M[2][3] + M[1][3] * in.M[3][3];
    c20 = M[2][0] * in.M[0][0] + M[2][1] * in.M[1][0] + M[2][2] * in.M[2][0] + M[2][3] * in.M[3][0];
    c21 = M[2][0] * in.M[0][1] + M[2][1] * in.M[1][1] + M[2][2] * in.M[2][1] + M[2][3] * in.M[3][1];
    c22 = M[2][0] * in.M[0][2] + M[2][1] * in.M[1][2] + M[2][2] * in.M[2][2] + M[2][3] * in.M[3][2];
    c23 = M[2][0] * in.M[0][3] + M[2][1] * in.M[1][3] + M[2][2] * in.M[2][3] + M[2][3] * in.M[3][3];
    c30 = M[3][0] * in.M[0][0] + M[3][1] * in.M[1][0] + M[3][2] * in.M[2][0] + M[3][3] * in.M[3][0];
    c31 = M[3][0] * in.M[0][1] + M[3][1] * in.M[1][1] + M[3][2] * in.M[2][1] + M[3][3] * in.M[3][1];
    c32 = M[3][0] * in.M[0][2] + M[3][1] * in.M[1][2] + M[3][2] * in.M[2][2] + M[3][3] * in.M[3][2];
    c33 = M[3][0] * in.M[0][3] + M[3][1] * in.M[1][3] + M[3][2] * in.M[2][3] + M[3][3] * in.M[3][3];
    return cTM2Matrix44(c00, c01, c02, c03, c10, c11, c12, c13, c20, c21, c22, c23, c30, c31, c32, c33);
}