#include "fcl/tm/cTM2Vector4.h"

cTM2Vector4::cTM2Vector4() { ; }
cTM2Vector4::cTM2Vector4(cTaylorModel2 p0, cTaylorModel2 p1, cTaylorModel2 p2, cTaylorModel2 p3) {
    P[0] = p0;
    P[1] = p1;
    P[2] = p2;
    P[3] = p3;
}
cTM2Vector4::cTM2Vector4(double p0, double p1, double p2, double p3) {
    P[0] = p0;
    P[1] = p1;
    P[2] = p2;
    P[3] = p3;
}

cTM2Vector4::cTM2Vector4(double p[4]){
    P[0] = p[0];
    P[1] = p[1];
    P[2] = p[2];
    P[3] = p[3];
}

