#ifndef __CTAYLORMODEL3_HPP__
#define __CTAYLORMODEL3_HPP__

#define CTM3_PI 3.1415926535897932
#define CTM3_HALF_PI 1.57079632679489661

#include "cInterval.h"
#include "cMatrix33.h"

// The class cTaylorModel3 implements a third order Taylor model,
// i.e. a cubic approximation of a function over a time interval, with an
// interval remainder NB: operations on two Taylor models assume that the time
// interval is the same

extern cInterval globalTimeInterval;
extern cInterval globalTimeInterval2;
extern cInterval globalTimeInterval3;
extern cInterval globalTimeInterval4;
extern cInterval globalTimeInterval5;
extern cInterval globalTimeInterval6;

class cTaylorModel3 {
public:
	double c[4];  // the coefficients of the cubic
	cInterval r;  // the remainder

	cTaylorModel3();
	cTaylorModel3(double v);
	cTaylorModel3(double cc[], cInterval& rr);
	cTaylorModel3(double c0, double c1, double c2, double c3, cInterval rr);

	cTaylorModel3 operator+(const cTaylorModel3& in);
	cTaylorModel3 operator-(const cTaylorModel3& in);
	cTaylorModel3& operator+=(const cTaylorModel3& in);
	cTaylorModel3& operator-=(const cTaylorModel3& in);
	cTaylorModel3 operator*(const cTaylorModel3& in);
	cTaylorModel3 operator*(const double d);
	cTaylorModel3 operator-();
	//cTaylorModel3 operator_square_root();

	void print();
	cInterval bound();
	cInterval bound(double l, double r);
	cInterval evaluate(double t);
	void setZero();

	void cosModel(double w, double q0);
	void sinModel(double w, double q0);
	void cospolytModel(double a0, double a1, double a2, double a3);	// directly Taylor Model
	void sinpolytModel(double a0, double a1, double a2, double a3);
	//void cospolyt(double a0, double a1, double a2, double a3, double t0);	// expression then Taylor Model, including offset
	//void sinpolyt(double a0, double a1, double a2, double a3, double t0);
	//void cosat2(double w);
	//void sinat2(double w);
	//void cosat3(double w);
	//void sinat3(double w);
	void linearModel(double p, double v);
};

cTaylorModel3 operator*(double d, cTaylorModel3& tm);

class cTM3Matrix44 {
public:
	cTaylorModel3 M[4][4];

	cTM3Matrix44();
	cTM3Matrix44(cTaylorModel3& a00, cTaylorModel3& a01, cTaylorModel3& a02, cTaylorModel3& a03,
                 cTaylorModel3& a10, cTaylorModel3& a11, cTaylorModel3& a12, cTaylorModel3& a13,
                 cTaylorModel3& a20, cTaylorModel3& a21, cTaylorModel3& a22, cTaylorModel3& a23,
                 cTaylorModel3& a30, cTaylorModel3& a31, cTaylorModel3& a32, cTaylorModel3& a33);
	cTM3Matrix44(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13, double a20, double a21, double a22, double a23, double a30, double a31, double a32, double a33);
	cTM3Matrix44(cMatrix33& a, double xyz[3]);
//	cTM3Matrix44(cTM3Matrix33& a, cTM3Vector3& t);

	cTM3Matrix44 operator*(const cTM3Matrix44& in);

	void init()
    {
	    cTaylorModel3 zero;
	    for (int i = 0; i < 4; i++) {
	        for (int j = 0; j < 4; j++) {
	            M[i][j] = zero;
	        }
	    }
    }
};

class cTMVector4 {
public:
	cTaylorModel3 P[4];
	
	cTMVector4();
	cTMVector4(cTaylorModel3 p0, cTaylorModel3 p1, cTaylorModel3 p2, cTaylorModel3 p3);
	cTMVector4(double p0, double p1, double p2, double p3);
	cTMVector4(double p[4]);
};

void setGlobalTimeInterval(double l, double r);
cInterval getGlobalTimeInterval();

inline cTMVector4 M44multV14(cTM3Matrix44 &M, cTMVector4 &V)
{
    register cTMVector4 res_V;
    res_V.P[0] = M.M[0][0] * V.P[0] + M.M[0][1] * V.P[1] + M.M[0][2] * V.P[2] + M.M[0][3] * V.P[3];
    res_V.P[1] = M.M[1][0] * V.P[0] + M.M[1][1] * V.P[1] + M.M[1][2] * V.P[2] + M.M[1][3] * V.P[3];
    res_V.P[2] = M.M[2][0] * V.P[0] + M.M[2][1] * V.P[1] + M.M[2][2] * V.P[2] + M.M[2][3] * V.P[3];
    res_V.P[3] = M.M[3][0] * V.P[0] + M.M[3][1] * V.P[1] + M.M[3][2] * V.P[2] + M.M[3][3] * V.P[3];
    return res_V;
}

// calculate the distance
inline cTaylorModel3 distance_square_cal(cTMVector4& p, cTMVector4& q)
{
//    register cTaylorModel3 x = p.P[0] - q.P[0];
//    register cTaylorModel3 y = p.P[1] - q.P[1];
//    register cTaylorModel3 z = p.P[2] - q.P[2];
//
//    register cTaylorModel3 dis_square = x * x + y * y + z * z;
//
//    return dis_square;
    return (p.P[0] - q.P[0])*(p.P[0] - q.P[0]) + (p.P[1] - q.P[1])*(p.P[1] - q.P[1]) + (p.P[2] - q.P[2])*(p.P[2] - q.P[2]);
}


#endif  // __CTAYLORMODEL3_HPP__