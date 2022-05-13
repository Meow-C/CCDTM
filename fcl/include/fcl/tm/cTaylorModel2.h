#ifndef __CTAYLORMODEL2_HPP__
#define __CTAYLORMODEL2_HPP__

#define CTM3_PI 3.1415926535897932
#define CTM3_HALF_PI 1.57079632679489661

#include "cInterval.h"
#include "cMatrix33.h"

// The class cTaylorModel2 implements a third order Taylor model,
// i.e. a cubic approximation of a function over a time interval, with an
// interval remainder NB: operations on two Taylor models assume that the time
// interval is the same

class cTaylorModel2 {
public:
	double c[3];  // the coefficients of the cubic
	cInterval r;  // the remainder

	cTaylorModel2();
	cTaylorModel2(double v);
	cTaylorModel2(double cc[], cInterval& rr);
	cTaylorModel2(double c0, double c1, double c2, cInterval rr);

	cTaylorModel2 operator+(const cTaylorModel2& in);
	cTaylorModel2 operator-(const cTaylorModel2& in);
	cTaylorModel2& operator+=(const cTaylorModel2& in);
	cTaylorModel2& operator-=(const cTaylorModel2& in);
	cTaylorModel2 operator*(const cTaylorModel2& in);
	cTaylorModel2 operator*(const double d);
	cTaylorModel2 operator-();

	void print();
	cInterval bound();
	cInterval bound(double l, double r);
	cInterval evaluate(double t);
	void setZero();

	void cosModel(double w, double q0);
	void sinModel(double w, double q0);
	void linearModel(double p, double v);
};

cTaylorModel2 operator*(double d, cTaylorModel2& tm);

void setGlobalTimeInterval(double l, double r);
cInterval getGlobalTimeInterval();

#endif  // __CTAYLORMODEL3_HPP__