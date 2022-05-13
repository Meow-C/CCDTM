#ifndef __CTM2VECTOR3_HPP__
#define __CTM2VECTOR3_HPP__

#include <math.h>

#include "cTaylorModel2.h"
#include "cVector3.h"

class cTM2Vector3 {
public:

	cTaylorModel2	i[3];

	cTM2Vector3();
	cTM2Vector3(cTaylorModel2 val[3]);
	cTM2Vector3(cTaylorModel2& nx, cTaylorModel2& ny, cTaylorModel2& nz);
	cTM2Vector3(cVector3& u);

	cTM2Vector3		operator+(const cTM2Vector3& u);
	cTM2Vector3		operator+(const double& d);
	cTM2Vector3& operator+=(const cTM2Vector3& u);
	cTM2Vector3		operator-(const cTM2Vector3& u);
	cTM2Vector3& operator-=(const cTM2Vector3& u);
	cTM2Vector3& operator=(const cVector3& u);
	cTaylorModel2	operator|(const cTM2Vector3& u);
	cTM2Vector3		operator^(const cTM2Vector3& u);

	cIAVector3		bound();
	void			linearModel(cVector3& position, cVector3& velocity);
	void			print();
	double			volume();
	void			setZero();
	cTaylorModel2	norm2() { return (i[0] * i[0]) + (i[1] * i[1]) + (i[2] * i[2]); }
	cIAVector3		evaluate(double t);
	cTaylorModel2   dot(cTM2Vector3 other);

};

cTM2Vector3 operator*(cTaylorModel2&, cTM2Vector3&);
cTM2Vector3 operator*(double, cTM2Vector3&);

#endif // __cTM2Vector3_HPP__
