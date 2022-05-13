#ifndef __CTM3MATRIX33_HPP__
#define __CTM3MATRIX33_HPP__

#include <math.h>
#include <iostream>

#include "cTM3Vector3.h"
#include "cMatrix33.h"
#include "cIAMatrix33.h"
#include "cTaylorModel3.h"

// The class cTM3Matrix33 implements a 3x4 3rd-order Taylor model matrix

class cTM3Matrix33 {

public:

    cTaylorModel3			i[3][3];


	cTM3Matrix33();
	cTM3Matrix33(cTaylorModel3 tm[3][3]);
	cTM3Matrix33(cMatrix33&);

	cTM3Vector3				getE1();
	cTM3Vector3				getE2();
	cTM3Vector3				getE3();

	// *******************************
	// Interval arithmetics operations
	// *******************************

	cTM3Vector3				operator*(cVector3& v);
	cTM3Vector3				operator*(cTM3Vector3& v);
	cTM3Matrix33			operator*(cMatrix33& mat);
	cTM3Matrix33			operator*(cTM3Matrix33& mat);
	cTM3Matrix33			operator+(cTM3Matrix33& mat);
	void					operator+=(cTM3Matrix33& mat);

	// ***********************
	// Non interval operations
	// ***********************

	cIAMatrix33				bound();
	void					print();
	void					setIdentity();
	void					setZero();
	double					diameter();

};

cTM3Matrix33 operator*(cTaylorModel3&, cTM3Matrix33&);

#endif // __CTM3MATRIX33_HPP__
