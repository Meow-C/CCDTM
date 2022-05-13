#ifndef __CTM2MATRIX33_HPP__
#define __CTM2MATRIX33_HPP__

#include <math.h>
#include <iostream>

#include "cTM2Vector3.h"
#include "cMatrix33.h"
#include "cIAMatrix33.h"
#include "cTaylorModel2.h"

// The class cTM2Matrix implements a 3x4 3rd-order Taylor model matrix

class cTM2Matrix33 {

public:

    cTaylorModel2			i[3][3];


	cTM2Matrix33();
	cTM2Matrix33(cTaylorModel2 tm[3][3]);
	cTM2Matrix33(cMatrix33&);

	cTM2Vector3				getE1();
	cTM2Vector3				getE2();
	cTM2Vector3				getE3();

	// *******************************
	// Interval arithmetics operations
	// *******************************

	cTM2Vector3				operator*(cVector3& v);
	cTM2Vector3				operator*(cTM2Vector3& v);
	cTM2Matrix33			operator*(cMatrix33& mat);
	cTM2Matrix33			operator*(cTM2Matrix33& mat);
	cTM2Matrix33			operator+(cTM2Matrix33& mat);
	void					operator+=(cTM2Matrix33& mat);

	// ***********************
	// Non interval operations
	// ***********************

	cIAMatrix33				bound();
	void					print();
	void					setIdentity();
	void					setZero();
	double					diameter();

};

cTM2Matrix33 operator*(cTaylorModel2&, cTM2Matrix33&);

#endif // __cTM2Matrix_HPP__
