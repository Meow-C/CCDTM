#include <iostream>
#include <memory.h>

#include "fcl/tm/cTM2Matrix33.h"
#include "fcl/tm/cIAMatrix33.h"
#include "fcl/tm/cTM2Vector3.h"
#include "fcl/tm/cMatrix33.h"

using namespace std;

cTM2Matrix33::cTM2Matrix33() { }
cTM2Matrix33::cTM2Matrix33(cTaylorModel2 mat[3][3]) { memcpy(this, mat, 9 * sizeof(cTaylorModel2)); }
cTM2Matrix33::cTM2Matrix33(cMatrix33& mat) {

	memset(this, 0, sizeof(cTM2Matrix33));
	i[0][0].c[0] = mat.m[0][0]; i[0][1].c[0] = mat.m[0][1]; i[0][2].c[0] = mat.m[0][2];
	i[1][0].c[0] = mat.m[1][0]; i[1][1].c[0] = mat.m[1][1]; i[1][2].c[0] = mat.m[1][2];
	i[2][0].c[0] = mat.m[2][0]; i[2][1].c[0] = mat.m[2][1]; i[2][2].c[0] = mat.m[2][2];

}

void cTM2Matrix33::setIdentity() {

	memset(this, 0, sizeof(cTM2Matrix33));
	i[0][0].c[0] = i[1][1].c[0] = i[2][2].c[0] = 1.0;

}

void cTM2Matrix33::setZero() {

	memset(this, 0, sizeof(cTM2Matrix33));

}

cTM2Vector3 cTM2Matrix33::getE1() { return cTM2Vector3(i[0][0], i[1][0], i[2][0]); }
cTM2Vector3 cTM2Matrix33::getE2() { return cTM2Vector3(i[0][1], i[1][1], i[2][1]); }
cTM2Vector3 cTM2Matrix33::getE3() { return cTM2Vector3(i[0][2], i[1][2], i[2][2]); }

cTM2Matrix33 cTM2Matrix33::operator*(cMatrix33& mat) {

	// Interval method to multiply an interval matrix by a real matrix

	cTaylorModel2 res[3][3];
	register double temp;

	temp = mat.m[0][0];
	res[0][0].c[0] = i[0][0].c[0] * temp; res[0][0].c[1] = i[0][0].c[1] * temp; res[0][0].c[2] = i[0][0].c[2] * temp; res[0][0].c[3] = i[0][0].c[3] * temp; res[1][0].c[0] = i[1][0].c[0] * temp; res[1][0].c[1] = i[1][0].c[1] * temp; res[1][0].c[2] = i[1][0].c[2] * temp; res[1][0].c[3] = i[1][0].c[3] * temp; res[2][0].c[0] = i[2][0].c[0] * temp; res[2][0].c[1] = i[2][0].c[1] * temp; res[2][0].c[2] = i[2][0].c[2] * temp; res[2][0].c[3] = i[2][0].c[3] * temp;
	if (temp >= 0) { res[0][0].r.i[0] = i[0][0].r.i[0] * temp; res[0][0].r.i[1] = i[0][0].r.i[1] * temp; res[1][0].r.i[0] = i[1][0].r.i[0] * temp; res[1][0].r.i[1] = i[1][0].r.i[1] * temp; res[2][0].r.i[0] = i[2][0].r.i[0] * temp; res[2][0].r.i[1] = i[2][0].r.i[1] * temp; }
	else { res[0][0].r.i[0] = i[0][0].r.i[1] * temp; res[0][0].r.i[1] = i[0][0].r.i[0] * temp; res[1][0].r.i[0] = i[1][0].r.i[1] * temp; res[1][0].r.i[1] = i[1][0].r.i[0] * temp; res[2][0].r.i[0] = i[2][0].r.i[1] * temp; res[2][0].r.i[1] = i[2][0].r.i[0] * temp; }

	temp = mat.m[0][1];
	res[0][1].c[0] = i[0][0].c[0] * temp; res[0][1].c[1] = i[0][0].c[1] * temp; res[0][1].c[2] = i[0][0].c[2] * temp; res[0][1].c[3] = i[0][0].c[3] * temp; res[1][1].c[0] = i[1][0].c[0] * temp; res[1][1].c[1] = i[1][0].c[1] * temp; res[1][1].c[2] = i[1][0].c[2] * temp; res[1][1].c[3] = i[1][0].c[3] * temp; res[2][1].c[0] = i[2][0].c[0] * temp; res[2][1].c[1] = i[2][0].c[1] * temp; res[2][1].c[2] = i[2][0].c[2] * temp; res[2][1].c[3] = i[2][0].c[3] * temp;
	if (temp >= 0) { res[0][1].r.i[0] = i[0][0].r.i[0] * temp; res[0][1].r.i[1] = i[0][0].r.i[1] * temp; res[1][1].r.i[0] = i[1][0].r.i[0] * temp; res[1][1].r.i[1] = i[1][0].r.i[1] * temp; res[2][1].r.i[0] = i[2][0].r.i[0] * temp; res[2][1].r.i[1] = i[2][0].r.i[1] * temp; }
	else { res[0][1].r.i[0] = i[0][0].r.i[1] * temp; res[0][1].r.i[1] = i[0][0].r.i[0] * temp; res[1][1].r.i[0] = i[1][0].r.i[1] * temp; res[1][1].r.i[1] = i[1][0].r.i[0] * temp; res[2][1].r.i[0] = i[2][0].r.i[1] * temp; res[2][1].r.i[1] = i[2][0].r.i[0] * temp; }

	temp = mat.m[0][2];
	res[0][2].c[0] = i[0][0].c[0] * temp; res[0][2].c[1] = i[0][0].c[1] * temp; res[0][2].c[2] = i[0][0].c[2] * temp; res[0][2].c[3] = i[0][0].c[3] * temp; res[1][2].c[0] = i[1][0].c[0] * temp; res[1][2].c[1] = i[1][0].c[1] * temp; res[1][2].c[2] = i[1][0].c[2] * temp; res[1][2].c[3] = i[1][0].c[3] * temp; res[2][2].c[0] = i[2][0].c[0] * temp; res[2][2].c[1] = i[2][0].c[1] * temp; res[2][2].c[2] = i[2][0].c[2] * temp; res[2][2].c[3] = i[2][0].c[3] * temp;
	if (temp >= 0) { res[0][2].r.i[0] = i[0][0].r.i[0] * temp; res[0][2].r.i[1] = i[0][0].r.i[1] * temp; res[1][2].r.i[0] = i[1][0].r.i[0] * temp; res[1][2].r.i[1] = i[1][0].r.i[1] * temp; res[2][2].r.i[0] = i[2][0].r.i[0] * temp; res[2][2].r.i[1] = i[2][0].r.i[1] * temp; }
	else { res[0][2].r.i[0] = i[0][0].r.i[1] * temp; res[0][2].r.i[1] = i[0][0].r.i[0] * temp; res[1][2].r.i[0] = i[1][0].r.i[1] * temp; res[1][2].r.i[1] = i[1][0].r.i[0] * temp; res[2][2].r.i[0] = i[2][0].r.i[1] * temp; res[2][2].r.i[1] = i[2][0].r.i[0] * temp; }

	temp = mat.m[1][0];
	res[0][0].c[0] += i[0][1].c[0] * temp; res[0][0].c[1] += i[0][1].c[1] * temp; res[0][0].c[2] += i[0][1].c[2] * temp; res[0][0].c[3] += i[0][1].c[3] * temp; res[1][0].c[0] += i[1][1].c[0] * temp; res[1][0].c[1] += i[1][1].c[1] * temp; res[1][0].c[2] += i[1][1].c[2] * temp; res[1][0].c[3] += i[1][1].c[3] * temp; res[2][0].c[0] += i[2][1].c[0] * temp; res[2][0].c[1] += i[2][1].c[1] * temp; res[2][0].c[2] += i[2][1].c[2] * temp; res[2][0].c[3] += i[2][1].c[3] * temp;
	if (temp >= 0) { res[0][0].r.i[0] += i[0][1].r.i[0] * temp; res[0][0].r.i[1] += i[0][1].r.i[1] * temp; res[1][0].r.i[0] += i[1][1].r.i[0] * temp; res[1][0].r.i[1] += i[1][1].r.i[1] * temp; res[2][0].r.i[0] += i[2][1].r.i[0] * temp; res[2][0].r.i[1] += i[2][1].r.i[1] * temp; }
	else { res[0][0].r.i[0] += i[0][1].r.i[1] * temp; res[0][0].r.i[1] += i[0][1].r.i[0] * temp; res[1][0].r.i[0] += i[1][1].r.i[1] * temp; res[1][0].r.i[1] += i[1][1].r.i[0] * temp; res[2][0].r.i[0] += i[2][1].r.i[1] * temp; res[2][0].r.i[1] += i[2][1].r.i[0] * temp; }

	temp = mat.m[1][1];
	res[0][1].c[0] += i[0][1].c[0] * temp; res[0][1].c[1] += i[0][1].c[1] * temp; res[0][1].c[2] += i[0][1].c[2] * temp; res[0][1].c[3] += i[0][1].c[3] * temp; res[1][1].c[0] += i[1][1].c[0] * temp; res[1][1].c[1] += i[1][1].c[1] * temp; res[1][1].c[2] += i[1][1].c[2] * temp; res[1][1].c[3] += i[1][1].c[3] * temp; res[2][1].c[0] += i[2][1].c[0] * temp; res[2][1].c[1] += i[2][1].c[1] * temp; res[2][1].c[2] += i[2][1].c[2] * temp; res[2][1].c[3] += i[2][1].c[3] * temp;
	if (temp >= 0) { res[0][1].r.i[0] += i[0][1].r.i[0] * temp; res[0][1].r.i[1] += i[0][1].r.i[1] * temp; res[1][1].r.i[0] += i[1][1].r.i[0] * temp; res[1][1].r.i[1] += i[1][1].r.i[1] * temp; res[2][1].r.i[0] += i[2][1].r.i[0] * temp; res[2][1].r.i[1] += i[2][1].r.i[1] * temp; }
	else { res[0][1].r.i[0] += i[0][1].r.i[1] * temp; res[0][1].r.i[1] += i[0][1].r.i[0] * temp; res[1][1].r.i[0] += i[1][1].r.i[1] * temp; res[1][1].r.i[1] += i[1][1].r.i[0] * temp; res[2][1].r.i[0] += i[2][1].r.i[1] * temp; res[2][1].r.i[1] += i[2][1].r.i[0] * temp; }

	temp = mat.m[1][2];
	res[0][2].c[0] += i[0][1].c[0] * temp; res[0][2].c[1] += i[0][1].c[1] * temp; res[0][2].c[2] += i[0][1].c[2] * temp; res[0][2].c[3] += i[0][1].c[3] * temp; res[1][2].c[0] += i[1][1].c[0] * temp; res[1][2].c[1] += i[1][1].c[1] * temp; res[1][2].c[2] += i[1][1].c[2] * temp; res[1][2].c[3] += i[1][1].c[3] * temp; res[2][2].c[0] += i[2][1].c[0] * temp; res[2][2].c[1] += i[2][1].c[1] * temp; res[2][2].c[2] += i[2][1].c[2] * temp; res[2][2].c[3] += i[2][1].c[3] * temp;
	if (temp >= 0) { res[0][2].r.i[0] += i[0][1].r.i[0] * temp; res[0][2].r.i[1] += i[0][1].r.i[1] * temp; res[1][2].r.i[0] += i[1][1].r.i[0] * temp; res[1][2].r.i[1] += i[1][1].r.i[1] * temp; res[2][2].r.i[0] += i[2][1].r.i[0] * temp; res[2][2].r.i[1] += i[2][1].r.i[1] * temp; }
	else { res[0][2].r.i[0] += i[0][1].r.i[1] * temp; res[0][2].r.i[1] += i[0][1].r.i[0] * temp; res[1][2].r.i[0] += i[1][1].r.i[1] * temp; res[1][2].r.i[1] += i[1][1].r.i[0] * temp; res[2][2].r.i[0] += i[2][1].r.i[1] * temp; res[2][2].r.i[1] += i[2][1].r.i[0] * temp; }

	temp = mat.m[2][0];
	res[0][0].c[0] += i[0][2].c[0] * temp; res[0][0].c[1] += i[0][2].c[1] * temp; res[0][0].c[2] += i[0][2].c[2] * temp; res[0][0].c[3] += i[0][2].c[3] * temp; res[1][0].c[0] += i[1][2].c[0] * temp; res[1][0].c[1] += i[1][2].c[1] * temp; res[1][0].c[2] += i[1][2].c[2] * temp; res[1][0].c[3] += i[1][2].c[3] * temp; res[2][0].c[0] += i[2][2].c[0] * temp; res[2][0].c[1] += i[2][2].c[1] * temp; res[2][0].c[2] += i[2][2].c[2] * temp; res[2][0].c[3] += i[2][2].c[3] * temp;
	if (temp >= 0) { res[0][0].r.i[0] += i[0][2].r.i[0] * temp; res[0][0].r.i[1] += i[0][2].r.i[1] * temp; res[1][0].r.i[0] += i[1][2].r.i[0] * temp; res[1][0].r.i[1] += i[1][2].r.i[1] * temp; res[2][0].r.i[0] += i[2][2].r.i[0] * temp; res[2][0].r.i[1] += i[2][2].r.i[1] * temp; }
	else { res[0][0].r.i[0] += i[0][2].r.i[1] * temp; res[0][0].r.i[1] += i[0][2].r.i[0] * temp; res[1][0].r.i[0] += i[1][2].r.i[1] * temp; res[1][0].r.i[1] += i[1][2].r.i[0] * temp; res[2][0].r.i[0] += i[2][2].r.i[1] * temp; res[2][0].r.i[1] += i[2][2].r.i[0] * temp; }

	temp = mat.m[2][1];
	res[0][1].c[0] += i[0][2].c[0] * temp; res[0][1].c[1] += i[0][2].c[1] * temp; res[0][1].c[2] += i[0][2].c[2] * temp; res[0][1].c[3] += i[0][2].c[3] * temp; res[1][1].c[0] += i[1][2].c[0] * temp; res[1][1].c[1] += i[1][2].c[1] * temp; res[1][1].c[2] += i[1][2].c[2] * temp; res[1][1].c[3] += i[1][2].c[3] * temp; res[2][1].c[0] += i[2][2].c[0] * temp; res[2][1].c[1] += i[2][2].c[1] * temp; res[2][1].c[2] += i[2][2].c[2] * temp; res[2][1].c[3] += i[2][2].c[3] * temp;
	if (temp >= 0) { res[0][1].r.i[0] += i[0][2].r.i[0] * temp; res[0][1].r.i[1] += i[0][2].r.i[1] * temp; res[1][1].r.i[0] += i[1][2].r.i[0] * temp; res[1][1].r.i[1] += i[1][2].r.i[1] * temp; res[2][1].r.i[0] += i[2][2].r.i[0] * temp; res[2][1].r.i[1] += i[2][2].r.i[1] * temp; }
	else { res[0][1].r.i[0] += i[0][2].r.i[1] * temp; res[0][1].r.i[1] += i[0][2].r.i[0] * temp; res[1][1].r.i[0] += i[1][2].r.i[1] * temp; res[1][1].r.i[1] += i[1][2].r.i[0] * temp; res[2][1].r.i[0] += i[2][2].r.i[1] * temp; res[2][1].r.i[1] += i[2][2].r.i[0] * temp; }

	temp = mat.m[2][2];
	res[0][2].c[0] += i[0][2].c[0] * temp; res[0][2].c[1] += i[0][2].c[1] * temp; res[0][2].c[2] += i[0][2].c[2] * temp; res[0][2].c[3] += i[0][2].c[3] * temp; res[1][2].c[0] += i[1][2].c[0] * temp; res[1][2].c[1] += i[1][2].c[1] * temp; res[1][2].c[2] += i[1][2].c[2] * temp; res[1][2].c[3] += i[1][2].c[3] * temp; res[2][2].c[0] += i[2][2].c[0] * temp; res[2][2].c[1] += i[2][2].c[1] * temp; res[2][2].c[2] += i[2][2].c[2] * temp; res[2][2].c[3] += i[2][2].c[3] * temp;
	if (temp >= 0) { res[0][2].r.i[0] += i[0][2].r.i[0] * temp; res[0][2].r.i[1] += i[0][2].r.i[1] * temp; res[1][2].r.i[0] += i[1][2].r.i[0] * temp; res[1][2].r.i[1] += i[1][2].r.i[1] * temp; res[2][2].r.i[0] += i[2][2].r.i[0] * temp; res[2][2].r.i[1] += i[2][2].r.i[1] * temp; }
	else { res[0][2].r.i[0] += i[0][2].r.i[1] * temp; res[0][2].r.i[1] += i[0][2].r.i[0] * temp; res[1][2].r.i[0] += i[1][2].r.i[1] * temp; res[1][2].r.i[1] += i[1][2].r.i[0] * temp; res[2][2].r.i[0] += i[2][2].r.i[1] * temp; res[2][2].r.i[1] += i[2][2].r.i[0] * temp; }

	return cTM2Matrix33(res);

}

cTM2Vector3 cTM2Matrix33::operator*(cVector3& v) {

	// Interval method to multiply an interval matrix by a real matrix

	cTaylorModel2 res[3];

	register double temp;

	temp = v.v[0];
	res[0].c[0] = i[0][0].c[0] * temp; res[0].c[1] = i[0][0].c[1] * temp; res[0].c[2] = i[0][0].c[2] * temp; res[0].c[3] = i[0][0].c[3] * temp; res[1].c[0] = i[1][0].c[0] * temp; res[1].c[1] = i[1][0].c[1] * temp; res[1].c[2] = i[1][0].c[2] * temp; res[1].c[3] = i[1][0].c[3] * temp; res[2].c[0] = i[2][0].c[0] * temp; res[2].c[1] = i[2][0].c[1] * temp; res[2].c[2] = i[2][0].c[2] * temp; res[2].c[3] = i[2][0].c[3] * temp;
	if (temp >= 0) { res[0].r.i[0] = i[0][0].r.i[0] * temp; res[0].r.i[1] = i[0][0].r.i[1] * temp; res[1].r.i[0] = i[1][0].r.i[0] * temp; res[1].r.i[1] = i[1][0].r.i[1] * temp; res[2].r.i[0] = i[2][0].r.i[0] * temp; res[2].r.i[1] = i[2][0].r.i[1] * temp; }
	else { res[0].r.i[0] = i[0][0].r.i[1] * temp; res[0].r.i[1] = i[0][0].r.i[0] * temp; res[1].r.i[0] = i[1][0].r.i[1] * temp; res[1].r.i[1] = i[1][0].r.i[0] * temp; res[2].r.i[0] = i[2][0].r.i[1] * temp; res[2].r.i[1] = i[2][0].r.i[0] * temp; }

	temp = v.v[1];
	res[0].c[0] += i[0][1].c[0] * temp; res[0].c[1] += i[0][1].c[1] * temp; res[0].c[2] += i[0][1].c[2] * temp; res[0].c[3] += i[0][1].c[3] * temp; res[1].c[0] += i[1][1].c[0] * temp; res[1].c[1] += i[1][1].c[1] * temp; res[1].c[2] += i[1][1].c[2] * temp; res[1].c[3] += i[1][1].c[3] * temp; res[2].c[0] += i[2][1].c[0] * temp; res[2].c[1] += i[2][1].c[1] * temp; res[2].c[2] += i[2][1].c[2] * temp; res[2].c[3] += i[2][1].c[3] * temp;
	if (temp >= 0) { res[0].r.i[0] += i[0][1].r.i[0] * temp; res[0].r.i[1] += i[0][1].r.i[1] * temp; res[1].r.i[0] += i[1][1].r.i[0] * temp; res[1].r.i[1] += i[1][1].r.i[1] * temp; res[2].r.i[0] += i[2][1].r.i[0] * temp; res[2].r.i[1] += i[2][1].r.i[1] * temp; }
	else { res[0].r.i[0] += i[0][1].r.i[1] * temp; res[0].r.i[1] += i[0][1].r.i[0] * temp; res[1].r.i[0] += i[1][1].r.i[1] * temp; res[1].r.i[1] += i[1][1].r.i[0] * temp; res[2].r.i[0] += i[2][1].r.i[1] * temp; res[2].r.i[1] += i[2][1].r.i[0] * temp; }

	temp = v.v[2];
	res[0].c[0] += i[0][2].c[0] * temp; res[0].c[1] += i[0][2].c[1] * temp; res[0].c[2] += i[0][2].c[2] * temp; res[0].c[3] += i[0][2].c[3] * temp; res[1].c[0] += i[1][2].c[0] * temp; res[1].c[1] += i[1][2].c[1] * temp; res[1].c[2] += i[1][2].c[2] * temp; res[1].c[3] += i[1][2].c[3] * temp; res[2].c[0] += i[2][2].c[0] * temp; res[2].c[1] += i[2][2].c[1] * temp; res[2].c[2] += i[2][2].c[2] * temp; res[2].c[3] += i[2][2].c[3] * temp;
	if (temp >= 0) { res[0].r.i[0] += i[0][2].r.i[0] * temp; res[0].r.i[1] += i[0][2].r.i[1] * temp; res[1].r.i[0] += i[1][2].r.i[0] * temp; res[1].r.i[1] += i[1][2].r.i[1] * temp; res[2].r.i[0] += i[2][2].r.i[0] * temp; res[2].r.i[1] += i[2][2].r.i[1] * temp; }
	else { res[0].r.i[0] += i[0][2].r.i[1] * temp; res[0].r.i[1] += i[0][2].r.i[0] * temp; res[1].r.i[0] += i[1][2].r.i[1] * temp; res[1].r.i[1] += i[1][2].r.i[0] * temp; res[2].r.i[0] += i[2][2].r.i[1] * temp; res[2].r.i[1] += i[2][2].r.i[0] * temp; }

	return cTM2Vector3(res);

}

cTM2Vector3 cTM2Matrix33::operator*(cTM2Vector3& v) {

	// Interval method : r=m*v 

	cTaylorModel2 res[3];

	res[0] = i[0][0] * v.i[0] + i[0][1] * v.i[1] + i[0][2] * v.i[2];
	res[1] = i[1][0] * v.i[0] + i[1][1] * v.i[1] + i[1][2] * v.i[2];
	res[2] = i[2][0] * v.i[0] + i[2][1] * v.i[1] + i[2][2] * v.i[2];

	return cTM2Vector3(res);

}

cTM2Matrix33 cTM2Matrix33::operator*(cTM2Matrix33& mat) {

	// Interval method : res=m*mat

	cTaylorModel2 res[3][3];

	res[0][0] = i[0][0] * mat.i[0][0] + i[0][1] * mat.i[1][0] + i[0][2] * mat.i[2][0];
	res[1][0] = i[1][0] * mat.i[0][0] + i[1][1] * mat.i[1][0] + i[1][2] * mat.i[2][0];
	res[2][0] = i[2][0] * mat.i[0][0] + i[2][1] * mat.i[1][0] + i[2][2] * mat.i[2][0];

	res[0][1] = i[0][0] * mat.i[0][1] + i[0][1] * mat.i[1][1] + i[0][2] * mat.i[2][1];
	res[1][1] = i[1][0] * mat.i[0][1] + i[1][1] * mat.i[1][1] + i[1][2] * mat.i[2][1];
	res[2][1] = i[2][0] * mat.i[0][1] + i[2][1] * mat.i[1][1] + i[2][2] * mat.i[2][1];

	res[0][2] = i[0][0] * mat.i[0][2] + i[0][1] * mat.i[1][2] + i[0][2] * mat.i[2][2];
	res[1][2] = i[1][0] * mat.i[0][2] + i[1][1] * mat.i[1][2] + i[1][2] * mat.i[2][2];
	res[2][2] = i[2][0] * mat.i[0][2] + i[2][1] * mat.i[1][2] + i[2][2] * mat.i[2][2];

	return cTM2Matrix33(res);

}

cTM2Matrix33 cTM2Matrix33::operator+(cTM2Matrix33& mat) {

	// res=m+mat

	cTaylorModel2 res[3][3];

	res[0][0] = i[0][0] + mat.i[0][0]; res[0][1] = i[0][1] + mat.i[0][1]; res[0][2] = i[0][2] + mat.i[0][2];
	res[1][0] = i[1][0] + mat.i[1][0]; res[1][1] = i[1][1] + mat.i[1][1]; res[1][2] = i[1][2] + mat.i[1][2];
	res[2][0] = i[2][0] + mat.i[2][0]; res[2][1] = i[2][1] + mat.i[2][1]; res[2][2] = i[2][2] + mat.i[2][2];

	return cTM2Matrix33(res);

}

void cTM2Matrix33::operator+=(cTM2Matrix33& mat) {

	// m=m+mat

	i[0][0] += mat.i[0][0]; i[0][1] += mat.i[0][1]; i[0][2] += mat.i[0][2];
	i[1][0] += mat.i[1][0]; i[1][1] += mat.i[1][1]; i[1][2] += mat.i[1][2];
	i[2][0] += mat.i[2][0]; i[2][1] += mat.i[2][1]; i[2][2] += mat.i[2][2];

}

void cTM2Matrix33::print() {

	cout << "E1:" << endl; getE1().print();
	cout << "E2:" << endl; getE2().print();
	cout << "E3:" << endl; getE3().print();
	cout << endl;
	cout << endl;

}

cIAMatrix33 cTM2Matrix33::bound() {

	cIAMatrix33 res;
	res.i[0][0] = i[0][0].bound(); res.i[0][1] = i[0][1].bound(); res.i[0][2] = i[0][2].bound();
	res.i[1][0] = i[1][0].bound(); res.i[1][1] = i[1][1].bound(); res.i[1][2] = i[1][2].bound();
	res.i[2][0] = i[2][0].bound(); res.i[2][1] = i[2][1].bound(); res.i[2][2] = i[2][2].bound();

	return res;

}

double cTM2Matrix33::diameter() {

	double diameter = 0;

	if (i[0][0].r.diameter() > diameter) diameter = i[0][0].r.diameter();
	if (i[0][1].r.diameter() > diameter) diameter = i[0][1].r.diameter();
	if (i[0][2].r.diameter() > diameter) diameter = i[0][2].r.diameter();

	if (i[1][0].r.diameter() > diameter) diameter = i[1][0].r.diameter();
	if (i[1][1].r.diameter() > diameter) diameter = i[1][1].r.diameter();
	if (i[1][2].r.diameter() > diameter) diameter = i[1][2].r.diameter();

	if (i[2][0].r.diameter() > diameter) diameter = i[2][0].r.diameter();
	if (i[2][1].r.diameter() > diameter) diameter = i[2][1].r.diameter();
	if (i[2][2].r.diameter() > diameter) diameter = i[2][2].r.diameter();

	return diameter;

}

cTM2Matrix33 operator*(cTaylorModel2& t, cTM2Matrix33& m) {

	cTaylorModel2 res[3][3];

	res[0][0] = t * m.i[0][0]; res[0][1] = t * m.i[0][1]; res[0][2] = t * m.i[0][2];
	res[1][0] = t * m.i[1][0]; res[1][1] = t * m.i[1][1]; res[1][2] = t * m.i[1][2];
	res[2][0] = t * m.i[2][0]; res[2][1] = t * m.i[2][1]; res[2][2] = t * m.i[2][2];

	return cTM2Matrix33(res);

}

