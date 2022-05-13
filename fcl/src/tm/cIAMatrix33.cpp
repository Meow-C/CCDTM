/*************************************************************************\

  Copyright 2004 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify OR distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The author may be contacted via:

  US Mail:             Stephane Redon
					   Department of Computer Science
					   Sitterson Hall, CB #3175
					   University of N. Carolina
					   Chapel Hill, NC 27599-3175

  Phone:               (919) 962-1930

  EMail:               redon@cs.unc.edu

\**************************************************************************/

#include <math.h>
#include <iostream>
#include <memory.h>

#include "fcl/tm/cIAMatrix33.h"

#include "fcl/tm/cIAVector3.h"
#include "fcl/tm/cMatrix33.h"

using namespace std;

cIAMatrix33::cIAMatrix33() { i[0][0] = i[1][1] = i[2][2] = i[0][1] = i[0][2] = i[1][0] = i[1][2] = i[2][0] = i[2][1] = 0.0; }
cIAMatrix33::cIAMatrix33(double v) { i[0][0] = i[1][1] = i[2][2] = i[0][1] = i[0][2] = i[1][0] = i[1][2] = i[2][0] = i[2][1] = v; }
cIAMatrix33::cIAMatrix33(cMatrix33& mat) {

	i[0][0] = mat.m[0][0]; i[0][1] = mat.m[0][1]; i[0][2] = mat.m[0][2];
	i[1][0] = mat.m[1][0]; i[1][1] = mat.m[1][1]; i[1][2] = mat.m[1][2];
	i[2][0] = mat.m[2][0]; i[2][1] = mat.m[2][1]; i[2][2] = mat.m[2][2];

}
cIAMatrix33::cIAMatrix33(double mat[3][3][2]) { memcpy(i, mat, sizeof(cIAMatrix33)); }
void cIAMatrix33::setIdentity() { i[0][0] = i[1][1] = i[2][2] = 1.0; i[0][1] = i[0][2] = i[1][0] = i[1][2] = i[2][0] = i[2][1] = 0.0; }
cIAVector3 cIAMatrix33::getE1() { return cIAVector3(i[0][0], i[1][0], i[2][0]); }
cIAVector3 cIAMatrix33::getE2() { return cIAVector3(i[0][1], i[1][1], i[2][1]); }
cIAVector3 cIAMatrix33::getE3() { return cIAVector3(i[0][2], i[1][2], i[2][2]); }
cVector3 cIAMatrix33::getRealE1() { return cVector3(i[0][0].i[0], i[1][0].i[0], i[2][0].i[0]); }
cVector3 cIAMatrix33::getRealE2() { return cVector3(i[0][1].i[0], i[1][1].i[0], i[2][1].i[0]); }
cVector3 cIAMatrix33::getRealE3() { return cVector3(i[0][2].i[0], i[1][2].i[0], i[2][2].i[0]); }

cIAMatrix33 cIAMatrix33::operator*(cMatrix33& mat) {

	// Interval method to multiply an interval matrix by a real matrix

	double res[3][3][2];

	if (mat.m[0][0] < 0) {
		res[0][0][0] = mat.m[0][0] * i[0][0].i[1]; res[0][0][1] = mat.m[0][0] * i[0][0].i[0];
		res[1][0][0] = mat.m[0][0] * i[1][0].i[1]; res[1][0][1] = mat.m[0][0] * i[1][0].i[0];
		res[2][0][0] = mat.m[0][0] * i[2][0].i[1]; res[2][0][1] = mat.m[0][0] * i[2][0].i[0];
	}
	else {
		res[0][0][0] = mat.m[0][0] * i[0][0].i[0]; res[0][0][1] = mat.m[0][0] * i[0][0].i[1];
		res[1][0][0] = mat.m[0][0] * i[1][0].i[0]; res[1][0][1] = mat.m[0][0] * i[1][0].i[1];
		res[2][0][0] = mat.m[0][0] * i[2][0].i[0]; res[2][0][1] = mat.m[0][0] * i[2][0].i[1];
	}

	if (mat.m[0][1] < 0) {
		res[0][1][0] = mat.m[0][1] * i[0][0].i[1]; res[0][1][1] = mat.m[0][1] * i[0][0].i[0];
		res[1][1][0] = mat.m[0][1] * i[1][0].i[1]; res[1][1][1] = mat.m[0][1] * i[1][0].i[0];
		res[2][1][0] = mat.m[0][1] * i[2][0].i[1]; res[2][1][1] = mat.m[0][1] * i[2][0].i[0];
	}
	else {
		res[0][1][0] = mat.m[0][1] * i[0][0].i[0]; res[0][1][1] = mat.m[0][1] * i[0][0].i[1];
		res[1][1][0] = mat.m[0][1] * i[1][0].i[0]; res[1][1][1] = mat.m[0][1] * i[1][0].i[1];
		res[2][1][0] = mat.m[0][1] * i[2][0].i[0]; res[2][1][1] = mat.m[0][1] * i[2][0].i[1];
	}

	if (mat.m[0][2] < 0) {
		res[0][2][0] = mat.m[0][2] * i[0][0].i[1]; res[0][2][1] = mat.m[0][2] * i[0][0].i[0];
		res[1][2][0] = mat.m[0][2] * i[1][0].i[1]; res[1][2][1] = mat.m[0][2] * i[1][0].i[0];
		res[2][2][0] = mat.m[0][2] * i[2][0].i[1]; res[2][2][1] = mat.m[0][2] * i[2][0].i[0];
	}
	else {
		res[0][2][0] = mat.m[0][2] * i[0][0].i[0]; res[0][2][1] = mat.m[0][2] * i[0][0].i[1];
		res[1][2][0] = mat.m[0][2] * i[1][0].i[0]; res[1][2][1] = mat.m[0][2] * i[1][0].i[1];
		res[2][2][0] = mat.m[0][2] * i[2][0].i[0]; res[2][2][1] = mat.m[0][2] * i[2][0].i[1];
	}

	if (mat.m[1][0] < 0) {
		res[0][0][0] += mat.m[1][0] * i[0][1].i[1]; res[0][0][1] += mat.m[1][0] * i[0][1].i[0];
		res[1][0][0] += mat.m[1][0] * i[1][1].i[1]; res[1][0][1] += mat.m[1][0] * i[1][1].i[0];
		res[2][0][0] += mat.m[1][0] * i[2][1].i[1]; res[2][0][1] += mat.m[1][0] * i[2][1].i[0];
	}
	else {
		res[0][0][0] += mat.m[1][0] * i[0][1].i[0]; res[0][0][1] += mat.m[1][0] * i[0][1].i[1];
		res[1][0][0] += mat.m[1][0] * i[1][1].i[0]; res[1][0][1] += mat.m[1][0] * i[1][1].i[1];
		res[2][0][0] += mat.m[1][0] * i[2][1].i[0]; res[2][0][1] += mat.m[1][0] * i[2][1].i[1];
	}

	if (mat.m[1][1] < 0) {
		res[0][1][0] += mat.m[1][1] * i[0][1].i[1]; res[0][1][1] += mat.m[1][1] * i[0][1].i[0];
		res[1][1][0] += mat.m[1][1] * i[1][1].i[1]; res[1][1][1] += mat.m[1][1] * i[1][1].i[0];
		res[2][1][0] += mat.m[1][1] * i[2][1].i[1]; res[2][1][1] += mat.m[1][1] * i[2][1].i[0];
	}
	else {
		res[0][1][0] += mat.m[1][1] * i[0][1].i[0]; res[0][1][1] += mat.m[1][1] * i[0][1].i[1];
		res[1][1][0] += mat.m[1][1] * i[1][1].i[0]; res[1][1][1] += mat.m[1][1] * i[1][1].i[1];
		res[2][1][0] += mat.m[1][1] * i[2][1].i[0]; res[2][1][1] += mat.m[1][1] * i[2][1].i[1];
	}

	if (mat.m[1][2] < 0) {
		res[0][2][0] += mat.m[1][2] * i[0][1].i[1]; res[0][2][1] += mat.m[1][2] * i[0][1].i[0];
		res[1][2][0] += mat.m[1][2] * i[1][1].i[1]; res[1][2][1] += mat.m[1][2] * i[1][1].i[0];
		res[2][2][0] += mat.m[1][2] * i[2][1].i[1]; res[2][2][1] += mat.m[1][2] * i[2][1].i[0];
	}
	else {
		res[0][2][0] += mat.m[1][2] * i[0][1].i[0]; res[0][2][1] += mat.m[1][2] * i[0][1].i[1];
		res[1][2][0] += mat.m[1][2] * i[1][1].i[0]; res[1][2][1] += mat.m[1][2] * i[1][1].i[1];
		res[2][2][0] += mat.m[1][2] * i[2][1].i[0]; res[2][2][1] += mat.m[1][2] * i[2][1].i[1];
	}

	if (mat.m[2][0] < 0) {
		res[0][0][0] += mat.m[2][0] * i[0][2].i[1]; res[0][0][1] += mat.m[2][0] * i[0][2].i[0];
		res[1][0][0] += mat.m[2][0] * i[1][2].i[1]; res[1][0][1] += mat.m[2][0] * i[1][2].i[0];
		res[2][0][0] += mat.m[2][0] * i[2][2].i[1]; res[2][0][1] += mat.m[2][0] * i[2][2].i[0];
	}
	else {
		res[0][0][0] += mat.m[2][0] * i[0][2].i[0]; res[0][0][1] += mat.m[2][0] * i[0][2].i[1];
		res[1][0][0] += mat.m[2][0] * i[1][2].i[0]; res[1][0][1] += mat.m[2][0] * i[1][2].i[1];
		res[2][0][0] += mat.m[2][0] * i[2][2].i[0]; res[2][0][1] += mat.m[2][0] * i[2][2].i[1];
	}

	if (mat.m[2][1] < 0) {
		res[0][1][0] += mat.m[2][1] * i[0][2].i[1]; res[0][1][1] += mat.m[2][1] * i[0][2].i[0];
		res[1][1][0] += mat.m[2][1] * i[1][2].i[1]; res[1][1][1] += mat.m[2][1] * i[1][2].i[0];
		res[2][1][0] += mat.m[2][1] * i[2][2].i[1]; res[2][1][1] += mat.m[2][1] * i[2][2].i[0];
	}
	else {
		res[0][1][0] += mat.m[2][1] * i[0][2].i[0]; res[0][1][1] += mat.m[2][1] * i[0][2].i[1];
		res[1][1][0] += mat.m[2][1] * i[1][2].i[0]; res[1][1][1] += mat.m[2][1] * i[1][2].i[1];
		res[2][1][0] += mat.m[2][1] * i[2][2].i[0]; res[2][1][1] += mat.m[2][1] * i[2][2].i[1];
	}

	if (mat.m[2][2] < 0) {
		res[0][2][0] += mat.m[2][2] * i[0][2].i[1]; res[0][2][1] += mat.m[2][2] * i[0][2].i[0];
		res[1][2][0] += mat.m[2][2] * i[1][2].i[1]; res[1][2][1] += mat.m[2][2] * i[1][2].i[0];
		res[2][2][0] += mat.m[2][2] * i[2][2].i[1]; res[2][2][1] += mat.m[2][2] * i[2][2].i[0];
	}
	else {
		res[0][2][0] += mat.m[2][2] * i[0][2].i[0]; res[0][2][1] += mat.m[2][2] * i[0][2].i[1];
		res[1][2][0] += mat.m[2][2] * i[1][2].i[0]; res[1][2][1] += mat.m[2][2] * i[1][2].i[1];
		res[2][2][0] += mat.m[2][2] * i[2][2].i[0]; res[2][2][1] += mat.m[2][2] * i[2][2].i[1];
	}

	return cIAMatrix33(res);

}

cIAVector3 cIAMatrix33::operator*(cVector3& vec) {

	// Interval method to multiply an interval matrix by a real matrix

	double res[3][2];

	if (vec.v[0] < 0) {
		res[0][0] = vec.v[0] * i[0][0].i[1]; res[0][1] = vec.v[0] * i[0][0].i[0];
		res[1][0] = vec.v[0] * i[1][0].i[1]; res[1][1] = vec.v[0] * i[1][0].i[0];
		res[2][0] = vec.v[0] * i[2][0].i[1]; res[2][1] = vec.v[0] * i[2][0].i[0];
	}
	else {
		res[0][0] = vec.v[0] * i[0][0].i[0]; res[0][1] = vec.v[0] * i[0][0].i[1];
		res[1][0] = vec.v[0] * i[1][0].i[0]; res[1][1] = vec.v[0] * i[1][0].i[1];
		res[2][0] = vec.v[0] * i[2][0].i[0]; res[2][1] = vec.v[0] * i[2][0].i[1];
	}

	if (vec.v[1] < 0) {
		res[0][0] += vec.v[1] * i[0][1].i[1]; res[0][1] += vec.v[1] * i[0][1].i[0];
		res[1][0] += vec.v[1] * i[1][1].i[1]; res[1][1] += vec.v[1] * i[1][1].i[0];
		res[2][0] += vec.v[1] * i[2][1].i[1]; res[2][1] += vec.v[1] * i[2][1].i[0];
	}
	else {
		res[0][0] += vec.v[1] * i[0][1].i[0]; res[0][1] += vec.v[1] * i[0][1].i[1];
		res[1][0] += vec.v[1] * i[1][1].i[0]; res[1][1] += vec.v[1] * i[1][1].i[1];
		res[2][0] += vec.v[1] * i[2][1].i[0]; res[2][1] += vec.v[1] * i[2][1].i[1];
	}

	if (vec.v[2] < 0) {
		res[0][0] += vec.v[2] * i[0][2].i[1]; res[0][1] += vec.v[2] * i[0][2].i[0];
		res[1][0] += vec.v[2] * i[1][2].i[1]; res[1][1] += vec.v[2] * i[1][2].i[0];
		res[2][0] += vec.v[2] * i[2][2].i[1]; res[2][1] += vec.v[2] * i[2][2].i[0];
	}
	else {
		res[0][0] += vec.v[2] * i[0][2].i[0]; res[0][1] += vec.v[2] * i[0][2].i[1];
		res[1][0] += vec.v[2] * i[1][2].i[0]; res[1][1] += vec.v[2] * i[1][2].i[1];
		res[2][0] += vec.v[2] * i[2][2].i[0]; res[2][1] += vec.v[2] * i[2][2].i[1];
	}

	return cIAVector3(res);

}

cIAMatrix33 cIAMatrix33::AddRealMatrix(cIAMatrix33& mat) {

	double res[3][3][2];

	res[0][0][0] = i[0][0].i[0] + mat.i[0][0].i[0];
	res[0][1][0] = i[0][1].i[0] + mat.i[0][1].i[0];
	res[0][2][0] = i[0][2].i[0] + mat.i[0][2].i[0];

	res[1][0][0] = i[1][0].i[0] + mat.i[1][0].i[0];
	res[1][1][0] = i[1][1].i[0] + mat.i[1][1].i[0];
	res[1][2][0] = i[1][2].i[0] + mat.i[1][2].i[0];

	res[2][0][0] = i[2][0].i[0] + mat.i[2][0].i[0];
	res[2][1][0] = i[2][1].i[0] + mat.i[2][1].i[0];
	res[2][2][0] = i[2][2].i[0] + mat.i[2][2].i[0];

	return cIAMatrix33(res);

}

// *******************************
// Interval arithmetics operations
// *******************************

cIAVector3 cIAMatrix33::operator*(cIAVector3& v) {

	// Interval method : r=m*v 

	double xl, xu, yl, yu, zl, zu;
	register double temp, vmin, vmax;

	// r.v[0]

	vmin = vmax = i[0][0].i[0] * v.i[0].i[0];
	temp = i[0][0].i[0] * v.i[0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * v.i[0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * v.i[0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	xl = vmin; xu = vmax;

	vmin = vmax = i[0][1].i[0] * v.i[1].i[0];
	temp = i[0][1].i[0] * v.i[1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * v.i[1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * v.i[1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	xl += vmin; xu += vmax;

	vmin = vmax = i[0][2].i[0] * v.i[2].i[0];
	temp = i[0][2].i[0] * v.i[2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * v.i[2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * v.i[2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	xl += vmin; xu += vmax;

	// r.v[1]

	vmin = vmax = i[1][0].i[0] * v.i[0].i[0];
	temp = i[1][0].i[0] * v.i[0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * v.i[0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * v.i[0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	yl = vmin; yu = vmax;

	vmin = vmax = i[1][1].i[0] * v.i[1].i[0];
	temp = i[1][1].i[0] * v.i[1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * v.i[1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * v.i[1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	yl += vmin; yu += vmax;

	vmin = vmax = i[1][2].i[0] * v.i[2].i[0];
	temp = i[1][2].i[0] * v.i[2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * v.i[2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * v.i[2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	yl += vmin; yu += vmax;

	// r.v[2]

	vmin = vmax = i[2][0].i[0] * v.i[0].i[0];
	temp = i[2][0].i[0] * v.i[0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * v.i[0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * v.i[0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	zl = vmin; zu = vmax;

	vmin = vmax = i[2][1].i[0] * v.i[1].i[0];
	temp = i[2][1].i[0] * v.i[1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * v.i[1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * v.i[1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	zl += vmin; zu += vmax;

	vmin = vmax = i[2][2].i[0] * v.i[2].i[0];
	temp = i[2][2].i[0] * v.i[2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * v.i[2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * v.i[2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	zl += vmin; zu += vmax;

	return cIAVector3(xl, xu, yl, yu, zl, zu);

}

cIAMatrix33 cIAMatrix33::operator*(cIAMatrix33& mat) {

	// Interval method : res=m*mat 

	register double temp, vmin, vmax;
	double res[3][3][2];

	// res[0][0]

	vmin = vmax = i[0][0].i[0] * mat.i[0][0].i[0];
	temp = i[0][0].i[0] * mat.i[0][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * mat.i[0][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * mat.i[0][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][0][0] = vmin; res[0][0][1] = vmax;

	vmin = vmax = i[0][1].i[0] * mat.i[1][0].i[0];
	temp = i[0][1].i[0] * mat.i[1][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * mat.i[1][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * mat.i[1][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][0][0] += vmin; res[0][0][1] += vmax;

	vmin = vmax = i[0][2].i[0] * mat.i[2][0].i[0];
	temp = i[0][2].i[0] * mat.i[2][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * mat.i[2][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * mat.i[2][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][0][0] += vmin; res[0][0][1] += vmax;

	// res[1][0]

	vmin = vmax = i[1][0].i[0] * mat.i[0][0].i[0];
	temp = i[1][0].i[0] * mat.i[0][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * mat.i[0][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * mat.i[0][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][0][0] = vmin; res[1][0][1] = vmax;

	vmin = vmax = i[1][1].i[0] * mat.i[1][0].i[0];
	temp = i[1][1].i[0] * mat.i[1][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * mat.i[1][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * mat.i[1][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][0][0] += vmin; res[1][0][1] += vmax;

	vmin = vmax = i[1][2].i[0] * mat.i[2][0].i[0];
	temp = i[1][2].i[0] * mat.i[2][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * mat.i[2][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * mat.i[2][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][0][0] += vmin; res[1][0][1] += vmax;

	// res[2][0]

	vmin = vmax = i[2][0].i[0] * mat.i[0][0].i[0];
	temp = i[2][0].i[0] * mat.i[0][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * mat.i[0][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * mat.i[0][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][0][0] = vmin; res[2][0][1] = vmax;

	vmin = vmax = i[2][1].i[0] * mat.i[1][0].i[0];
	temp = i[2][1].i[0] * mat.i[1][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * mat.i[1][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * mat.i[1][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][0][0] += vmin; res[2][0][1] += vmax;

	vmin = vmax = i[2][2].i[0] * mat.i[2][0].i[0];
	temp = i[2][2].i[0] * mat.i[2][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * mat.i[2][0].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * mat.i[2][0].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][0][0] += vmin; res[2][0][1] += vmax;

	// res[0][1]

	vmin = vmax = i[0][0].i[0] * mat.i[0][1].i[0];
	temp = i[0][0].i[0] * mat.i[0][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * mat.i[0][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * mat.i[0][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][1][0] = vmin; res[0][1][1] = vmax;

	vmin = vmax = i[0][1].i[0] * mat.i[1][1].i[0];
	temp = i[0][1].i[0] * mat.i[1][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * mat.i[1][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * mat.i[1][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][1][0] += vmin; res[0][1][1] += vmax;

	vmin = vmax = i[0][2].i[0] * mat.i[2][1].i[0];
	temp = i[0][2].i[0] * mat.i[2][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * mat.i[2][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * mat.i[2][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][1][0] += vmin; res[0][1][1] += vmax;

	// res[1][1]

	vmin = vmax = i[1][0].i[0] * mat.i[0][1].i[0];
	temp = i[1][0].i[0] * mat.i[0][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * mat.i[0][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * mat.i[0][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][1][0] = vmin; res[1][1][1] = vmax;

	vmin = vmax = i[1][1].i[0] * mat.i[1][1].i[0];
	temp = i[1][1].i[0] * mat.i[1][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * mat.i[1][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * mat.i[1][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][1][0] += vmin; res[1][1][1] += vmax;

	vmin = vmax = i[1][2].i[0] * mat.i[2][1].i[0];
	temp = i[1][2].i[0] * mat.i[2][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * mat.i[2][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * mat.i[2][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][1][0] += vmin; res[1][1][1] += vmax;

	// res[2][1]

	vmin = vmax = i[2][0].i[0] * mat.i[0][1].i[0];
	temp = i[2][0].i[0] * mat.i[0][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * mat.i[0][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * mat.i[0][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][1][0] = vmin; res[2][1][1] = vmax;

	vmin = vmax = i[2][1].i[0] * mat.i[1][1].i[0];
	temp = i[2][1].i[0] * mat.i[1][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * mat.i[1][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * mat.i[1][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][1][0] += vmin; res[2][1][1] += vmax;

	vmin = vmax = i[2][2].i[0] * mat.i[2][1].i[0];
	temp = i[2][2].i[0] * mat.i[2][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * mat.i[2][1].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * mat.i[2][1].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][1][0] += vmin; res[2][1][1] += vmax;

	// res[0][2]

	vmin = vmax = i[0][0].i[0] * mat.i[0][2].i[0];
	temp = i[0][0].i[0] * mat.i[0][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * mat.i[0][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][0].i[1] * mat.i[0][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][2][0] = vmin; res[0][2][1] = vmax;

	vmin = vmax = i[0][1].i[0] * mat.i[1][2].i[0];
	temp = i[0][1].i[0] * mat.i[1][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * mat.i[1][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][1].i[1] * mat.i[1][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][2][0] += vmin; res[0][2][1] += vmax;

	vmin = vmax = i[0][2].i[0] * mat.i[2][2].i[0];
	temp = i[0][2].i[0] * mat.i[2][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * mat.i[2][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[0][2].i[1] * mat.i[2][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[0][2][0] += vmin; res[0][2][1] += vmax;

	// res[1][2]

	vmin = vmax = i[1][0].i[0] * mat.i[0][2].i[0];
	temp = i[1][0].i[0] * mat.i[0][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * mat.i[0][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][0].i[1] * mat.i[0][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][2][0] = vmin; res[1][2][1] = vmax;

	vmin = vmax = i[1][1].i[0] * mat.i[1][2].i[0];
	temp = i[1][1].i[0] * mat.i[1][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * mat.i[1][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][1].i[1] * mat.i[1][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][2][0] += vmin; res[1][2][1] += vmax;

	vmin = vmax = i[1][2].i[0] * mat.i[2][2].i[0];
	temp = i[1][2].i[0] * mat.i[2][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * mat.i[2][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[1][2].i[1] * mat.i[2][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[1][2][0] += vmin; res[1][2][1] += vmax;

	// res[2][2]

	vmin = vmax = i[2][0].i[0] * mat.i[0][2].i[0];
	temp = i[2][0].i[0] * mat.i[0][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * mat.i[0][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][0].i[1] * mat.i[0][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][2][0] = vmin; res[2][2][1] = vmax;

	vmin = vmax = i[2][1].i[0] * mat.i[1][2].i[0];
	temp = i[2][1].i[0] * mat.i[1][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * mat.i[1][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][1].i[1] * mat.i[1][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][2][0] += vmin; res[2][2][1] += vmax;

	vmin = vmax = i[2][2].i[0] * mat.i[2][2].i[0];
	temp = i[2][2].i[0] * mat.i[2][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * mat.i[2][2].i[0]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	temp = i[2][2].i[1] * mat.i[2][2].i[1]; if (temp < vmin) vmin = temp; else if (temp > vmax) vmax = temp;
	res[2][2][0] += vmin; res[2][2][1] += vmax;

	return cIAMatrix33(res);

}

cIAMatrix33 cIAMatrix33::operator+(cIAMatrix33& mat) {

	// res=m+mat

	double res[3][3][2];

	res[0][0][0] = i[0][0].i[0] + mat.i[0][0].i[0]; res[0][0][1] = i[0][0].i[1] + mat.i[0][0].i[1]; res[0][1][0] = i[0][1].i[0] + mat.i[0][1].i[0]; res[0][1][1] = i[0][1].i[1] + mat.i[0][1].i[1]; res[0][2][0] = i[0][2].i[0] + mat.i[0][2].i[0]; res[0][2][1] = i[0][2].i[1] + mat.i[0][2].i[1];
	res[1][0][0] = i[1][0].i[0] + mat.i[1][0].i[0]; res[1][0][1] = i[1][0].i[1] + mat.i[1][0].i[1]; res[1][1][0] = i[1][1].i[0] + mat.i[1][1].i[0]; res[1][1][1] = i[1][1].i[1] + mat.i[1][1].i[1]; res[1][2][0] = i[1][2].i[0] + mat.i[1][2].i[0]; res[1][2][1] = i[1][2].i[1] + mat.i[1][2].i[1];
	res[2][0][0] = i[2][0].i[0] + mat.i[2][0].i[0]; res[2][0][1] = i[2][0].i[1] + mat.i[2][0].i[1]; res[2][1][0] = i[2][1].i[0] + mat.i[2][1].i[0]; res[2][1][1] = i[2][1].i[1] + mat.i[2][1].i[1]; res[2][2][0] = i[2][2].i[0] + mat.i[2][2].i[0]; res[2][2][1] = i[2][2].i[1] + mat.i[2][2].i[1];

	return cIAMatrix33(res);

}

void cIAMatrix33::operator+=(cIAMatrix33& mat) {

	// m=m+mat

	i[0][0].i[0] += mat.i[0][0].i[0]; i[0][0].i[1] += mat.i[0][0].i[1]; i[0][1].i[0] += mat.i[0][1].i[0]; i[0][1].i[1] += mat.i[0][1].i[1]; i[0][2].i[0] += mat.i[0][2].i[0]; i[0][2].i[1] += mat.i[0][2].i[1];
	i[1][0].i[0] += mat.i[1][0].i[0]; i[1][0].i[1] += mat.i[1][0].i[1]; i[1][1].i[0] += mat.i[1][1].i[0]; i[1][1].i[1] += mat.i[1][1].i[1]; i[1][2].i[0] += mat.i[1][2].i[0]; i[1][2].i[1] += mat.i[1][2].i[1];
	i[2][0].i[0] += mat.i[2][0].i[0]; i[2][0].i[1] += mat.i[2][0].i[1]; i[2][1].i[0] += mat.i[2][1].i[0]; i[2][1].i[1] += mat.i[2][1].i[1]; i[2][2].i[0] += mat.i[2][2].i[0]; i[2][2].i[1] += mat.i[2][2].i[1];

}


// ***********************
// Non interval operations
// ***********************

cIAVector3 cIAMatrix33::MultiplyRealVector(cIAVector3& v) {

	return cIAVector3(
		i[0][0].i[0] * v.i[0].i[0] + i[0][1].i[0] * v.i[1].i[0] + i[0][2].i[0] * v.i[2].i[0],
		i[1][0].i[0] * v.i[0].i[0] + i[1][1].i[0] * v.i[1].i[0] + i[1][2].i[0] * v.i[2].i[0],
		i[2][0].i[0] * v.i[0].i[0] + i[2][1].i[0] * v.i[1].i[0] + i[2][2].i[0] * v.i[2].i[0]);

}

cIAVector3 cIAMatrix33::MultiplyRealVector(cVector3& vec) {

	return cIAVector3(
		i[0][0].i[0] * vec.v[0] + i[0][1].i[0] * vec.v[1] + i[0][2].i[0] * vec.v[2],
		i[1][0].i[0] * vec.v[0] + i[1][1].i[0] * vec.v[1] + i[1][2].i[0] * vec.v[2],
		i[2][0].i[0] * vec.v[0] + i[2][1].i[0] * vec.v[1] + i[2][2].i[0] * vec.v[2]);

}

void cIAMatrix33::print() {
	cout << " [" << i[0][0].i[0] << "," << i[0][0].i[1] << "]" << " [" << i[0][1].i[0] << "," << i[0][1].i[1] << "]" << " [" << i[0][2].i[0] << "," << i[0][2].i[1] << "]" << endl;
	cout << " [" << i[1][0].i[0] << "," << i[1][0].i[1] << "]" << " [" << i[1][1].i[0] << "," << i[1][1].i[1] << "]" << " [" << i[1][2].i[0] << "," << i[1][2].i[1] << "]" << endl;
	cout << " [" << i[2][0].i[0] << "," << i[2][0].i[1] << "]" << " [" << i[2][1].i[0] << "," << i[2][1].i[1] << "]" << " [" << i[2][2].i[0] << "," << i[2][2].i[1] << "]" << endl;
}



