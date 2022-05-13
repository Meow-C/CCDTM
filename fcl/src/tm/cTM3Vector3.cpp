/*************************************************************************\

  Copyright 2006 INRIA.
  All Rights Reserved.

  Permission to use, copy, modify OR distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL INRIA BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF INRIA HAS
  BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

  INRIA SPECIFICALLY DISCLAIMS ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND INRIA
  HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The author may be contacted via:

  Mail:					Stephane Redon
						INRIA Rhone-Alpes Research Unit
						655 avenue de l'Europe - Montbonnot
						38334 Saint Ismier Cedex - France

  Phone:				+33 4 76 61 55 69

  EMail:				stephane.redon@inria.fr

\**************************************************************************/



#include <math.h>
#include <memory.h>

#include "fcl/tm/cTM3Vector3.h"
#include "fcl/tm/cInterval.h"
#include "fcl/tm/cVector3.h"
#include "fcl/tm/cIAVector3.h"

cTM3Vector3::cTM3Vector3() {}
cTM3Vector3::cTM3Vector3(cTaylorModel3 val[3]) { memcpy(i, val, 3 * sizeof(cTaylorModel3)); }
cTM3Vector3::cTM3Vector3(cTaylorModel3& nx, cTaylorModel3& ny, cTaylorModel3& nz) { i[0] = nx; i[1] = ny; i[2] = nz; }
cTM3Vector3::cTM3Vector3(cVector3& u) { i[0] = cTaylorModel3(u.v[0]); i[1] = cTaylorModel3(u.v[1]); i[2] = cTaylorModel3(u.v[2]); }

void cTM3Vector3::setZero() { memset(this, 0, sizeof(cTM3Vector3)); }

// operators

cTM3Vector3 cTM3Vector3::operator+(const cTM3Vector3& u) { cTaylorModel3 res[3]; res[0] = i[0] + u.i[0]; res[1] = i[1] + u.i[1]; res[2] = i[2] + u.i[2]; return cTM3Vector3(res); }
cTM3Vector3 cTM3Vector3::operator+(const double& d) { cTaylorModel3 res[3]; res[0] = i[0]; res[1] = i[1]; res[2] = i[2]; res[0].c[0] += d; return cTM3Vector3(res); }
cTM3Vector3 cTM3Vector3::operator-(const cTM3Vector3& u) { cTaylorModel3 res[3]; res[0] = i[0] - u.i[0]; res[1] = i[1] - u.i[1]; res[2] = i[2] - u.i[2]; return cTM3Vector3(res); }
cTM3Vector3& cTM3Vector3::operator+=(const cTM3Vector3& u) { i[0] += u.i[0]; i[1] += u.i[1]; i[2] += u.i[2]; return *this; }
cTM3Vector3& cTM3Vector3::operator-=(const cTM3Vector3& u) { i[0] -= u.i[0]; i[1] -= u.i[1]; i[2] -= u.i[2]; return *this; }
cTM3Vector3& cTM3Vector3::operator=(const cVector3& u) { i[0] = cTaylorModel3(u.v[0]); i[1] = cTaylorModel3(u.v[1]); i[2] = cTaylorModel3(u.v[2]); return *this; }
cTaylorModel3 cTM3Vector3::operator|(const cTM3Vector3& u) { return i[0] * u.i[0] + i[1] * u.i[1] + i[2] * u.i[2]; } // interval dot product
cTM3Vector3 cTM3Vector3::operator^(const cTM3Vector3& u) { cTaylorModel3 res[3]; res[0] = i[1] * u.i[2] - i[2] * u.i[1]; res[1] = i[2] * u.i[0] - i[0] * u.i[2]; res[2] = i[0] * u.i[1] - i[1] * u.i[0]; return cTM3Vector3(res); } // interval cross product
double cTM3Vector3::volume() { return i[0].bound().diameter() * i[1].bound().diameter() * i[2].bound().diameter(); }

cIAVector3 cTM3Vector3::bound() {

	cInterval res[3];
	res[0] = i[0].bound();
	res[1] = i[1].bound();
	res[2] = i[2].bound();

	return cIAVector3(res);

}

void cTM3Vector3::print() {

	i[0].print();
	i[1].print();
	i[2].print();
	std::cout << std::endl;

}

cIAVector3 cTM3Vector3::evaluate(double t) {

	cInterval res[3];
	res[0] = i[0].evaluate(t);
	res[1] = i[1].evaluate(t);
	res[2] = i[2].evaluate(t);

	return cIAVector3(res);

}

cTaylorModel3 cTM3Vector3::dot(cTM3Vector3 other) {
//    cTM3Vector3 res(i[0] * other.i[0], i[1] * other.i[1], i[2] * other.i[2]);
    cTaylorModel3 res = i[0] * other.i[0] + i[1] * other.i[1] + i[2] * other.i[2];

    return res;
}

cTM3Vector3 operator*(cTaylorModel3& d, cTM3Vector3& v) {

	cTaylorModel3 res[3];
	res[0] = d * v.i[0];
	res[1] = d * v.i[1];
	res[2] = d * v.i[2];

	return cTM3Vector3(res);

}

cTM3Vector3 operator*(double d, cTM3Vector3& v) {

	cTaylorModel3 res[3];
	res[0] = d * v.i[0];
	res[1] = d * v.i[1];
	res[2] = d * v.i[2];

	return cTM3Vector3(res);

}

void cTM3Vector3::linearModel(cVector3& position, cVector3& velocity) {

	i[0].linearModel(position.v[0], velocity.v[0]);
	i[1].linearModel(position.v[1], velocity.v[1]);
	i[2].linearModel(position.v[2], velocity.v[2]);

}

