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


#ifndef __CTM3VECTOR3_HPP__
#define __CTM3VECTOR3_HPP__

#include <math.h>

#include "cTaylorModel3.h"
#include "cVector3.h"

class cTM3Vector3 {
public:

	cTaylorModel3	i[3];

	cTM3Vector3();
	cTM3Vector3(cTaylorModel3 val[3]);
	cTM3Vector3(cTaylorModel3& nx, cTaylorModel3& ny, cTaylorModel3& nz);
	cTM3Vector3(cVector3& u);

	cTM3Vector3		operator+(const cTM3Vector3& u);
	cTM3Vector3		operator+(const double& d);
	cTM3Vector3& operator+=(const cTM3Vector3& u);
	cTM3Vector3		operator-(const cTM3Vector3& u);
	cTM3Vector3& operator-=(const cTM3Vector3& u);
	cTM3Vector3& operator=(const cVector3& u);
	cTaylorModel3	operator|(const cTM3Vector3& u);
	cTM3Vector3		operator^(const cTM3Vector3& u);

	cIAVector3		bound();
	void			linearModel(cVector3& position, cVector3& velocity);
	void			print();
	double			volume();
	void			setZero();
	cTaylorModel3	norm2() { return (i[0] * i[0]) + (i[1] * i[1]) + (i[2] * i[2]); }
	cIAVector3		evaluate(double t);
	cTaylorModel3   dot(cTM3Vector3 other);

};

cTM3Vector3 operator*(cTaylorModel3&, cTM3Vector3&);
cTM3Vector3 operator*(double, cTM3Vector3&);

#endif // __CTM3VECTOR3_HPP__
