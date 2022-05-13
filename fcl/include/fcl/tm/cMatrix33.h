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


#ifndef __CMATRIX33_HPP__
#define __CMATRIX33_HPP__

#include "cVector3.h"

#include <math.h>

class cMatrix33 {

public:

	double				m[3][3];
	
	cMatrix33();
	cMatrix33(double v);
	cMatrix33(cVector3 d);
	cMatrix33(double mat[3][3]);
	cMatrix33(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);
	cMatrix33(cVector3 v0, cVector3 v1, cVector3 v2);

	cVector3			getE1() { return cVector3(m[0][0],m[1][0],m[2][0]); }
	cVector3			getE2() { return cVector3(m[0][1],m[1][1],m[2][1]); }
	cVector3			getE3() { return cVector3(m[0][2],m[1][2],m[2][2]); }

	cMatrix33			operator+(cMatrix33& mat);
	void				operator+=(cMatrix33 &mat);
	cMatrix33			operator*(cMatrix33& mat) const;
	cVector3			operator*(cVector3 &vec);
	void				operator*=(double d);
	cMatrix33			operator*(double d) const;
	
	void				print();
	void				setIdentity();
	void				setZero();
	cMatrix33			getTranspose() const;
	void				diagonalize(cVector3 &ev, cMatrix33 &p);


};


#endif // __CMATRIX33_HPP__
