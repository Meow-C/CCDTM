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


#ifndef __CIAMATRIX33_HPP__
#define __CIAMATRIX33_HPP__

#include <math.h>
#include <iostream>

#include "cIAVector3.h"
#include "cMatrix33.h"

class cMatrix33;

// The class cIAMatrix33 implements a 3x3 interval matrix

class cIAMatrix33 {

public:

	cInterval		i[3][3];

	cIAMatrix33();
	cIAMatrix33(double v);
	cIAMatrix33(cMatrix33& m);
	cIAMatrix33(double mat[3][3][2]);
	void setIdentity();

	cIAVector3				getE1();
	cIAVector3				getE2();
	cIAVector3				getE3();
	cVector3				getRealE1();
	cVector3				getRealE2();
	cVector3				getRealE3();

	cIAMatrix33				AddRealMatrix(cIAMatrix33& mat);

	// *******************************
	// Interval arithmetics operations
	// *******************************

	cIAVector3				operator*(cVector3& v);
	cIAVector3				operator*(cIAVector3& v);
	cIAMatrix33				operator*(cIAMatrix33& mat);
	cIAMatrix33				operator*(cMatrix33& mat);
	cIAMatrix33				operator+(cIAMatrix33& mat);
	void					operator+=(cIAMatrix33& mat);

	// ***********************
	// Non interval operations
	// ***********************

	void					print();
	cIAVector3				MultiplyRealVector(cIAVector3& v);
	cIAVector3				MultiplyRealVector(cVector3& v);

};


#endif // __CIAMATRIX33_HPP__
