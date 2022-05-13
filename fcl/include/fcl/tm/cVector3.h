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


#ifndef __CVECTOR3_HPP__
#define __CVECTOR3_HPP__

#include <math.h>
#include <iostream>

class cIAVector3;

class cVector3 {

public:

	double				v[3];

	cVector3();
	cVector3(double x);
	cVector3(double x, double y, double z);
	cVector3(double val[3]);
 	cVector3(const cIAVector3&);
   	
	double				operator|(const cVector3& u) const;
	cVector3			operator^(const cVector3& u) const;
	cVector3			operator+(const cVector3 &u) const;
	cVector3&			operator+=(const cVector3 &u);
	cVector3			operator-(const cVector3 &u) const;
	cVector3&			operator-=(const cVector3 &u);
	cVector3			operator*(double d) const;
	cVector3&			operator*=(double d);
	cVector3			operator/(double d) const;
	cVector3&			operator/=(double d);
	cVector3			operator-();
	
    void				setZero();
	void				normalize();
	double				norm() const;
	double				norm2()	const;
	void				print()	const;
    void                scale(const cVector3 &u);

};


cVector3 operator*(double d, cVector3 u);

#endif // __CVECTOR3_HPP__
