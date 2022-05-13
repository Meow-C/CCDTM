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
#include <memory.h>

#include "fcl/tm/cIAVector3.h"
#include "fcl/tm/cInterval.h"
#include "fcl/tm/cVector3.h"
#include "fcl/tm/cTM3Vector3.h"


cIAVector3::cIAVector3() {}
cIAVector3::cIAVector3(double val) { i[0]=i[1]=i[2]=val; }
cIAVector3::cIAVector3(double x, double y, double z) { i[0].i[0]=x;i[1].i[0]=y;i[2].i[0]=z; }
cIAVector3::cIAVector3(double xl, double xu, double yl, double yu, double zl, double zu) {
	i[0].i[0]=xl;i[0].i[1]=xu;
	i[1].i[0]=yl;i[1].i[1]=yu;
	i[2].i[0]=zl;i[2].i[1]=zu;
}
cIAVector3::cIAVector3(double val[3][2]) { memcpy(i,val,6*sizeof(double)); }
cIAVector3::cIAVector3(cInterval val[3]) { memcpy(i,val,6*sizeof(double)); }
cIAVector3::cIAVector3(cInterval &nx, cInterval &ny, cInterval &nz) { i[0]=nx;i[1]=ny;i[2]=nz; }
cIAVector3::cIAVector3(cVector3 &u) { i[0]=u.v[0];i[1]=u.v[1];i[2]=u.v[2]; }
cIAVector3::cIAVector3(cTM3Vector3 &u) { i[0]=u.i[0].bound();i[1]=u.i[1].bound();i[2]=u.i[2].bound(); }

void cIAVector3::setZero() { i[0]=i[1]=i[2]=0.0; }

// operators

cIAVector3 cIAVector3::operator+(const cIAVector3 &u) { cInterval res[3];res[0]=i[0]+u.i[0];res[1]=i[1]+u.i[1];res[2]=i[2]+u.i[2];return cIAVector3(res); }
cIAVector3& cIAVector3::operator+=(const cIAVector3 &u) { i[0]+=u.i[0];i[1]+=u.i[1];i[2]+=u.i[2];return *this; }
cIAVector3 cIAVector3::operator-(const cIAVector3 &u) { cInterval res[3];res[0]=i[0]-u.i[0];res[1]=i[1]-u.i[1];res[2]=i[2]-u.i[2];return cIAVector3(res); }
cIAVector3& cIAVector3::operator-=(const cIAVector3 &u) { i[0]-=u.i[0];i[1]-=u.i[1];i[2]-=u.i[2];return *this; }
cIAVector3& cIAVector3::operator=(const cVector3 &u) { i[0]=u.v[0];i[1]=u.v[1];i[2]=u.v[2];return *this; }
cInterval cIAVector3::operator|(const cIAVector3 &u) { return i[0]*u.i[0]+i[1]*u.i[1]+i[2]*u.i[2]; }
cIAVector3 cIAVector3::operator^(const cIAVector3 &u) { cInterval res[3];res[0]=i[1]*u.i[2]-i[2]*u.i[1];res[1]=i[2]*u.i[0]-i[0]*u.i[2];res[2]=i[0]*u.i[1]-i[1]*u.i[0];return cIAVector3(res); }
double cIAVector3::volume() { return i[0].diameter()*i[1].diameter()*i[2].diameter(); }
void cIAVector3::print() {
	std::cout << " [" << i[0].i[0] << "," << i[0].i[1] << "]" << std::endl;
	std::cout << " [" << i[1].i[0] << "," << i[1].i[1] << "]" << std::endl;
	std::cout << " [" << i[2].i[0] << "," << i[2].i[1] << "]" << std::endl;
	std::cout << std::endl;
}
cVector3 cIAVector3::center() { return cVector3(i[0].center(),i[1].center(),i[2].center()); }
void cIAVector3::contain(cIAVector3 &u) { // enlarge the AABB represented by the IA vector to contain the one represented by u
	
	if (u.i[0].i[0]<i[0].i[0]) i[0].i[0]=u.i[0].i[0];
	if (u.i[1].i[0]<i[1].i[0]) i[1].i[0]=u.i[1].i[0];
	if (u.i[2].i[0]<i[2].i[0]) i[2].i[0]=u.i[2].i[0];
	if (u.i[0].i[1]>i[0].i[1]) i[0].i[1]=u.i[0].i[1];
	if (u.i[1].i[1]>i[1].i[1]) i[1].i[1]=u.i[1].i[1];
	if (u.i[2].i[1]>i[2].i[1]) i[2].i[1]=u.i[2].i[1];
	
}
void cIAVector3::contain(cVector3 &u) { // enlarge the AABB represented by the IA vector to contain the one vector by u
	
	if (u.v[0]<i[0].i[0]) i[0].i[0]=u.v[0];
	if (u.v[1]<i[1].i[0]) i[1].i[0]=u.v[1];
	if (u.v[2]<i[2].i[0]) i[2].i[0]=u.v[2];
	if (u.v[0]>i[0].i[1]) i[0].i[1]=u.v[0];
	if (u.v[1]>i[1].i[1]) i[1].i[1]=u.v[1];
	if (u.v[2]>i[2].i[1]) i[2].i[1]=u.v[2];
	
}
bool cIAVector3::overlaps(cIAVector3 &u) { // returns true if and only if the AABB u overlaps the AABB
	
	if (u.i[0].i[1]<i[0].i[0]) return false;
	if (u.i[0].i[0]>i[0].i[1]) return false;
	if (u.i[1].i[1]<i[1].i[0]) return false;
	if (u.i[1].i[0]>i[1].i[1]) return false;
	if (u.i[2].i[1]<i[2].i[0]) return false;
	if (u.i[2].i[0]>i[2].i[1]) return false;
	
	return true;
	
}

bool cIAVector3::inBounds(cIAVector3 &u) {

	if (i[0].i[0]<u.i[0].i[0]) return false;
	if (i[0].i[1]>u.i[0].i[1]) return false;
	if (i[1].i[0]<u.i[1].i[0]) return false;
	if (i[1].i[1]>u.i[1].i[1]) return false;
	if (i[2].i[0]<u.i[2].i[0]) return false;
	if (i[2].i[1]>u.i[2].i[1]) return false;

	return true;

}