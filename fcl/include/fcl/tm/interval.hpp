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



#ifndef __INTERVAL_HPP__
#define __INTERVAL_HPP__

#include <math.h>
#include <iostream>

class cVector3;
class cMatrix33;
class cIAMatrix33;
class cIAVector3;
class cIAMAtrix33;
class cTM3Vector3;
class cTM3Matrix33; 

#define C_HALF_PI 1.57079632679489661

// useful functions

void cIABoundTwoBoxes(cIAVector3 &b1, cIAVector3 &b2, cIAVector3 &b3);

// non-interval functions

void cConstrainRotationMatrix(cIAMatrix33 *m); // constrain an interval rotation matrix based on its properties
void cConstrainRotationMatrixTM3(cTM3Matrix33 *m); // constrain an interval rotation matrix based on its properties
void cMultiply3x3Matrices(cIAMatrix33 *m1, cIAMatrix33 *m2, cIAMatrix33 *m3);
void cMultiply3x3Matrices(cMatrix33 *m1, cMatrix33 *m2, cMatrix33 *m3);
void cIAMultiplyMatrix33PositionVector3(double t0, double t1, cIAMatrix33 *m, cVector3 *c, cVector3 *s, cIAVector3 *r);


#endif
