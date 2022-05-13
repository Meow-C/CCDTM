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


#include "fcl/tm/interval.hpp"

#include <math.h>
#include <stdlib.h>
#include <iostream>

#include "fcl/tm/cVector3.h"
#include "fcl/tm/cMatrix33.h"
#include "fcl/tm/cIAVector3.h"
#include "fcl/tm/cIAMatrix33.h"
#include "fcl/tm/cTM3Vector3.h"
#include "fcl/tm/cTM3Matrix33.h"


// useful functions

void cIABoundTwoBoxes(cIAVector3 &b1, cIAVector3 &b2, cIAVector3 &b3) {

	// compute a box b3 which bounds the boxes b1 and b2

	if (b2.i[0].i[0]<b1.i[0].i[0]) b3.i[0].i[0]=b2.i[0].i[0]; else b3.i[0].i[0]=b1.i[0].i[0];
	if (b2.i[1].i[0]<b1.i[1].i[0]) b3.i[1].i[0]=b2.i[1].i[0]; else b3.i[1].i[0]=b1.i[1].i[0];
	if (b2.i[2].i[0]<b1.i[2].i[0]) b3.i[2].i[0]=b2.i[2].i[0]; else b3.i[2].i[0]=b1.i[2].i[0];

	if (b2.i[0].i[1]>b1.i[0].i[1]) b3.i[0].i[1]=b2.i[0].i[1]; else b3.i[0].i[1]=b1.i[0].i[1];
	if (b2.i[1].i[1]>b1.i[1].i[1]) b3.i[1].i[1]=b2.i[1].i[1]; else b3.i[1].i[1]=b1.i[1].i[1];
	if (b2.i[2].i[1]>b1.i[2].i[1]) b3.i[2].i[1]=b2.i[2].i[1]; else b3.i[2].i[1]=b1.i[2].i[1];

}

void cMultiply3x3Matrices(cIAMatrix33 *m1, cIAMatrix33 *m2, cIAMatrix33 *m3) {

	// m3=m1*m2

	m3->i[0][0].i[0]=m1->i[0][0].i[0]*m2->i[0][0].i[0]+m1->i[0][1].i[0]*m2->i[1][0].i[0]+m1->i[0][2].i[0]*m2->i[2][0].i[0];
	m3->i[0][1].i[0]=m1->i[0][0].i[0]*m2->i[0][1].i[0]+m1->i[0][1].i[0]*m2->i[1][1].i[0]+m1->i[0][2].i[0]*m2->i[2][1].i[0];
	m3->i[0][2].i[0]=m1->i[0][0].i[0]*m2->i[0][2].i[0]+m1->i[0][1].i[0]*m2->i[1][2].i[0]+m1->i[0][2].i[0]*m2->i[2][2].i[0];

	m3->i[1][0].i[0]=m1->i[1][0].i[0]*m2->i[0][0].i[0]+m1->i[1][1].i[0]*m2->i[1][0].i[0]+m1->i[1][2].i[0]*m2->i[2][0].i[0];
	m3->i[1][1].i[0]=m1->i[1][0].i[0]*m2->i[0][1].i[0]+m1->i[1][1].i[0]*m2->i[1][1].i[0]+m1->i[1][2].i[0]*m2->i[2][1].i[0];
	m3->i[1][2].i[0]=m1->i[1][0].i[0]*m2->i[0][2].i[0]+m1->i[1][1].i[0]*m2->i[1][2].i[0]+m1->i[1][2].i[0]*m2->i[2][2].i[0];

	m3->i[2][0].i[0]=m1->i[2][0].i[0]*m2->i[0][0].i[0]+m1->i[2][1].i[0]*m2->i[1][0].i[0]+m1->i[2][2].i[0]*m2->i[2][0].i[0];
	m3->i[2][1].i[0]=m1->i[2][0].i[0]*m2->i[0][1].i[0]+m1->i[2][1].i[0]*m2->i[1][1].i[0]+m1->i[2][2].i[0]*m2->i[2][1].i[0];
	m3->i[2][2].i[0]=m1->i[2][0].i[0]*m2->i[0][2].i[0]+m1->i[2][1].i[0]*m2->i[1][2].i[0]+m1->i[2][2].i[0]*m2->i[2][2].i[0];

}

// ******************
// interval functions
// ******************

void cIAMultiplyMatrix33PositionVector3(double t0, double t1, cIAMatrix33 *m, cVector3 *c, cVector3 *s, cIAVector3 *r) {

	// Bounds the product m.(c+t.s) by expanding the product

	// Compute m.c

	if (c->v[0]>0) { 
		r->i[0].i[0]=c->v[0]*m->i[0][0].i[0];r->i[0].i[1]=c->v[0]*m->i[0][0].i[1];
		r->i[1].i[0]=c->v[0]*m->i[1][0].i[0];r->i[1].i[1]=c->v[0]*m->i[1][0].i[1];
		r->i[2].i[0]=c->v[0]*m->i[2][0].i[0];r->i[2].i[1]=c->v[0]*m->i[2][0].i[1];
	}
	else { 
		r->i[0].i[0]=c->v[0]*m->i[0][0].i[1];r->i[0].i[1]=c->v[0]*m->i[0][0].i[0];
		r->i[1].i[0]=c->v[0]*m->i[1][0].i[1];r->i[1].i[1]=c->v[0]*m->i[1][0].i[0];
		r->i[2].i[0]=c->v[0]*m->i[2][0].i[1];r->i[2].i[1]=c->v[0]*m->i[2][0].i[0];
	}

	if (c->v[1]>0) { 
		r->i[0].i[0]+=c->v[1]*m->i[0][1].i[0];r->i[0].i[1]+=c->v[1]*m->i[0][1].i[1];
		r->i[1].i[0]+=c->v[1]*m->i[1][1].i[0];r->i[1].i[1]+=c->v[1]*m->i[1][1].i[1];
		r->i[2].i[0]+=c->v[1]*m->i[2][1].i[0];r->i[2].i[1]+=c->v[1]*m->i[2][1].i[1];
	}
	else { 
		r->i[0].i[0]+=c->v[1]*m->i[0][1].i[1];r->i[0].i[1]+=c->v[1]*m->i[0][1].i[0];
		r->i[1].i[0]+=c->v[1]*m->i[1][1].i[1];r->i[1].i[1]+=c->v[1]*m->i[1][1].i[0];
		r->i[2].i[0]+=c->v[1]*m->i[2][1].i[1];r->i[2].i[1]+=c->v[1]*m->i[2][1].i[0];
	}

	if (c->v[2]>0) { 
		r->i[0].i[0]+=c->v[2]*m->i[0][2].i[0];r->i[0].i[1]+=c->v[2]*m->i[0][2].i[1];
		r->i[1].i[0]+=c->v[2]*m->i[1][2].i[0];r->i[1].i[1]+=c->v[2]*m->i[1][2].i[1];
		r->i[2].i[0]+=c->v[2]*m->i[2][2].i[0];r->i[2].i[1]+=c->v[2]*m->i[2][2].i[1];
	}
	else { 
		r->i[0].i[0]+=c->v[2]*m->i[0][2].i[1];r->i[0].i[1]+=c->v[2]*m->i[0][2].i[0];
		r->i[1].i[0]+=c->v[2]*m->i[1][2].i[1];r->i[1].i[1]+=c->v[2]*m->i[1][2].i[0];
		r->i[2].i[0]+=c->v[2]*m->i[2][2].i[1];r->i[2].i[1]+=c->v[2]*m->i[2][2].i[0];
	}

	// Compute ms=m.s

	cIAVector3 ms;

	if (s->v[0]>0) { 
		ms.i[0].i[0]=s->v[0]*m->i[0][0].i[0];ms.i[0].i[1]=s->v[0]*m->i[0][0].i[1];
		ms.i[1].i[0]=s->v[0]*m->i[1][0].i[0];ms.i[1].i[1]=s->v[0]*m->i[1][0].i[1];
		ms.i[2].i[0]=s->v[0]*m->i[2][0].i[0];ms.i[2].i[1]=s->v[0]*m->i[2][0].i[1];
	}
	else { 
		ms.i[0].i[0]=s->v[0]*m->i[0][0].i[1];ms.i[0].i[1]=s->v[0]*m->i[0][0].i[0];
		ms.i[1].i[0]=s->v[0]*m->i[1][0].i[1];ms.i[1].i[1]=s->v[0]*m->i[1][0].i[0];
		ms.i[2].i[0]=s->v[0]*m->i[2][0].i[1];ms.i[2].i[1]=s->v[0]*m->i[2][0].i[0];
	}

	if (s->v[1]>0) { 
		ms.i[0].i[0]+=s->v[1]*m->i[0][1].i[0];ms.i[0].i[1]+=s->v[1]*m->i[0][1].i[1];
		ms.i[1].i[0]+=s->v[1]*m->i[1][1].i[0];ms.i[1].i[1]+=s->v[1]*m->i[1][1].i[1];
		ms.i[2].i[0]+=s->v[1]*m->i[2][1].i[0];ms.i[2].i[1]+=s->v[1]*m->i[2][1].i[1];
	}
	else { 
		ms.i[0].i[0]+=s->v[1]*m->i[0][1].i[1];ms.i[0].i[1]+=s->v[1]*m->i[0][1].i[0];
		ms.i[1].i[0]+=s->v[1]*m->i[1][1].i[1];ms.i[1].i[1]+=s->v[1]*m->i[1][1].i[0];
		ms.i[2].i[0]+=s->v[1]*m->i[2][1].i[1];ms.i[2].i[1]+=s->v[1]*m->i[2][1].i[0];
	}

	if (s->v[2]>0) { 
		ms.i[0].i[0]+=s->v[2]*m->i[0][2].i[0];ms.i[0].i[1]+=s->v[2]*m->i[0][2].i[1];
		ms.i[1].i[0]+=s->v[2]*m->i[1][2].i[0];ms.i[1].i[1]+=s->v[2]*m->i[1][2].i[1];
		ms.i[2].i[0]+=s->v[2]*m->i[2][2].i[0];ms.i[2].i[1]+=s->v[2]*m->i[2][2].i[1];
	}
	else { 
		ms.i[0].i[0]+=s->v[2]*m->i[0][2].i[1];ms.i[0].i[1]+=s->v[2]*m->i[0][2].i[0];
		ms.i[1].i[0]+=s->v[2]*m->i[1][2].i[1];ms.i[1].i[1]+=s->v[2]*m->i[1][2].i[0];
		ms.i[2].i[0]+=s->v[2]*m->i[2][2].i[1];ms.i[2].i[1]+=s->v[2]*m->i[2][2].i[0];
	}

	// compute mc+ms.t

	if (ms.i[0].i[0]>0) { r->i[0].i[0]+=t0*ms.i[0].i[0]; } else { r->i[0].i[0]+=t1*ms.i[0].i[0]; }
	if (ms.i[0].i[1]>0) { r->i[0].i[1]+=t1*ms.i[0].i[1]; } else { r->i[0].i[1]+=t0*ms.i[0].i[1]; }

	if (ms.i[1].i[0]>0) { r->i[1].i[0]+=t0*ms.i[1].i[0]; } else { r->i[1].i[0]+=t1*ms.i[1].i[0]; }
	if (ms.i[1].i[1]>0) { r->i[1].i[1]+=t1*ms.i[1].i[1]; } else { r->i[1].i[1]+=t0*ms.i[1].i[1]; }

	if (ms.i[2].i[0]>0) { r->i[2].i[0]+=t0*ms.i[2].i[0]; } else { r->i[2].i[0]+=t1*ms.i[2].i[0]; }
	if (ms.i[2].i[1]>0) { r->i[2].i[1]+=t1*ms.i[2].i[1]; } else { r->i[2].i[1]+=t0*ms.i[2].i[1]; }

}

void cConstrainRotationMatrix(cIAMatrix33 *m) {

	// use properties of a 3x3 orthonormal matrix to constrain the interval matrix

	if (m->i[0][0].i[0]<-1.0) m->i[0][0].i[0]=-1.0;if (m->i[0][0].i[1]>1.0) m->i[0][0].i[1]=1.0;
	if (m->i[0][1].i[0]<-1.0) m->i[0][1].i[0]=-1.0;if (m->i[0][1].i[1]>1.0) m->i[0][1].i[1]=1.0;
	if (m->i[0][2].i[0]<-1.0) m->i[0][2].i[0]=-1.0;if (m->i[0][2].i[1]>1.0) m->i[0][2].i[1]=1.0;

	if (m->i[1][0].i[0]<-1.0) m->i[1][0].i[0]=-1.0;if (m->i[1][0].i[1]>1.0) m->i[1][0].i[1]=1.0;
	if (m->i[1][1].i[0]<-1.0) m->i[1][1].i[0]=-1.0;if (m->i[1][1].i[1]>1.0) m->i[1][1].i[1]=1.0;
	if (m->i[1][2].i[0]<-1.0) m->i[1][2].i[0]=-1.0;if (m->i[1][2].i[1]>1.0) m->i[1][2].i[1]=1.0;

	if (m->i[2][0].i[0]<-1.0) m->i[2][0].i[0]=-1.0;if (m->i[2][0].i[1]>1.0) m->i[2][0].i[1]=1.0;
	if (m->i[2][1].i[0]<-1.0) m->i[2][1].i[0]=-1.0;if (m->i[2][1].i[1]>1.0) m->i[2][1].i[1]=1.0;
	if (m->i[2][2].i[0]<-1.0) m->i[2][2].i[0]=-1.0;if (m->i[2][2].i[1]>1.0) m->i[2][2].i[1]=1.0;

}

void cConstrainRotationMatrixTM3(cTM3Matrix33 *m) {

	// use properties of a 3x3 orthonormal matrix to constrain the interval matrix

	if (m->i[0][0].r.i[0]<-1.0) m->i[0][0].r.i[0]=-1.0;if (m->i[0][0].r.i[1]>1.0) m->i[0][0].r.i[1]=1.0;
	if (m->i[0][1].r.i[0]<-1.0) m->i[0][1].r.i[0]=-1.0;if (m->i[0][1].r.i[1]>1.0) m->i[0][1].r.i[1]=1.0;
	if (m->i[0][2].r.i[0]<-1.0) m->i[0][2].r.i[0]=-1.0;if (m->i[0][2].r.i[1]>1.0) m->i[0][2].r.i[1]=1.0;

	if (m->i[1][0].r.i[0]<-1.0) m->i[1][0].r.i[0]=-1.0;if (m->i[1][0].r.i[1]>1.0) m->i[1][0].r.i[1]=1.0;
	if (m->i[1][1].r.i[0]<-1.0) m->i[1][1].r.i[0]=-1.0;if (m->i[1][1].r.i[1]>1.0) m->i[1][1].r.i[1]=1.0;
	if (m->i[1][2].r.i[0]<-1.0) m->i[1][2].r.i[0]=-1.0;if (m->i[1][2].r.i[1]>1.0) m->i[1][2].r.i[1]=1.0;

	if (m->i[2][0].r.i[0]<-1.0) m->i[2][0].r.i[0]=-1.0;if (m->i[2][0].r.i[1]>1.0) m->i[2][0].r.i[1]=1.0;
	if (m->i[2][1].r.i[0]<-1.0) m->i[2][1].r.i[0]=-1.0;if (m->i[2][1].r.i[1]>1.0) m->i[2][1].r.i[1]=1.0;
	if (m->i[2][2].r.i[0]<-1.0) m->i[2][2].r.i[0]=-1.0;if (m->i[2][2].r.i[1]>1.0) m->i[2][2].r.i[1]=1.0;

	if ((m->i[0][0].r.i[0]==-1.0)&&(m->i[0][0].r.i[1]==1.0)) m->i[0][0].c[0]=m->i[0][0].c[1]=m->i[0][0].c[2]=m->i[0][0].c[3]=0.0;
	if ((m->i[0][1].r.i[0]==-1.0)&&(m->i[0][1].r.i[1]==1.0)) m->i[0][1].c[0]=m->i[0][1].c[1]=m->i[0][1].c[2]=m->i[0][1].c[3]=0.0;
	if ((m->i[0][2].r.i[0]==-1.0)&&(m->i[0][2].r.i[1]==1.0)) m->i[0][2].c[0]=m->i[0][2].c[1]=m->i[0][2].c[2]=m->i[0][2].c[3]=0.0;

	if ((m->i[1][0].r.i[0]==-1.0)&&(m->i[1][0].r.i[1]==1.0)) m->i[1][0].c[0]=m->i[1][0].c[1]=m->i[1][0].c[2]=m->i[1][0].c[3]=0.0;
	if ((m->i[1][1].r.i[0]==-1.0)&&(m->i[1][1].r.i[1]==1.0)) m->i[1][1].c[0]=m->i[1][1].c[1]=m->i[1][1].c[2]=m->i[1][1].c[3]=0.0;
	if ((m->i[1][2].r.i[0]==-1.0)&&(m->i[1][2].r.i[1]==1.0)) m->i[1][2].c[0]=m->i[1][2].c[1]=m->i[1][2].c[2]=m->i[1][2].c[3]=0.0;

	if ((m->i[2][0].r.i[0]==-1.0)&&(m->i[2][0].r.i[1]==1.0)) m->i[2][0].c[0]=m->i[2][0].c[1]=m->i[2][0].c[2]=m->i[2][0].c[3]=0.0;
	if ((m->i[2][1].r.i[0]==-1.0)&&(m->i[2][1].r.i[1]==1.0)) m->i[2][1].c[0]=m->i[2][1].c[1]=m->i[2][1].c[2]=m->i[2][1].c[3]=0.0;
	if ((m->i[2][2].r.i[0]==-1.0)&&(m->i[2][2].r.i[1]==1.0)) m->i[2][2].c[0]=m->i[2][2].c[1]=m->i[2][2].c[2]=m->i[2][2].c[3]=0.0;

}



