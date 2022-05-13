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

#include "fcl/tm/cMatrix33.h"
#include "fcl/tm/cVector3.h"

cMatrix33::cMatrix33() {
	m[0][0]=m[0][1]=m[0][2]=m[1][0]=m[1][1]=m[1][2]=m[2][0]=m[2][1]=m[2][2]=0.0;
}	//:) [0,0,0;0,0,0;0,0,0]

cMatrix33::cMatrix33(double v) {
	m[0][0]=m[0][1]=m[0][2]=m[1][0]=m[1][1]=m[1][2]=m[2][0]=m[2][1]=m[2][2]=v;
}	//:) [v,v,v;v,v,v;v,v,v]

cMatrix33::cMatrix33(cVector3 d) {
	m[0][1]=m[0][2]=m[1][0]=m[1][2]=m[2][0]=m[2][1]=0.0;
	m[0][0]=d.v[0];m[1][1]=d.v[1];m[2][2]=d.v[2];
}	//:) [d[0],0,0;0,d[1],0;0,0,d[3]] diagonal

cMatrix33::cMatrix33(double mat[3][3]) {
	m[0][0]=mat[0][0];m[0][1]=mat[0][1];m[0][2]=mat[0][2];
	m[1][0]=mat[1][0];m[1][1]=mat[1][1];m[1][2]=mat[1][2];
	m[2][0]=mat[2][0];m[2][1]=mat[2][1];m[2][2]=mat[2][2];
}	

cMatrix33::cMatrix33(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22) {
	m[0][0]=m00;m[0][1]=m01;m[0][2]=m02;
	m[1][0]=m10;m[1][1]=m11;m[1][2]=m12;
	m[2][0]=m20;m[2][1]=m21;m[2][2]=m22;
}

cMatrix33::cMatrix33(cVector3 v0, cVector3 v1, cVector3 v2) {
	m[0][0]=v0.v[0];m[1][0]=v0.v[1];m[2][0]=v0.v[2];
	m[0][1]=v1.v[0];m[1][1]=v1.v[1];m[2][1]=v1.v[2];
	m[0][2]=v2.v[0];m[1][2]=v2.v[1];m[2][2]=v2.v[2];
}	//:) [v0,v1,v2] v:column vector

void cMatrix33::setIdentity() { 
	m[0][1]=m[0][2]=m[1][0]=m[1][2]=m[2][0]=m[2][1]=0.0;
	m[0][0]=m[1][1]=m[2][2]=1.0;
}	//:) identity matrix

void cMatrix33::setZero() { 
	m[0][1]=m[0][2]=m[1][0]=m[1][2]=m[2][0]=m[2][1]=m[0][0]=m[1][1]=m[2][2]=0.0;
}	//:) zero

cMatrix33 cMatrix33::operator+(cMatrix33& mat) {
	
	return cMatrix33(m[0][0]+mat.m[0][0],m[0][1]+mat.m[0][1],m[0][2]+mat.m[0][2],
		m[1][0]+mat.m[1][0],m[1][1]+mat.m[1][1],m[1][2]+mat.m[1][2],
		m[2][0]+mat.m[2][0],m[2][1]+mat.m[2][1],m[2][2]+mat.m[2][2]);
	
}

void cMatrix33::operator+=(cMatrix33 &mat) {
	
	m[0][0]+=mat.m[0][0];m[0][1]+=mat.m[0][1];m[0][2]+=mat.m[0][2];
	m[1][0]+=mat.m[1][0];m[1][1]+=mat.m[1][1];m[1][2]+=mat.m[1][2];
	m[2][0]+=mat.m[2][0];m[2][1]+=mat.m[2][1];m[2][2]+=mat.m[2][2];
	
}

cMatrix33 cMatrix33::operator*(cMatrix33& mat) const {
	
	double res[3][3];
	res[0][0]=m[0][0]*mat.m[0][0]+m[0][1]*mat.m[1][0]+m[0][2]*mat.m[2][0];
	res[0][1]=m[0][0]*mat.m[0][1]+m[0][1]*mat.m[1][1]+m[0][2]*mat.m[2][1];
	res[0][2]=m[0][0]*mat.m[0][2]+m[0][1]*mat.m[1][2]+m[0][2]*mat.m[2][2];
	
	res[1][0]=m[1][0]*mat.m[0][0]+m[1][1]*mat.m[1][0]+m[1][2]*mat.m[2][0];
	res[1][1]=m[1][0]*mat.m[0][1]+m[1][1]*mat.m[1][1]+m[1][2]*mat.m[2][1];
	res[1][2]=m[1][0]*mat.m[0][2]+m[1][1]*mat.m[1][2]+m[1][2]*mat.m[2][2];
	
	res[2][0]=m[2][0]*mat.m[0][0]+m[2][1]*mat.m[1][0]+m[2][2]*mat.m[2][0];
	res[2][1]=m[2][0]*mat.m[0][1]+m[2][1]*mat.m[1][1]+m[2][2]*mat.m[2][1];
	res[2][2]=m[2][0]*mat.m[0][2]+m[2][1]*mat.m[1][2]+m[2][2]*mat.m[2][2];
	
	return cMatrix33(res);
	
}	//:) matrix multiplication

cMatrix33 cMatrix33::getTranspose() const {
	
	double res[3][3];
	res[0][0]=m[0][0];res[0][1]=m[1][0];res[0][2]=m[2][0];
	res[1][0]=m[0][1];res[1][1]=m[1][1];res[1][2]=m[2][1];
	res[2][0]=m[0][2];res[2][1]=m[1][2];res[2][2]=m[2][2];
	
	return cMatrix33(res);
	
}	//:) matrix transpose

cVector3 cMatrix33::operator*(cVector3 &vec) {
	
	double res[3];
	res[0]=m[0][0]*vec.v[0]+m[0][1]*vec.v[1]+m[0][2]*vec.v[2];
	res[1]=m[1][0]*vec.v[0]+m[1][1]*vec.v[1]+m[1][2]*vec.v[2];
	res[2]=m[2][0]*vec.v[0]+m[2][1]*vec.v[1]+m[2][2]*vec.v[2];
	
	return cVector3(res);
	
}	//:) matrix33 * vector = vector

void cMatrix33::operator*=(double d) {
	
	m[0][0]*=d;m[0][1]*=d;m[0][2]*=d;
	m[1][0]*=d;m[1][1]*=d;m[1][2]*=d;
	m[2][0]*=d;m[2][1]*=d;m[2][2]*=d;
	
}	//:) matrix * constant = matrix

cMatrix33 cMatrix33::operator*(double d) const	{ return cMatrix33(m[0][0]*d,m[0][1]*d,m[0][2]*d,m[1][0]*d,m[1][1]*d,m[1][2]*d,m[2][0]*d,m[2][1]*d,m[2][2]*d); }

void cMatrix33::print()	{
	
	for (int i=0;i<3;i++) {
		
		for (int j=0;j<3;j++) std::cout << "\t[" << m[i][j] << "]";
		std::cout << std::endl;
		
	}
	
	std::cout << std::endl;
	std::cout << std::endl;
	
}

#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);a[k][l]=h+s*(g-h*tau);
#define cdfabs(x) ((x<0)? -x : x)

void cMatrix33::diagonalize(cVector3 &ev, cMatrix33 &p) {

	// NB : M=Pdiag(e)PINVERSE (et PINVERSE=PTRANSPOSE) 
	// NB : From Numerical Recipes

	int nrot; // number of Jacobi rotations
	int i,j,ip,iq;
	double tresh, theta,tau,t,sm,s,h,g,c;

	double a[3][3]; 
	double v[3][3]; 
	double d[3]; 
	double b[3];
	double z[3];

	// initialization

	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			a[i][j]=m[i][j];
			v[i][j]=0.0;
		}
		v[i][i]=1.0;
	}

	//:) a=m, v identity matrix

	for (i=0;i<3;i++) {
		b[i]=a[i][i];
		d[i]=a[i][i];
		z[i]=0.0;
	}

	// b=d, diagonal value of a, z=0

	nrot=0;

	for (i=1;i<=50;i++) {

		sm=fabs(a[0][1])+fabs(a[0][2])+fabs(a[1][2]);

		if (sm==0.0) { 
			
			for (ip=0;ip<3;ip++) {
				for (iq=0;iq<3;iq++) p.m[ip][iq]=v[ip][iq];
				ev.v[ip]=d[ip];
			}
		
			return;

		}

		if (i<4) tresh=0.2*sm/9.0; 
		else tresh=0.0;

		for (ip=0;ip<2;ip++) {
			for (iq=ip+1;iq<3;iq++) {
				g=100.0*cdfabs(a[ip][iq]);
				if ((i>4)&&(cdfabs(d[ip]+g)==cdfabs(d[ip]))&&(cdfabs(d[iq]+g)==cdfabs(d[iq]))) 
					a[ip][iq]=0;
				else if (cdfabs(a[ip][iq])>tresh) {
					h=d[iq]-d[ip];
					if ((cdfabs(h)+g)==(cdfabs(h))) t=a[ip][iq]/h;
					else {
						theta=0.5*h/(a[ip][iq]);
						t=1.0/(cdfabs(theta)+sqrt(1.0+theta*theta));
						if (theta<0.0) t=-t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a[ip][iq];
					z[ip]-=h;
					z[iq]+=h;
					d[ip]-=h;
					d[iq]+=h;
					a[ip][iq]=0.0;
					for (j=0;j<=(ip-1);j++) {
						ROTATE(a,j,ip,j,iq)
					}
					for (j=(ip+1);j<=(iq-1);j++) {
						ROTATE(a,ip,j,j,iq)
					}
					for (j=(iq+1);j<3;j++) {
						ROTATE(a,ip,j,iq,j)
					}
					for (j=0;j<3;j++) {
						ROTATE(v,j,ip,j,iq)
					}
					nrot++;
				}
			}
		}
		for (ip=0;ip<3;ip++) {
			b[ip]+=z[ip];
			d[ip]=b[ip];
			z[ip]=0.0;
		}
	}

	std::cout << "Error in diagonalization" << std::endl;

}

