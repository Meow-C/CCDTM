#include <math.h>
#include <memory.h>

#include "fcl/tm/cTM2Vector3.h"
#include "fcl/tm/cInterval.h"
#include "fcl/tm/cVector3.h"
#include "fcl/tm/cIAVector3.h"

cTM2Vector3::cTM2Vector3() {}
cTM2Vector3::cTM2Vector3(cTaylorModel2 val[3]) { memcpy(i, val, 3 * sizeof(cTaylorModel2)); }
cTM2Vector3::cTM2Vector3(cTaylorModel2& nx, cTaylorModel2& ny, cTaylorModel2& nz) { i[0] = nx; i[1] = ny; i[2] = nz; }
cTM2Vector3::cTM2Vector3(cVector3& u) { i[0] = cTaylorModel2(u.v[0]); i[1] = cTaylorModel2(u.v[1]); i[2] = cTaylorModel2(u.v[2]); }

void cTM2Vector3::setZero() { memset(this, 0, sizeof(cTM2Vector3)); }

// operators

cTM2Vector3 cTM2Vector3::operator+(const cTM2Vector3& u) { cTaylorModel2 res[3]; res[0] = i[0] + u.i[0]; res[1] = i[1] + u.i[1]; res[2] = i[2] + u.i[2]; return cTM2Vector3(res); }
cTM2Vector3 cTM2Vector3::operator+(const double& d) { cTaylorModel2 res[3]; res[0] = i[0]; res[1] = i[1]; res[2] = i[2]; res[0].c[0] += d; return cTM2Vector3(res); }
cTM2Vector3 cTM2Vector3::operator-(const cTM2Vector3& u) { cTaylorModel2 res[3]; res[0] = i[0] - u.i[0]; res[1] = i[1] - u.i[1]; res[2] = i[2] - u.i[2]; return cTM2Vector3(res); }
cTM2Vector3& cTM2Vector3::operator+=(const cTM2Vector3& u) { i[0] += u.i[0]; i[1] += u.i[1]; i[2] += u.i[2]; return *this; }
cTM2Vector3& cTM2Vector3::operator-=(const cTM2Vector3& u) { i[0] -= u.i[0]; i[1] -= u.i[1]; i[2] -= u.i[2]; return *this; }
cTM2Vector3& cTM2Vector3::operator=(const cVector3& u) { i[0] = cTaylorModel2(u.v[0]); i[1] = cTaylorModel2(u.v[1]); i[2] = cTaylorModel2(u.v[2]); return *this; }
cTaylorModel2 cTM2Vector3::operator|(const cTM2Vector3& u) { return i[0] * u.i[0] + i[1] * u.i[1] + i[2] * u.i[2]; } // interval dot product
cTM2Vector3 cTM2Vector3::operator^(const cTM2Vector3& u) { cTaylorModel2 res[3]; res[0] = i[1] * u.i[2] - i[2] * u.i[1]; res[1] = i[2] * u.i[0] - i[0] * u.i[2]; res[2] = i[0] * u.i[1] - i[1] * u.i[0]; return cTM2Vector3(res); } // interval cross product
double cTM2Vector3::volume() { return i[0].bound().diameter() * i[1].bound().diameter() * i[2].bound().diameter(); }

cIAVector3 cTM2Vector3::bound() {

	cInterval res[3];
	res[0] = i[0].bound();
	res[1] = i[1].bound();
	res[2] = i[2].bound();

	return cIAVector3(res);

}

void cTM2Vector3::print() {

	i[0].print();
	i[1].print();
	i[2].print();
	std::cout << std::endl;

}

cIAVector3 cTM2Vector3::evaluate(double t) {

	cInterval res[3];
	res[0] = i[0].evaluate(t);
	res[1] = i[1].evaluate(t);
	res[2] = i[2].evaluate(t);

	return cIAVector3(res);

}

cTaylorModel2 cTM2Vector3::dot(cTM2Vector3 other) {
//    cTM2Vector3 res(i[0] * other.i[0], i[1] * other.i[1], i[2] * other.i[2]);
    cTaylorModel2 res = i[0] * other.i[0] + i[1] * other.i[1] + i[2] * other.i[2];

    return res;
}

cTM2Vector3 operator*(cTaylorModel2& d, cTM2Vector3& v) {

	cTaylorModel2 res[3];
	res[0] = d * v.i[0];
	res[1] = d * v.i[1];
	res[2] = d * v.i[2];

	return cTM2Vector3(res);

}

cTM2Vector3 operator*(double d, cTM2Vector3& v) {

	cTaylorModel2 res[3];
	res[0] = d * v.i[0];
	res[1] = d * v.i[1];
	res[2] = d * v.i[2];

	return cTM2Vector3(res);

}

void cTM2Vector3::linearModel(cVector3& position, cVector3& velocity) {

	i[0].linearModel(position.v[0], velocity.v[0]);
	i[1].linearModel(position.v[1], velocity.v[1]);
	i[2].linearModel(position.v[2], velocity.v[2]);

}

