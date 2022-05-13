#include "fcl/tm/cTaylorModel2.h"
#include "fcl/tm/ModelParameters.h"
#include <iostream>
#include <math.h>
#include <memory.h>

cTaylorModel2::cTaylorModel2() { ; }
cTaylorModel2::cTaylorModel2(double v) {
    c[0] = v;
    c[1] = c[2] = r.i[0] = r.i[1] = 0;
}
cTaylorModel2::cTaylorModel2(double cc[], cInterval& rr) {
    c[0] = cc[0];
    c[1] = cc[1];
    c[2] = cc[2];
    r = rr;
}
cTaylorModel2::cTaylorModel2(double c0, double c1, double c2, cInterval rr) {
    c[0] = c0;
    c[1] = c1;
    c[2] = c2;
    r = rr;
}

cTaylorModel2 cTaylorModel2::operator+(const cTaylorModel2& in) {
    return cTaylorModel2(c[0] + in.c[0], c[1] + in.c[1], c[2] + in.c[2], r + in.r);
}
cTaylorModel2 cTaylorModel2::operator-(const cTaylorModel2& in) {
    return cTaylorModel2(c[0] - in.c[0], c[1] - in.c[1], c[2] - in.c[2], r - in.r);
}
cTaylorModel2& cTaylorModel2::operator+=(const cTaylorModel2& in) {
    c[0] += in.c[0];
    c[1] += in.c[1];
    c[2] += in.c[2];
    r += in.r;
    return *this;
}
cTaylorModel2& cTaylorModel2::operator-=(const cTaylorModel2& in) {
    c[0] -= in.c[0];
    c[1] -= in.c[1];
    c[2] -= in.c[2];
    r -= in.r;
    return *this;
}
cTaylorModel2 cTaylorModel2::operator*(const cTaylorModel2& in) {
    // NB: we assume the values of globalTimeInterval, globalTimeInterval2, ...
    // are up-to-date

    register double c0, c1, c2;
    register double inc0 = in.c[0], inc1 = in.c[1], inc2 = in.c[2];
    cInterval inr(in.r);

    c0 = c[0] * inc0;
    c1 = c[0] * inc1 + c[1] * inc0;
    c2 = c[0] * inc2 + c[1] * inc1 + c[2] * inc0;

    cInterval remainder(r * inr);
    register double tempVal = c[1] * inc2 + c[2] * inc1;
    if (tempVal >= 0) {
        remainder.i[0] += tempVal * globalTimeInterval3.i[0];
        remainder.i[1] += tempVal * globalTimeInterval3.i[1];
    }
    else {
        remainder.i[1] += tempVal * globalTimeInterval3.i[0];
        remainder.i[0] += tempVal * globalTimeInterval3.i[1];
    }

    tempVal = c[2] * inc2 + c[2] * inc2;
    if (tempVal >= 0) {
        remainder.i[0] += tempVal * globalTimeInterval4.i[0];
        remainder.i[1] += tempVal * globalTimeInterval4.i[1];
    }
    else {
        remainder.i[1] += tempVal * globalTimeInterval4.i[0];
        remainder.i[0] += tempVal * globalTimeInterval4.i[1];
    }

    remainder += (cInterval(c[0]) + c[1] * globalTimeInterval + c[2] * globalTimeInterval2) * inr
               + (cInterval(inc0) + inc1 * globalTimeInterval + inc2 * globalTimeInterval2) * r;

    return cTaylorModel2(c0, c1, c2, remainder);
}
cTaylorModel2 cTaylorModel2::operator*(const double d) {
    return cTaylorModel2(c[0] * d, c[1] * d, c[2] * d, r * d);
}
cTaylorModel2 cTaylorModel2::operator-() {
    return cTaylorModel2(-c[0], -c[1], -c[2], -r);
}

void cTaylorModel2::print() {
    std::cout << c[0] << (c[1] < 0 ? "" : "+") << c[1] << (c[2] < 0 ? "*t" : "*t+") << c[2] << "*t^2+"
        << "[" << r.i[0] << "," << r.i[1] << "] over ["
        << globalTimeInterval.i[0] << "," << globalTimeInterval.i[1] << "]\n";
    std::cout << "\n";
}

cInterval cTaylorModel2::bound() {
    //	return
    //cInterval(c[0],c[0])+c[1]*globalTimeInterval+c[2]*globalTimeInterval2+r;

#if 1

    if (!c[2])
        return cInterval(c[0] + r.i[0], c[0] + r.i[1]) +
               globalTimeInterval * c[1];

    // compute actual bounds of the quadratic

    // monotonous
    cInterval bounds = this->evaluate(globalTimeInterval.i[0]);
    cInterval bounds_bound = this->evaluate(globalTimeInterval.i[1]);
    //bounds.bound(this->evaluate(globalTimeInterval.i[1]));
    bounds.bound(bounds_bound);

    // x=-c[1]/c[2]
    double x_mid = -c[1]/c[2];

    if (globalTimeInterval.i[0] < x_mid && x_mid < globalTimeInterval.i[1]) {
        cInterval bounds = this->evaluate(x_mid);
        cInterval bounds_bound = this->evaluate(globalTimeInterval.i[0]);
        bounds.bound(bounds_bound);
        bounds_bound = this->evaluate(globalTimeInterval.i[1]);
        bounds.bound(bounds_bound);
    }

    return bounds;

#else

    return cInterval(c[0] + r.i[0], c[0] + r.i[1]) + globalTimeInterval * (c[1] + globalTimeInterval * c[2]);

#endif
}

cInterval cTaylorModel2::bound(double t0, double t1) {
    cInterval customTimeInterval(t0, t1);
    cInterval customTimeInterval2(
        customTimeInterval.i[0] * customTimeInterval.i[0],
        customTimeInterval.i[1] * customTimeInterval.i[1]);

    return cInterval(c[0], c[0]) + c[1] * customTimeInterval +
        c[2] * customTimeInterval2 + r;
}

cInterval cTaylorModel2::evaluate(double t) {
    return cInterval(c[0] + t * (c[1] + t * c[2])) + r;
}

void cTaylorModel2::setZero() { memset(this, 0, sizeof(cTaylorModel2)); }

void cTaylorModel2::cosModel(double w, double q0) {
    double a = 0.5 * (globalTimeInterval.i[0] + globalTimeInterval.i[1]);
    double t = w * a + q0;
    double w2 = w * w;
    double fa = cos(t);
    double fda = -w * sin(t);
    double fdda = -w2 * fa;

    c[0] = fa - a * (fda - 0.5 * a * fdda);
    c[1] = fda - a * fdda;
    c[2] = 0.5 * fdda;

    // compute bounds on the fourth derivative w^4*cos(w*t+q0)

    cInterval fdddBounds;

    if (!w)
        fdddBounds.i[0] = fdddBounds.i[1] = 0.0;
    else if (w > 0) {
        double sinQL = sin(q0 + globalTimeInterval.i[0] * w);
        double sinQR = sin(q0 + globalTimeInterval.i[1] * w);

        if (sinQL < sinQR) {
            fdddBounds.i[0] = sinQL;
            fdddBounds.i[1] = sinQR;
        }
        else {
            fdddBounds.i[1] = sinQL;
            fdddBounds.i[0] = sinQR;
        }

        // enlarge to counter round-off errors

        fdddBounds.i[0] -= 1e-15;
        fdddBounds.i[1] += 1e-15;

        // sin reaches a maximum if there exists an integer k in
        // [(q0-pi/2)/(2pi),(q0-pi/2+w)/(2pi)] sin reaches a minimum if there exists
        // an integer k in [(q0-3pi/2)/(2pi),(q0-3pi/2+w)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double min = 0.5 * q0 / CTM3_PI;

        min -= 0.25;
        if (ceil(min) <= min + step) fdddBounds.i[1] = 1.0;
        min -= 0.5;
        if (ceil(min) <= min + step) fdddBounds.i[0] = -1.0;

    }
    else {
        double sinQL = sin(q0 + globalTimeInterval.i[0] * w);
        double sinQR = sin(q0 + globalTimeInterval.i[1] * w);

        if (sinQL < sinQR)
            fdddBounds = cInterval(sinQL, sinQR);
        else
            fdddBounds = cInterval(sinQR, sinQL);

        // sin reaches a maximum if there exists an integer k in
        // [(q0-pi/2+w)/(2pi),(q0-pi/2)/(2pi)] sin reaches a minimum if there exists
        // an integer k in [(q0-3pi/2+w)/(2pi),(q0-3pi/2)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double max = 0.5 * q0 / CTM3_PI;

        max -= 0.25;
        if (floor(max) >= max + step) fdddBounds.i[1] = 1.0;
        max -= 0.5;
        if (floor(max) >= max + step) fdddBounds.i[0] = -1.0;
    }

    double w3 = w * w2;
    fdddBounds.i[0] *= w3;
    fdddBounds.i[1] *= w3;

    double rightSize = 0.5 * (globalTimeInterval.i[1] - globalTimeInterval.i[0]);
    double rightSize3 = rightSize * rightSize * rightSize;
    double leftSize3 = -rightSize3;
    cInterval Size(leftSize3, rightSize3);

    r = 1./6.0 * fdddBounds * Size;
}

void cTaylorModel2::sinModel(double w, double q0) {
    double a = 0.5 * (globalTimeInterval.i[0] + globalTimeInterval.i[1]);
    double t = w * a + q0;
    double w2 = w * w;
    double fa = sin(t);
    double fda = w * cos(t);
    double fdda = -w2 * fa;

    c[0] = fa - a * (fda - 0.5 * a * fdda);
    c[1] = fda - a * fdda;
    c[2] = 0.5 * fdda;

    // compute bounds on the fourth derivative w^4*sin(w*t+q0)

    cInterval fdddBounds;

    if (!w)
        fdddBounds.i[0] = fdddBounds.i[1] = 0.0;
    else if (w > 0) {
        double cosQL = cos(q0 + globalTimeInterval.i[0] * w);
        double cosQR = cos(q0 + globalTimeInterval.i[1] * w);

        if (cosQL < cosQR)
            fdddBounds = cInterval(cosQL, cosQR);
        else
            fdddBounds = cInterval(cosQR, cosQL);

        // enlarge to counter round-off errors

        fdddBounds.i[0] -= 1e-15;
        fdddBounds.i[1] += 1e-15;

        // cos reaches a maximum if there exists an integer k in
        // [(q0)/(2pi),(q0+w)/(2pi)] cos reaches a minimum if there exists an
        // integer k in [(q0-pi)/(2pi),(q0-pi+w)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double min = 0.5 * q0 / CTM3_PI;

        if (ceil(min) <= min + step) fdddBounds.i[1] = 1.0;
        min -= 0.5;
        if (ceil(min) <= min + step) fdddBounds.i[0] = -1.0;

    }
    else {
        double cosQL = cos(q0 + globalTimeInterval.i[0] * w);
        double cosQR = cos(q0 + globalTimeInterval.i[1] * w);

        if (cosQL < cosQR) {
            fdddBounds.i[0] = cosQL;
            fdddBounds.i[1] = cosQR;
        }
        else {
            fdddBounds.i[1] = cosQL;
            fdddBounds.i[0] = cosQR;
        }

        // cos reaches a maximum if there exists an integer k in
        // [(q0+w)/(2pi),(q0)/(2pi)] cos reaches a minimum if there exists an
        // integer k in [(q0-pi+w)/(2pi),(q0-pi)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double max = 0.5 * q0 / CTM3_PI;

        if (floor(max) >= max + step) fdddBounds.i[1] = 1.0;
        max -= 0.5;
        if (floor(max) >= max + step) fdddBounds.i[0] = -1.0;
    }

    double w3 = w * w2;
    fdddBounds = fdddBounds * -w3;

    double rightSize = 0.5 * (globalTimeInterval.i[1] - globalTimeInterval.i[0]);
    double rightSize3 = rightSize * rightSize * rightSize;
    double leftSize3 = -rightSize3;
    cInterval Size(leftSize3, rightSize3);

    r = 1./6.0 * fdddBounds * Size;
}

void cTaylorModel2::linearModel(double p, double v) {
    c[0] = p;
    c[1] = v;
    c[2] = r.i[0] = r.i[1] = 0.0;
}

cTaylorModel2 operator*(double d, cTaylorModel2& tm) {
    return cTaylorModel2(d * tm.c[0], d * tm.c[1], d * tm.c[2], d * tm.r);
}