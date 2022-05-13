#include "fcl/tm/cTaylorModel3.h"
#include "fcl/tm/ModelParameters.h"

#include <assert.h>
#include <iostream>
#include <math.h>
#include <memory.h>

cInterval globalTimeInterval;
cInterval globalTimeInterval2;
cInterval globalTimeInterval3;
cInterval globalTimeInterval4;
cInterval globalTimeInterval5;
cInterval globalTimeInterval6;

cTaylorModel3::cTaylorModel3() { ; }
cTaylorModel3::cTaylorModel3(double v) {
    c[0] = v;
    c[1] = c[2] = c[3] = r.i[0] = r.i[1] = 0;
}
cTaylorModel3::cTaylorModel3(double cc[], cInterval& rr) {
    c[0] = cc[0];
    c[1] = cc[1];
    c[2] = cc[2];
    c[3] = cc[3];
    r = rr;
}
cTaylorModel3::cTaylorModel3(double c0, double c1, double c2, double c3,
    cInterval rr) {
    c[0] = c0;
    c[1] = c1;
    c[2] = c2;
    c[3] = c3;
    r = rr;
}

cTaylorModel3 cTaylorModel3::operator+(const cTaylorModel3& in) {
    return cTaylorModel3(c[0] + in.c[0], c[1] + in.c[1], c[2] + in.c[2],
        c[3] + in.c[3], r + in.r);
}
cTaylorModel3 cTaylorModel3::operator-(const cTaylorModel3& in) {
    return cTaylorModel3(c[0] - in.c[0], c[1] - in.c[1], c[2] - in.c[2],
        c[3] - in.c[3], r - in.r);
}
cTaylorModel3& cTaylorModel3::operator+=(const cTaylorModel3& in) {
    c[0] += in.c[0];
    c[1] += in.c[1];
    c[2] += in.c[2];
    c[3] += in.c[3];
    r += in.r;
    return *this;
}
cTaylorModel3& cTaylorModel3::operator-=(const cTaylorModel3& in) {
    c[0] -= in.c[0];
    c[1] -= in.c[1];
    c[2] -= in.c[2];
    c[3] -= in.c[3];
    r -= in.r;
    return *this;
}
cTaylorModel3 cTaylorModel3::operator*(const cTaylorModel3& in) {
    // NB: we assume the values of globalTimeInterval, globalTimeInterval2, ...
    // are up-to-date

    register double c0, c1, c2, c3;
    register double inc0 = in.c[0], inc1 = in.c[1], inc2 = in.c[2],
        inc3 = in.c[3];
    cInterval inr(in.r);

    c0 = c[0] * inc0;
    c1 = c[0] * inc1 + c[1] * inc0;
    c2 = c[0] * inc2 + c[1] * inc1 + c[2] * inc0;
    c3 = c[0] * inc3 + c[1] * inc2 + c[2] * inc1 + c[3] * inc0;

    cInterval remainder(r * inr);
    register double tempVal = c[1] * inc3 + c[2] * inc2 + c[3] * inc1;
    if (tempVal >= 0) {
        remainder.i[0] += tempVal * globalTimeInterval4.i[0];
        remainder.i[1] += tempVal * globalTimeInterval4.i[1];
    }
    else {
        remainder.i[1] += tempVal * globalTimeInterval4.i[0];
        remainder.i[0] += tempVal * globalTimeInterval4.i[1];
    }

    tempVal = c[2] * inc3 + c[3] * inc2;
    if (tempVal >= 0) {
        remainder.i[0] += tempVal * globalTimeInterval5.i[0];
        remainder.i[1] += tempVal * globalTimeInterval5.i[1];
    }
    else {
        remainder.i[1] += tempVal * globalTimeInterval5.i[0];
        remainder.i[0] += tempVal * globalTimeInterval5.i[1];
    }

    tempVal = c[3] * inc3;
    if (tempVal >= 0) {
        remainder.i[0] += tempVal * globalTimeInterval6.i[0];
        remainder.i[1] += tempVal * globalTimeInterval6.i[1];
    }
    else {
        remainder.i[1] += tempVal * globalTimeInterval6.i[0];
        remainder.i[0] += tempVal * globalTimeInterval6.i[1];
    }

    remainder += (cInterval(c[0]) + c[1] * globalTimeInterval +
        c[2] * globalTimeInterval2 + c[3] * globalTimeInterval3) *
        inr +
        (cInterval(inc0) + inc1 * globalTimeInterval +
            inc2 * globalTimeInterval2 + inc3 * globalTimeInterval3) *
        r;

    return cTaylorModel3(c0, c1, c2, c3, remainder);
}

cTaylorModel3 cTaylorModel3::operator*(const double d) {
    return cTaylorModel3(c[0] * d, c[1] * d, c[2] * d, c[3] * d, r * d);
}
cTaylorModel3 cTaylorModel3::operator-() {
    return cTaylorModel3(-c[0], -c[1], -c[2], -c[3], -r);
}

void cTaylorModel3::print() {
    std::cout << c[0] << (c[1] < 0 ? "" : "+") << c[1] << (c[2] < 0 ? "*t" : "*t+")
        << c[2] << (c[3] < 0 ? "*t^2" : "*t^2+") << c[3] << "*t^3+"
        << "[" << r.i[0] << "," << r.i[1] << "] over ["
        << globalTimeInterval.i[0] << "," << globalTimeInterval.i[1] << "]"
        << std::endl;
    std::cout << std::endl;
}

cInterval cTaylorModel3::bound() {
    //	return
    //cInterval(c[0],c[0])+c[1]*globalTimeInterval+c[2]*globalTimeInterval2+c[3]*globalTimeInterval3+r;

#if 1

    if (!c[3])
        return cInterval(c[0] + r.i[0], c[0] + r.i[1]) +
        globalTimeInterval *
        (c[1] + globalTimeInterval * (c[2] + globalTimeInterval * c[3]));

    // compute actual bounds of the cubic

    cInterval bounds = this->evaluate(globalTimeInterval.i[0]);
    cInterval bounds_bound = this->evaluate(globalTimeInterval.i[1]);
    //bounds.bound(this->evaluate(globalTimeInterval.i[1]));
    bounds.bound(bounds_bound);

    // derivative : c[1]+2*c[2]*t+3*c[3]*t^2

    double delta = c[2] * c[2] - c[1] * 3 * c[3];
    if (delta < 0) return bounds;  // constant variation

    double r1 = (-c[2] - sqrt(delta)) / (3 * c[3]);
    double r2 = (-c[2] + sqrt(delta)) / (3 * c[3]);

    //if (globalTimeInterval.contains(r1)) bounds.bound(this->evaluate(r1));
    //if (globalTimeInterval.contains(r2)) bounds.bound(this->evaluate(r2));
    if (globalTimeInterval.contains(r1)) {
        cInterval bounds_bound = this->evaluate(r1);
        bounds.bound(bounds_bound);
    }
    if (globalTimeInterval.contains(r2)) {
        cInterval bounds_bound = this->evaluate(r2);
        bounds.bound(bounds_bound);
    }

    return bounds;

#else

    return cInterval(c[0] + r.i[0], c[0] + r.i[1]) +
        globalTimeInterval *
        (c[1] + globalTimeInterval * (c[2] + globalTimeInterval * c[3]));

#endif
}

cInterval cTaylorModel3::bound(double t0, double t1) {
    cInterval customTimeInterval(t0, t1);
    cInterval customTimeInterval2(
        customTimeInterval.i[0] * customTimeInterval.i[0],
        customTimeInterval.i[1] * customTimeInterval.i[1]);
    cInterval customTimeInterval3(
        customTimeInterval.i[0] * customTimeInterval2.i[0],
        customTimeInterval.i[1] * customTimeInterval2.i[1]);

    return cInterval(c[0], c[0]) + c[1] * customTimeInterval +
        c[2] * customTimeInterval2 + c[3] * customTimeInterval3 + r;
}

cInterval cTaylorModel3::evaluate(double t) {
    return cInterval(c[0] + t * (c[1] + t * (c[2] + t * c[3]))) + r;
}

void cTaylorModel3::setZero() { memset(this, 0, sizeof(cTaylorModel3)); }

void cTaylorModel3::cosModel(double w, double q0) {
    double a = 0.5 * (globalTimeInterval.i[0] + globalTimeInterval.i[1]);
    double t = w * a + q0;
    double w2 = w * w;
    double fa = cos(t);
    double fda = -w * sin(t);
    double fdda = -w2 * fa;
    double fddda = -w2 * fda;

    c[0] = fa - a * (fda - 0.5 * a * (fdda - 1.0 / 3.0 * a * fddda));
    c[1] = fda - a * fdda + 0.5 * a * a * fddda;
    c[2] = 0.5 * (fdda - a * fddda);
    c[3] = 1.0 / 6.0 * fddda;

    // compute bounds on the fourth derivative w^4*cos(w*t+q0)

    cInterval fddddBounds;

    if (!w)
        fddddBounds.i[0] = fddddBounds.i[1] = 0.0;
    else if (w > 0) {
        double cosQL = cos(q0 + globalTimeInterval.i[0] * w);
        double cosQR = cos(q0 + globalTimeInterval.i[1] * w);

        if (cosQL < cosQR)
            fddddBounds = cInterval(cosQL, cosQR);
        else
            fddddBounds = cInterval(cosQR, cosQL);

        // enlarge to counter round-off errors

        fddddBounds.i[0] -= 1e-15;
        fddddBounds.i[1] += 1e-15;

        // cos reaches a maximum if there exists an integer k in
        // [(q0)/(2pi),(q0+w)/(2pi)] cos reaches a minimum if there exists an
        // integer k in [(q0-pi)/(2pi),(q0-pi+w)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double min = 0.5 * q0 / CTM3_PI;

        if (ceil(min) <= min + step) fddddBounds.i[1] = 1.0;
        min -= 0.5;
        if (ceil(min) <= min + step) fddddBounds.i[0] = -1.0;

    }
    else {
        double cosQL = cos(q0 + globalTimeInterval.i[0] * w);
        double cosQR = cos(q0 + globalTimeInterval.i[1] * w);

        if (cosQL < cosQR) {
            fddddBounds.i[0] = cosQL;
            fddddBounds.i[1] = cosQR;
        }
        else {
            fddddBounds.i[1] = cosQL;
            fddddBounds.i[0] = cosQR;
        }

        // cos reaches a maximum if there exists an integer k in
        // [(q0+w)/(2pi),(q0)/(2pi)] cos reaches a minimum if there exists an
        // integer k in [(q0-pi+w)/(2pi),(q0-pi)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double max = 0.5 * q0 / CTM3_PI;

        if (floor(max) >= max + step) fddddBounds.i[1] = 1.0;
        max -= 0.5;
        if (floor(max) >= max + step) fddddBounds.i[0] = -1.0;
    }

    double w4 = w2 * w2;
    fddddBounds.i[0] *= w4;
    fddddBounds.i[1] *= w4;

    double midSize = 0.5 * (globalTimeInterval.i[1] - globalTimeInterval.i[0]);
    double midSize2 = midSize * midSize;
    double midSize4 = midSize2 * midSize2;

    if (fddddBounds.i[0] >= 0) {
        r.i[0] = 0.0;
        r.i[1] = fddddBounds.i[1] * midSize4 * (1. / 24.0);
    }
    else if (fddddBounds.i[1] <= 0) {
        r.i[0] = fddddBounds.i[0] * midSize4 * (1. / 24.0);
        r.i[1] = 0.0;
    }
    else {
        r.i[0] = fddddBounds.i[0] * midSize4 * (1. / 24.0);
        r.i[1] = fddddBounds.i[1] * midSize4 * (1. / 24.0);
    }
}

void cTaylorModel3::sinModel(double w, double q0) {
    double a = 0.5 * (globalTimeInterval.i[0] + globalTimeInterval.i[1]);
    double t = w * a + q0;
    double w2 = w * w;
    double fa = sin(t);
    double fda = w * cos(t);
    double fdda = -w2 * fa;
    double fddda = -w2 * fda;

    c[0] = fa - a * (fda - 0.5 * a * (fdda - 1.0 / 3.0 * a * fddda));
    c[1] = fda - a * fdda + 0.5 * a * a * fddda;
    c[2] = 0.5 * (fdda - a * fddda);
    c[3] = 1.0 / 6.0 * fddda;

    // compute bounds on the fourth derivative w^4*sin(w*t+q0)

    cInterval fddddBounds;

    if (!w)
        fddddBounds.i[0] = fddddBounds.i[1] = 0.0;
    else if (w > 0) {
        double sinQL = sin(q0 + globalTimeInterval.i[0] * w);
        double sinQR = sin(q0 + globalTimeInterval.i[1] * w);

        if (sinQL < sinQR) {
            fddddBounds.i[0] = sinQL;
            fddddBounds.i[1] = sinQR;
        }
        else {
            fddddBounds.i[1] = sinQL;
            fddddBounds.i[0] = sinQR;
        }

        // enlarge to counter round-off errors

        fddddBounds.i[0] -= 1e-15;
        fddddBounds.i[1] += 1e-15;

        // sin reaches a maximum if there exists an integer k in
        // [(q0-pi/2)/(2pi),(q0-pi/2+w)/(2pi)] sin reaches a minimum if there exists
        // an integer k in [(q0-3pi/2)/(2pi),(q0-3pi/2+w)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double min = 0.5 * q0 / CTM3_PI;

        min -= 0.25;
        if (ceil(min) <= min + step) fddddBounds.i[1] = 1.0;
        min -= 0.5;
        if (ceil(min) <= min + step) fddddBounds.i[0] = -1.0;

    }
    else {
        double sinQL = sin(q0 + globalTimeInterval.i[0] * w);
        double sinQR = sin(q0 + globalTimeInterval.i[1] * w);

        if (sinQL < sinQR)
            fddddBounds = cInterval(sinQL, sinQR);
        else
            fddddBounds = cInterval(sinQR, sinQL);

        // sin reaches a maximum if there exists an integer k in
        // [(q0-pi/2+w)/(2pi),(q0-pi/2)/(2pi)] sin reaches a minimum if there exists
        // an integer k in [(q0-3pi/2+w)/(2pi),(q0-3pi/2)/(2pi)]

        double step = 0.5 * w / CTM3_PI;
        double max = 0.5 * q0 / CTM3_PI;

        max -= 0.25;
        if (floor(max) >= max + step) fddddBounds.i[1] = 1.0;
        max -= 0.5;
        if (floor(max) >= max + step) fddddBounds.i[0] = -1.0;
    }

    double w4 = w2 * w2;
    fddddBounds.i[0] *= w4;
    fddddBounds.i[1] *= w4;

    double midSize = 0.5 * (globalTimeInterval.i[1] - globalTimeInterval.i[0]);
    double midSize2 = midSize * midSize;
    double midSize4 = midSize2 * midSize2;

    if (fddddBounds.i[0] >= 0) {
        r.i[0] = 0.0;
        r.i[1] = fddddBounds.i[1] * midSize4 * (1. / 24.0);
    }
    else if (fddddBounds.i[1] <= 0) {
        r.i[0] = fddddBounds.i[0] * midSize4 * (1. / 24.0);
        r.i[1] = 0.0;
    }
    else {
        r.i[0] = fddddBounds.i[0] * midSize4 * (1. / 24.0);
        r.i[1] = fddddBounds.i[1] * midSize4 * (1. / 24.0);
    }
}

void cTaylorModel3::cospolytModel(double a0, double a1, double a2, double a3) {
    // y = cos(a0+a1*x+a2*x^2+a3*x^3);

    double x0 = (globalTimeInterval.i[0] + globalTimeInterval.i[1]) / 2;
    double theta = a0 + a1 * x0 + a2 * x0 * x0 + a3 * x0 * x0 * x0;
    double cos_theta = cos(theta);  //y(x0)
    double sin_theta = sin(theta);
    double theta_d = a1 + 2 * a2 * x0 + 3 * a3 * x0 * x0;
    double theta_dd = 2 * a2 + 6 * a3 * x0;
    double theta_ddd = 6 * a3;
    
    // y = b0+b1*(x-x0)+b2*(x-x0)^2+b3*(x-x0)^3;
    double b0 = cos_theta;
    double b1 = -theta_d * sin_theta;
    double b2 = -0.5 * (theta_dd * sin_theta + theta_d * theta_d * cos_theta);
    double b3 = -0.5 * (theta_d * theta_dd * cos_theta + (theta_ddd - theta_d * theta_d * theta_d) * sin_theta / 3);

    c[0] = b0 - b1 * x0 + b2 * x0 * x0 - b3 * x0 * x0 * x0;
    c[1] = b1 - 2 * b2 * x0 + 3 * b3 * x0 * x0;
    c[2] = b2 - 3 * b3 * x0;
    c[3] = b3;

    cInterval fddddBounds;

    //// Peano Remainder
    //fddddBounds = pow((globalTimeInterval.i[1] - globalTimeInterval.i[0]) / 4, 3) * triBounds;

    double t0 = globalTimeInterval.i[0];
    double t1 = globalTimeInterval.i[1];
    double y0 = abs(c[0] + c[1] * t0 + c[2] * t0 * t0 + c[3] * t0 * t0 * t0 - cos(a0 + a1 * t0 + a2 * t0 * t0 + a3 * t0 * t0 * t0));
    double y1 = abs(c[0] + c[1] * t1 + c[2] * t1 * t1 + c[3] * t1 * t1 * t1 - cos(a0 + a1 * t1 + a2 * t1 * t1 + a3 * t1 * t1 * t1));
    double err = std::max(y0, y1);

    fddddBounds.i[0] = -err;
    fddddBounds.i[1] = +err;

    // enlarge to counter round-off errors
    fddddBounds.i[0] -= 1e-15;
    fddddBounds.i[1] += 1e-15;

    r.i[0] = fddddBounds.i[0];
    r.i[1] = fddddBounds.i[1];
}

//void cTaylorModel3::sinpolytModel(double a0, double a1, double a2, double a3) {
//    cInterval globalTimeInterval7, globalTimeInterval8, globalTimeInterval9;
//    globalTimeInterval7.i[0] = globalTimeInterval4.i[0] * globalTimeInterval3.i[0];
//    globalTimeInterval7.i[1] = globalTimeInterval4.i[1] * globalTimeInterval3.i[1];
//    globalTimeInterval8.i[0] = globalTimeInterval4.i[0] * globalTimeInterval4.i[0];
//    globalTimeInterval8.i[1] = globalTimeInterval4.i[1] * globalTimeInterval4.i[1];
//    globalTimeInterval9.i[0] = globalTimeInterval5.i[0] * globalTimeInterval4.i[0];
//    globalTimeInterval9.i[1] = globalTimeInterval5.i[1] * globalTimeInterval4.i[1];
//
//    double x0 = (globalTimeInterval.i[0] + globalTimeInterval.i[1]) / 2;
//    double theta = a0 + a1 * x0 + a2 * x0 * x0 + a3 * x0 * x0 * x0;
//    double cos_theta = cos(theta);
//    double sin_theta = sin(theta);
//    double theta_theta = theta * theta;
//    double theta_theta_theta = theta_theta * theta;
//
//    double b0 = sin_theta - cos_theta * theta - 0.5 * sin_theta * theta_theta + cos_theta * theta_theta_theta / 6;
//    double b1 = cos_theta + sin_theta * theta - 0.5 * cos_theta * theta_theta;
//    double b2 = -0.5 * sin_theta + 0.5 * cos_theta * theta;
//    double b3 = -cos_theta / 6;
//
//    c[0] = b0 + a0 * b1 + a0 * a0 * b2 + a0 * a0 * a0 * b3;
//    c[1] = 0;
//    c[2] = a2 * b1 + 2 * a0 * a2 * b2 + 3 * a0 * a0 * a2 * b3;
//    c[3] = a3 * b1 + 2 * a0 * a3 * b2 + 3 * a0 * a0 * a3 * b3;
//
//    cInterval I1, I2;
//
//    I1 =  (a2 * a2*b2 + 3 * a0 * a2 * a2*b3)*globalTimeInterval4 + (6 * a0 * a2 * a3 * b3 + 2 * a2 * a3 * b2) * globalTimeInterval5 + (3 * a0 * a3 * a3 * b3 + a2 * a2 * a2 * b3 + a3 * a3 * b2) * globalTimeInterval6 + 3 * a2 * a2 * a3 * b3 * globalTimeInterval7 + 3 * a2 * a3 * a3 * b3 * globalTimeInterval8 + a3 * a3 * a3 * b3 * globalTimeInterval9;
//
//    cInterval fx, fx_bar;
//    if (thetaf > theta0) {
//        fx.i[0] = a0 + a1 * globalTimeInterval2.i[0] + a3 * globalTimeInterval3.i[0];
//        fx.i[1] = a0 + a1 * globalTimeInterval2.i[1] + a3 * globalTimeInterval3.i[1];
//    }
//    else {
//        fx.i[0] = a0 + a1 * globalTimeInterval2.i[1] + a3 * globalTimeInterval3.i[1];
//        fx.i[1] = a0 + a1 * globalTimeInterval2.i[0] + a3 * globalTimeInterval3.i[0];
//    }
//    fx_bar = fx - theta;
//
//
//    cInterval xi(0, 1);
//    cInterval xi_theta;
//    xi_theta = xi * fx_bar;
//
//    I2.i[0] = std::min(cos(xi_theta.i[0] + theta), cos(xi_theta.i[1] + theta));
//    I2.i[1] = std::max(cos(xi_theta.i[0] + theta), cos(xi_theta.i[1] + theta));
//
//    // min and max
//    if (ceil((xi_theta.i[0] + theta) / (2 * CTM3_PI)) <= floor((xi_theta.i[1] + theta) / (2 * CTM3_PI))) I2.i[1] = 1;
//    if (ceil((xi_theta.i[0] + theta - CTM3_PI) / (2 * CTM3_PI)) <= floor((xi_theta.i[1] + theta - CTM3_PI) / (2 * CTM3_PI))) I2.i[1] = 1;
//    // min and max
//
//    r = (fx_bar * fx_bar * fx_bar * fx_bar * I2 + I1) / 12;
//
//}

void cTaylorModel3::sinpolytModel(double a0, double a1, double a2, double a3) {
    // y = sin(a0+a1*x+a2*x^2+a3*x^3);

    double x0 = (globalTimeInterval.i[0] + globalTimeInterval.i[1]) / 2;
    double theta = a0 + a1 * x0 + a2 * x0 * x0 + a3 * x0 * x0 * x0;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);  //y(x0)
    double theta_d = a1 + 2 * a2 * x0 + 3 * a3 * x0 * x0;
    double theta_dd = 2 * a2 + 6 * a3 * x0;
    double theta_ddd = 6 * a3;

    // y = b0+b1*(x-x0)+b2*(x-x0)^2+b3*(x-x0)^3;
    double b0 = sin_theta;
    double b1 = theta_d * cos_theta;
    double b2 = 0.5 * (theta_dd * cos_theta - theta_d * theta_d * sin_theta);
    double b3 = 0.5 * (-theta_d * theta_dd * sin_theta + (theta_ddd - theta_d * theta_d * theta_d) * cos_theta / 3);

    c[0] = b0 - b1 * x0 + b2 * x0 * x0 - b3 * x0 * x0 * x0;
    c[1] = b1 - 2 * b2 * x0 + 3 * b3 * x0 * x0;
    c[2] = b2 - 3 * b3 * x0;
    c[3] = b3;

    cInterval fddddBounds;

    //// Peano Remainder
    //fddddBounds = pow((globalTimeInterval.i[1] - globalTimeInterval.i[0]) / 4, 3) * triBounds;

    double t0 = globalTimeInterval.i[0];
    double t1 = globalTimeInterval.i[1];
    double y0 = abs(c[0] + c[1] * t0 + c[2] * t0 * t0 + c[3] * t0 * t0 * t0 - sin(a0 + a1 * t0 + a2 * t0 * t0 + a3 * t0 * t0 * t0));
    double y1 = abs(c[0] + c[1] * t1 + c[2] * t1 * t1 + c[3] * t1 * t1 * t1 - sin(a0 + a1 * t1 + a2 * t1 * t1 + a3 * t1 * t1 * t1));
    double err = std::max(y0, y1);
    double cor = err * pow((globalTimeInterval.i[1] - globalTimeInterval.i[0]), 1); //correction value

    fddddBounds.i[0] = -err;
    fddddBounds.i[1] = +err;

    // enlarge to counter round-off errors
    fddddBounds.i[0] -= 1e-15;
    fddddBounds.i[1] += 1e-15;

    r.i[0] = fddddBounds.i[0];
    r.i[1] = fddddBounds.i[1];
}

void cTaylorModel3::linearModel(double p, double v) {
    c[0] = p;
    c[1] = v;
    c[2] = c[3] = r.i[0] = r.i[1] = 0.0;
}

cTaylorModel3 operator*(double d, cTaylorModel3& tm) {
    return cTaylorModel3(d * tm.c[0], d * tm.c[1], d * tm.c[2], d * tm.c[3],
        d * tm.r);
}

cTM3Matrix44::cTM3Matrix44() { ; }
cTM3Matrix44::cTM3Matrix44(cTaylorModel3& a00, cTaylorModel3& a01, cTaylorModel3& a02, cTaylorModel3& a03,
                           cTaylorModel3& a10, cTaylorModel3& a11, cTaylorModel3& a12, cTaylorModel3& a13,
                           cTaylorModel3& a20, cTaylorModel3& a21, cTaylorModel3& a22, cTaylorModel3& a23,
                           cTaylorModel3& a30, cTaylorModel3& a31, cTaylorModel3& a32, cTaylorModel3& a33) {
    M[0][0] = a00;
    M[0][1] = a01;
    M[0][2] = a02;
    M[0][3] = a03;
    M[1][0] = a10;
    M[1][1] = a11;
    M[1][2] = a12;
    M[1][3] = a13;
    M[2][0] = a20;
    M[2][1] = a21;
    M[2][2] = a22;
    M[2][3] = a23;
    M[3][0] = a30;
    M[3][1] = a31;
    M[3][2] = a32;
    M[3][3] = a33;
}
cTM3Matrix44::cTM3Matrix44(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13, double a20, double a21, double a22, double a23, double a30, double a31, double a32, double a33) {
    cTaylorModel3 b00(a00);
    cTaylorModel3 b01(a01);
    cTaylorModel3 b02(a02);
    cTaylorModel3 b03(a03);
    cTaylorModel3 b10(a10);
    cTaylorModel3 b11(a11);
    cTaylorModel3 b12(a12);
    cTaylorModel3 b13(a13);
    cTaylorModel3 b20(a20);
    cTaylorModel3 b21(a21);
    cTaylorModel3 b22(a22);
    cTaylorModel3 b23(a23);
    cTaylorModel3 b30(a30);
    cTaylorModel3 b31(a31);
    cTaylorModel3 b32(a32);
    cTaylorModel3 b33(a33);
    M[0][0] = b00;
    M[0][1] = b01;
    M[0][2] = b02;
    M[0][3] = b03;
    M[1][0] = b10;
    M[1][1] = b11;
    M[1][2] = b12;
    M[1][3] = b13;
    M[2][0] = b20;
    M[2][1] = b21;
    M[2][2] = b22;
    M[2][3] = b23;
    M[3][0] = b30;
    M[3][1] = b31;
    M[3][2] = b32;
    M[3][3] = b33;
}
cTM3Matrix44::cTM3Matrix44(cMatrix33& a, double xyz[3]) {
    cTaylorModel3 b00(a.m[0][0]);
    cTaylorModel3 b01(a.m[0][1]);
    cTaylorModel3 b02(a.m[0][2]);
    cTaylorModel3 b03(xyz[0]);
    cTaylorModel3 b10(a.m[1][0]);
    cTaylorModel3 b11(a.m[1][1]);
    cTaylorModel3 b12(a.m[1][2]);
    cTaylorModel3 b13(xyz[1]);
    cTaylorModel3 b20(a.m[2][0]);
    cTaylorModel3 b21(a.m[2][1]);
    cTaylorModel3 b22(a.m[2][2]);
    cTaylorModel3 b23(xyz[2]);
    cTaylorModel3 b30(0);
    cTaylorModel3 b31(0);
    cTaylorModel3 b32(0);
    cTaylorModel3 b33(1);
    M[0][0] = b00;
    M[0][1] = b01;
    M[0][2] = b02;
    M[0][3] = b03;
    M[1][0] = b10;
    M[1][1] = b11;
    M[1][2] = b12;
    M[1][3] = b13;
    M[2][0] = b20;
    M[2][1] = b21;
    M[2][2] = b22;
    M[2][3] = b23;
    M[3][0] = b30;
    M[3][1] = b31;
    M[3][2] = b32;
    M[3][3] = b33;
}
//cTM3Matrix44::cTM3Matrix44(cTM3Matrix33& aa, double xyz[3]) {
//    cTaylorModel3 b03(xyz[0]);
//    cTaylorModel3 b13(xyz[1]);
//    cTaylorModel3 b23(xyz[2]);
//    cTaylorModel3 b30(0);
//    cTaylorModel3 b31(0);
//    cTaylorModel3 b32(0);
//    cTaylorModel3 b33(1);
//    M[0][0] = aa.i[0][0];
//    M[0][1] = aa.i[0][1];
//    M[0][2] = aa.i[0][2];
//    M[0][3] = b03;
//    M[1][0] = aa.i[1][0];
//    M[1][1] = aa.i[1][1];
//    M[1][2] = aa.i[1][2];
//    M[1][3] = b13;
//    M[2][0] = aa.i[2][0];
//    M[2][1] = aa.i[2][1];
//    M[2][2] = aa.i[2][2];
//    M[2][3] = b23;
//    M[3][0] = b30;
//    M[3][1] = b31;
//    M[3][2] = b32;
//    M[3][3] = b33;
//}

//cTM3Matrix44::cTM3Matrix44(cTM3Matrix33& a, cTM3Vector3& t) {
//    cTaylorModel3 zero(0);
//    cTaylorModel3 one(1);
//    M[0][0] = a.i[0][0];
//    M[0][1] = a.i[0][1];
//    M[0][2] = a.i[0][2];
//    M[0][3] = t.i[0];
//    M[1][0] = a.i[1][0];
//    M[1][1] = a.i[1][1];
//    M[1][2] = a.i[1][2];
//    M[1][3] = t.i[1];
//    M[2][0] = a.i[2][0];
//    M[2][1] = a.i[2][1];
//    M[2][2] = a.i[2][2];
//    M[2][3] = t.i[2];
//    M[3][0] = zero;
//    M[3][1] = zero;
//    M[3][2] = zero;
//    M[3][3] = one;
//}

cTM3Matrix44 cTM3Matrix44::operator*(const cTM3Matrix44& in){
    register cTaylorModel3 c00, c01, c02, c03, c10, c11, c12, c13, c20, c21, c22, c23, c30, c31, c32, c33;
    c00 = M[0][0] * in.M[0][0] + M[0][1] * in.M[1][0] + M[0][2] * in.M[2][0] + M[0][3] * in.M[3][0];
    c01 = M[0][0] * in.M[0][1] + M[0][1] * in.M[1][1] + M[0][2] * in.M[2][1] + M[0][3] * in.M[3][1];
    c02 = M[0][0] * in.M[0][2] + M[0][1] * in.M[1][2] + M[0][2] * in.M[2][2] + M[0][3] * in.M[3][2];
    c03 = M[0][0] * in.M[0][3] + M[0][1] * in.M[1][3] + M[0][2] * in.M[2][3] + M[0][3] * in.M[3][3];
    c10 = M[1][0] * in.M[0][0] + M[1][1] * in.M[1][0] + M[1][2] * in.M[2][0] + M[1][3] * in.M[3][0];
    c11 = M[1][0] * in.M[0][1] + M[1][1] * in.M[1][1] + M[1][2] * in.M[2][1] + M[1][3] * in.M[3][1];
    c12 = M[1][0] * in.M[0][2] + M[1][1] * in.M[1][2] + M[1][2] * in.M[2][2] + M[1][3] * in.M[3][2];
    c13 = M[1][0] * in.M[0][3] + M[1][1] * in.M[1][3] + M[1][2] * in.M[2][3] + M[1][3] * in.M[3][3];
    c20 = M[2][0] * in.M[0][0] + M[2][1] * in.M[1][0] + M[2][2] * in.M[2][0] + M[2][3] * in.M[3][0];
    c21 = M[2][0] * in.M[0][1] + M[2][1] * in.M[1][1] + M[2][2] * in.M[2][1] + M[2][3] * in.M[3][1];
    c22 = M[2][0] * in.M[0][2] + M[2][1] * in.M[1][2] + M[2][2] * in.M[2][2] + M[2][3] * in.M[3][2];
    c23 = M[2][0] * in.M[0][3] + M[2][1] * in.M[1][3] + M[2][2] * in.M[2][3] + M[2][3] * in.M[3][3];
    c30 = M[3][0] * in.M[0][0] + M[3][1] * in.M[1][0] + M[3][2] * in.M[2][0] + M[3][3] * in.M[3][0];
    c31 = M[3][0] * in.M[0][1] + M[3][1] * in.M[1][1] + M[3][2] * in.M[2][1] + M[3][3] * in.M[3][1];
    c32 = M[3][0] * in.M[0][2] + M[3][1] * in.M[1][2] + M[3][2] * in.M[2][2] + M[3][3] * in.M[3][2];
    c33 = M[3][0] * in.M[0][3] + M[3][1] * in.M[1][3] + M[3][2] * in.M[2][3] + M[3][3] * in.M[3][3];
    return cTM3Matrix44(c00, c01, c02, c03, c10, c11, c12, c13, c20, c21, c22, c23, c30, c31, c32, c33);
}

cTMVector4::cTMVector4() { ; }
cTMVector4::cTMVector4(cTaylorModel3 p0, cTaylorModel3 p1, cTaylorModel3 p2, cTaylorModel3 p3) {
    P[0] = p0;
    P[1] = p1;
    P[2] = p2;
    P[3] = p3;
}
cTMVector4::cTMVector4(double p0, double p1, double p2, double p3) {
    P[0] = p0;
    P[1] = p1;
    P[2] = p2;
    P[3] = p3;
}

cTMVector4::cTMVector4(double p[4]){
    P[0] = p[0];
    P[1] = p[1];
    P[2] = p[2];
    P[3] = p[3];
}

//cTM3Matrix44 setupHMatrix(double roll, double pitch, double yaw, double x, double y, double z) {
//    // roll pitch yaw
//    // phi theta psi
//    cTaylorModel3 r11, r12, r13, r21, r22, r23, r31, r32, r33;
//    cTaylorModel3 cphi, sphi, ctheta, stheta, cpsi, spsi;
//    cphi.cosModel(joint_para[arm_seq][joint_seq].omega, joint_para[arm_seq][joint_seq].theta_0 + roll);
//    sphi.sinModel(joint_para[arm_seq][joint_seq].omega, joint_para[arm_seq][joint_seq].theta_0 + roll);
//    ctheta.cosModel(0, pitch);
//    stheta.sinModel(0, pitch);
//    cpsi.cosModel(0, yaw);
//    spsi.sinModel(0, yaw);
//    r11 = cphi * ctheta;
//    r12 = -sphi * cpsi + cphi * stheta * spsi;
//    r13 = sphi * spsi + cphi * stheta * cpsi;
//    r21 = sphi * ctheta;
//    r22 = cphi * cpsi + sphi * stheta * spsi;
//    r23 = -cphi * spsi + sphi * stheta * cpsi;
//    r31 = -stheta;
//    r32 = ctheta * spsi;
//    r33 = ctheta * cpsi;
//    cTaylorModel3 o1(x);
//    cTaylorModel3 o2(y);
//    cTaylorModel3 o3(z);
//    cTaylorModel3 const0(0);
//    cTaylorModel3 const1(1);
//    return cTM3Matrix44(r11, r12, r13, o1, r21, r22, r23, o2, r31, r32, r33, o3, const0, const0, const0, const1);
//}

void setGlobalTimeInterval(double l, double r) {
    globalTimeInterval.i[0] = l;
    globalTimeInterval.i[1] = r;
    globalTimeInterval2.i[0] = globalTimeInterval.i[0] * globalTimeInterval.i[0];
    globalTimeInterval2.i[1] = globalTimeInterval.i[1] * globalTimeInterval.i[1];
    globalTimeInterval3.i[0] = globalTimeInterval.i[0] * globalTimeInterval2.i[0];
    globalTimeInterval3.i[1] = globalTimeInterval.i[1] * globalTimeInterval2.i[1];
    globalTimeInterval4.i[0] =
        globalTimeInterval2.i[0] * globalTimeInterval2.i[0];
    globalTimeInterval4.i[1] =
        globalTimeInterval2.i[1] * globalTimeInterval2.i[1];
    globalTimeInterval5.i[0] =
        globalTimeInterval3.i[0] * globalTimeInterval2.i[0];
    globalTimeInterval5.i[1] =
        globalTimeInterval3.i[1] * globalTimeInterval2.i[1];
    globalTimeInterval6.i[0] =
        globalTimeInterval3.i[0] * globalTimeInterval3.i[0];
    globalTimeInterval6.i[1] =
        globalTimeInterval3.i[1] * globalTimeInterval3.i[1];
}

cInterval getGlobalTimeInterval() { return globalTimeInterval; }

//cTaylorModel3 distance_square_cal(cTMVector4& p, cTMVector4& q) {
//
//    cTaylorModel3 x = p.P[0] - q.P[0];
//    cTaylorModel3 y = p.P[1] - q.P[1];
//    cTaylorModel3 z = p.P[2] - q.P[2];
//
//    cTaylorModel3 dis_square = x * x + y * y + z * z;
//
//    return dis_square;
//}