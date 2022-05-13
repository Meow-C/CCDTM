#include "fcl/tm/cTaylorModel3.h"
#include "fcl/tm/cTM3Matrix33.h"
#include "fcl/tm/cTM3Vector3.h"

// COMPILE: g++ -o quat2EulerTest quat2EulerTest.cpp 
#include <iostream>
#include <cmath> 

using namespace std;

///////////////////////////////
// Quaternion struct
// Simple incomplete quaternion struct for demo purpose
///////////////////////////////
struct Quaternion {
    Quaternion() :x(0), y(0), z(0), w(1) {};
    Quaternion(double x, double y, double z, double w) :x(x), y(y), z(z), w(w) {};

    void normalize() {
        double norm = std::sqrt(x * x + y * y + z * z + w * w);
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    }

    double norm() {
        return std::sqrt(x * x + y * y + z * z + w * w);
    }

    double x;
    double y;
    double z;
    double w;

};

///////////////////////////////
// Quaternion to Euler
///////////////////////////////
enum RotSeq { zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx };

void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
    res[0] = atan2(r11, r12);
    res[1] = acos(r21);
    res[2] = atan2(r31, r32);
}

void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]) {
    res[0] = atan2(r31, r32);
    res[1] = asin(r21);
    res[2] = atan2(r11, r12);
}

void quaternion2Euler(const Quaternion& q, double res[], RotSeq rotSeq)
{
    switch (rotSeq) {
    case zyx:
        threeaxisrot(2 * (q.x * q.y + q.w * q.z),
            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
            -2 * (q.x * q.z - q.w * q.y),
            2 * (q.y * q.z + q.w * q.x),
            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
            res);
        break;

    case zyz:
        twoaxisrot(2 * (q.y * q.z - q.w * q.x),
            2 * (q.x * q.z + q.w * q.y),
            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
            2 * (q.y * q.z + q.w * q.x),
            -2 * (q.x * q.z - q.w * q.y),
            res);
        break;

    case zxy:
        threeaxisrot(-2 * (q.x * q.y - q.w * q.z),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
            2 * (q.y * q.z + q.w * q.x),
            -2 * (q.x * q.z - q.w * q.y),
            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
            res);
        break;

    case zxz:
        twoaxisrot(2 * (q.x * q.z + q.w * q.y),
            -2 * (q.y * q.z - q.w * q.x),
            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
            2 * (q.x * q.z - q.w * q.y),
            2 * (q.y * q.z + q.w * q.x),
            res);
        break;

    case yxz:
        threeaxisrot(2 * (q.x * q.z + q.w * q.y),
            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
            -2 * (q.y * q.z - q.w * q.x),
            2 * (q.x * q.y + q.w * q.z),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
            res);
        break;

    case yxy:
        twoaxisrot(2 * (q.x * q.y - q.w * q.z),
            2 * (q.y * q.z + q.w * q.x),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
            2 * (q.x * q.y + q.w * q.z),
            -2 * (q.y * q.z - q.w * q.x),
            res);
        break;

    case yzx:
        threeaxisrot(-2 * (q.x * q.z - q.w * q.y),
            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
            2 * (q.x * q.y + q.w * q.z),
            -2 * (q.y * q.z - q.w * q.x),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
            res);
        break;

    case yzy:
        twoaxisrot(2 * (q.y * q.z + q.w * q.x),
            -2 * (q.x * q.y - q.w * q.z),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
            2 * (q.y * q.z - q.w * q.x),
            2 * (q.x * q.y + q.w * q.z),
            res);
        break;

    case xyz:
        threeaxisrot(-2 * (q.y * q.z - q.w * q.x),
            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
            2 * (q.x * q.z + q.w * q.y),
            -2 * (q.x * q.y - q.w * q.z),
            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
            res);
        break;

    case xyx:
        twoaxisrot(2 * (q.x * q.y + q.w * q.z),
            -2 * (q.x * q.z - q.w * q.y),
            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
            2 * (q.x * q.y - q.w * q.z),
            2 * (q.x * q.z + q.w * q.y),
            res);
        break;

    case xzy:
        threeaxisrot(2 * (q.y * q.z + q.w * q.x),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
            -2 * (q.x * q.y - q.w * q.z),
            2 * (q.x * q.z + q.w * q.y),
            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
            res);
        break;

    case xzx:
        twoaxisrot(2 * (q.x * q.z - q.w * q.y),
            2 * (q.x * q.y + q.w * q.z),
            q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
            2 * (q.x * q.z + q.w * q.y),
            -2 * (q.x * q.y - q.w * q.z),
            res);
        break;
    default:
        std::cout << "Unknown rotation sequence" << std::endl;
        break;
    }
}

///////////////////////////////
// Helper functions
///////////////////////////////
Quaternion operator*(Quaternion& q1, Quaternion& q2) {
    Quaternion q;
    q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return q;
}

ostream& operator <<(std::ostream& stream, const Quaternion& q) {
    return cout << q.w << " " << showpos << q.x << "i " << q.y << "j " << q.z << "k" << noshowpos;
}

double rad2deg(double rad) {
    return rad * 180.0 / CTM3_PI;
}

void test() {
    Quaternion q0(0.821069, 0.122936, 0.424545, 0.361239);
    double res_q0[3];
    quaternion2Euler(q0, res_q0, zyx);
    cout << "q0: zyx" << endl;
    cout << res_q0[0] << " " << res_q0[1] << " " << res_q0[2] << endl;

    Quaternion p0(0.124737, 0.819668, 0.363881, 0.424471);
    double res_p0[3];
    quaternion2Euler(p0, res_p0, zyx);
    cout << "p0: zyx" << endl;
    cout << res_p0[0] << " " << res_p0[1] << " " << res_p0[2] << endl;

    Quaternion q1(0.653018, -0.572623, 0.494294, 0.0366629);
    double res_q1[3];
    quaternion2Euler(q1, res_q1, zyx);
    cout << "q1: zyx" << endl;
    cout << res_q1[0] << " " << res_q1[1] << " " << res_q1[2] << endl;

    Quaternion p1(0.670215, 0.549834, 0.495854, -0.0512166);
    double res_p1[3];
    quaternion2Euler(p1, res_p1, zyx);
    cout << "p1: zyx" << endl;
    cout << res_p1[0] << " " << res_p1[1] << " " << res_p1[2] << endl;

    cInterval T(0, 0.005);
    setGlobalTimeInterval(T.i[0], T.i[1]);
    double t = T.i[1] - T.i[0];

    double p_omega[3], q_omega[3];  //phi,theta,psi
    for (int i = 0; i < 3; i++) {
        p_omega[i] = (res_p1[i] - res_p0[i]) / t;
        q_omega[i] = (res_q1[i] - res_q0[i]) / t;
    }
    cTaylorModel3 cphi[2], sphi[2], ctheta[2], stheta[2], cpsi[2], spsi[2];
    cphi[0].cosModel(p_omega[0], res_p0[0]);
    sphi[0].sinModel(p_omega[0], res_p0[0]);
    ctheta[0].cosModel(p_omega[1], res_p0[1]);
    stheta[0].sinModel(p_omega[1], res_p0[1]);
    cpsi[0].cosModel(p_omega[2], res_p0[2]);
    spsi[0].sinModel(p_omega[2], res_p0[2]);
    cphi[1].cosModel(q_omega[0], res_q0[0]);
    sphi[1].sinModel(q_omega[0], res_q0[0]);
    ctheta[1].cosModel(q_omega[1], res_q0[1]);
    stheta[1].sinModel(q_omega[1], res_q0[1]);
    cpsi[1].cosModel(q_omega[2], res_q0[2]);
    spsi[1].sinModel(q_omega[2], res_q0[2]);


    double x[2] = { 0.648844 ,0.63987 };
    double y[2] = { 0.0373528 ,-0.331589 };
    double z[2] = { 0.479868 ,0.47588 };

    double end[3] = { 0, 0, 0.0001 };
    cTMVector4 end_p[2];
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);

    for (int i = 0; i < 2; i++) {
        cTaylorModel3 r11 = cphi[i] * ctheta[i];
        cTaylorModel3 r12 = -sphi[i] * cpsi[i] + cphi[i] * stheta[i] * spsi[i];
        cTaylorModel3 r13 = sphi[i] * spsi[i] + cphi[i] * stheta[i] * cpsi[i];
        cTaylorModel3 r21 = sphi[i] * ctheta[i];
        cTaylorModel3 r22 = cphi[i] * cpsi[i] + sphi[i] * stheta[i] * spsi[i];
        cTaylorModel3 r23 = -cphi[i] * spsi[i] + sphi[i] * stheta[i] * cpsi[i];
        cTaylorModel3 r31 = -stheta[i];
        cTaylorModel3 r32 = ctheta[i] * spsi[i];
        cTaylorModel3 r33 = ctheta[i] * cpsi[i];
        cTaylorModel3 px(x[i]);
        cTaylorModel3 py(y[i]);
        cTaylorModel3 pz(z[i]);
        cTM3Matrix44 H(r11, r12, r13, px, r21, r22, r23, py, r31, r32, r33, pz, zero, zero, zero, one);
        cTMVector4 p4(end[0], end[1], end[2], 1);
        end_p[i] = M44multV14(H, p4);
    }

    cTaylorModel3 distance[3];

    for (int i = 0; i < 3; i++) {
        distance[i] = end_p[0].P[i] - end_p[1].P[i];
    }

    cTaylorModel3 dis_square(0);
    for (int i = 0; i < 3; i++) {
        dis_square += distance[i] * distance[i];
    }

    cout << "dis_square: " << endl;
    dis_square.print();
}

///////////////////////////////
// Main
///////////////////////////////
//int main() {
//
//    Quaternion q(0, 0, 1, -1.03412e-13);
//    double res[3];
//    quaternion2Euler(q, res, zyx);
//
//    cout << res[0] << " " << res[1] << " " << res[2] << endl;
//
//    Quaternion q(0.40848, 0.313437, 0.829179, 0.217632);
//
//
//
//}