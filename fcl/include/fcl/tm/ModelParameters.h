#ifndef __MODELPARAMETERS_HPP__
#define __MODELPARAMETERS_HPP__

#include "cTaylorModel3.h"
#include "cTM3Matrix33.h"
#include "cTM2Matrix33.h"
#include "cTM2Matrix44.h"
#include <vector>
#include <string>

// function parameters
extern double tau;	// threshold

struct frame_para {
	double yaw0;
	double yaw1;
	double yaw_omega;
	double quaternion[4];	//xyzw
	double xyz[3];

	void init() {
	    yaw0 = 0;
	    yaw1 = 0;
	    yaw_omega = 0;
	    quaternion[0] = 0;
	    quaternion[1] = 0;
	    quaternion[2] = 0;
	    quaternion[3] = 0;
	    xyz[0] = 0;
	    xyz[1] = 0;
	    xyz[2] = 0;
	}
};

struct motion{
    cInterval time_interval;
    std::vector<double> start_state;
    std::vector<double> goal_state;
    std::vector<double> velocity;
    cTaylorModel3 dis_square;
};

// result of collision detection for motion
struct result{
    bool is_collide;
    int num;
    std::vector<cInterval> collision_time;
    std::vector<std::string> pairA;
    std::vector<std::string> pairB;

    result(){num = 0; is_collide = false;}
};

extern frame_para frame[2][10];

extern cTM3Matrix33 Rinit[2][10];
extern cTM3Matrix33 Ryaw[2][10];
extern cTM3Matrix44 HM[2][10];

extern cTM3Matrix44 HMhand[2];
extern cTM3Matrix44 HMleftfinger[2];
extern cTM3Matrix44 HMrightfinger[2];
extern cTM3Matrix44 HMleftyaw[2];
extern cTM3Matrix44 HMrightyaw[2];
extern cTM3Matrix44 HMbody;

// BVCollide
extern cInterval __T;
extern int __linki;
extern int __linkj;
extern double __toc_err;  // fcl::FCL_REAL

// single-arm dual-arm mark
extern bool arm_flag;   // true:single false: dual

extern int link_type[2];   // 0: link 1: hand 2: left finger 3: right finger 4: body(for yumi, ABB IRB14000)
extern int tra_num;
extern double running_time_TM3;
extern double running_time_TM2;

extern cTM2Matrix33 TM2_Rinit[2][10];
extern cTM2Matrix33 TM2_Ryaw[2][10];
extern cTM2Matrix44 TM2_HM[2][10];

extern cTM2Matrix44 TM2_HMhand[2];
extern cTM2Matrix44 TM2_HMleftfinger[2];
extern cTM2Matrix44 TM2_HMrightfinger[2];
extern cTM2Matrix44 TM2_HMleftyaw[2];
extern cTM2Matrix44 TM2_HMrightyaw[2];
extern cTM2Matrix44 TM2_HMbody;

struct motion_TM2{
    cInterval time_interval;
    std::vector<double> start_state;
    std::vector<double> goal_state;
    std::vector<double> velocity;
    cTaylorModel2 dis_square;
};

struct ball{
    double x;
    double y;
    double z;
    double r;
};

struct CCDTMData{
    cInterval __T;
    int __linki;
    int __linkj;
    double __toc_err;
    int link_type[2];
    int tra_num;
    cTM3Matrix44 HM[2][10];
    cTM3Matrix44 HMhand[2];
    cTM3Matrix44 HMleftfinger[2];
    cTM3Matrix44 HMrightfinger[2];
    cTM3Matrix44 HMbody;
    std::vector<ball> A;
    std::vector<ball> B;

    CCDTMData() {}

    CCDTMData(cTM3Matrix44 HMbody_, cTM3Matrix44 HM_[2][10], cTM3Matrix44 HMhand_[2], cTM3Matrix44 HMleftfinger_[2], cTM3Matrix44 HMrightfinger_[2])
    {
        HMbody = HMbody_;
        for (int i = 0; i < 2 ; i++) {
            HMhand[i] = HMhand_[i];
            HMleftfinger[i] = HMleftfinger_[i];
            HMrightfinger[i] = HMrightfinger_[i];
            for (int j = 0; j < 10; j++) {
                HM[i][j] = HM_[i][j];
            }
        }
    }
};

#endif // !__MODELPARAMETERS_HPP__