#include "fcl/tm/ModelParameters.h"

frame_para frame[2][10];
cTM3Matrix33 Rinit[2][10];
cTM3Matrix33 Ryaw[2][10];
cTM3Matrix44 HM[2][10];

cTM3Matrix44 HMhand[2];
cTM3Matrix44 HMleftfinger[2];
cTM3Matrix44 HMrightfinger[2];
cTM3Matrix44 HMleftyaw[2];
cTM3Matrix44 HMrightyaw[2];
cTM3Matrix44 HMbody;

cInterval __T;
int __linki;
int __linkj;
double __toc_err;

bool arm_flag = true;

int link_type[2];

int tra_num = 0;
double running_time_TM3 = 0;
double running_time_TM2 = 0;

cTM2Matrix33 TM2_Rinit[2][10];
cTM2Matrix33 TM2_Ryaw[2][10];
cTM2Matrix44 TM2_HM[2][10];

cTM2Matrix44 TM2_HMhand[2];
cTM2Matrix44 TM2_HMleftfinger[2];
cTM2Matrix44 TM2_HMrightfinger[2];
cTM2Matrix44 TM2_HMleftyaw[2];
cTM2Matrix44 TM2_HMrightyaw[2];
cTM2Matrix44 TM2_HMbody;