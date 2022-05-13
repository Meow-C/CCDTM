#ifndef __TAYLOR_FUNCTION_HPP__
#define __TAYLOR_FUNCTION_HPP__

#include <string>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "fcl/ccd/motion.h"
#include "cMatrix33.h"
#include "cTM3Matrix33.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/BVH/BV_splitter.h"
#include "fcl/tm/ModelParameters.h"
#include "fcl/tm/cTM2Matrix44.h"
#include "cTM2Vector4.h"

extern std::string file;
extern std::string OBJ_RESOURCES_DIR;
extern std::string file_l;
extern std::string file_r;

void ReadUrdfFile();
void ReadUrdfFile(std::string filename, int i);
void init_yaw();
void init_yaw(double t);
void init_frame_hm();
void setupyaw(double t, const fcl::LerpMotion* m1, const fcl::LerpMotion* m2, int &num);
void setupyaw(const fcl::LerpMotion* m1, const fcl::LerpMotion* m2, int &num);
void setupHM(int num);
void setupdualHM(int num);
void setupdualHM(int num, cInterval& interval);
void setupdualHM(int num, cInterval& interval, motion& sub_motion);
void setupdualHM_movable(int num, cInterval& interval, motion& sub_motion);
void setupdualHM_movable_uncertain(int num, cInterval& interval, motion& sub_motion, std::vector<double> &limit_l, std::vector<double> &limit_u);
void showHM(int num);
void setupHMhand(int num);
void setupHMfinger(int num);

Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y, const double z, const double w);
cMatrix33 MEigen2cM(Eigen::Matrix3d& R);
cTM3Matrix33 MEigen2cTM(Eigen::Matrix3d& R);
cTM3Matrix33 Myaw(double omega, double q0);
cTM3Matrix33 Myaw_uncertain(double omega, double q0, double low, double upper);

void model_load(std::string filename, fcl::BVHModel<fcl::OBBRSS> &obj);
void model_load(std::string filename, fcl::BVHModel<fcl::SPHERE> &obj);
void filename_load(std::vector<std::string> &filenames, const std::string folder);
//void model_traversal(std::vector<std::string> &filenames);
//void leaf_bv_traversal(fcl::BVHModel<fcl::OBBRSS> &obj);

void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles);
fcl::FCL_REAL isoverlarp(int i, int j, cTMVector4 p, cTMVector4 q, cInterval T, fcl::FCL_REAL toc_err);
fcl::FCL_REAL BVHTraversal(int rootA, int rootB, fcl::BVHModel<fcl::OBBRSS> A, fcl::BVHModel<fcl::OBBRSS> B);
fcl::FCL_REAL BVRSSCollide(fcl::BVNode<fcl::OBBRSS> bvA, fcl::BVNode<fcl::OBBRSS> bvB);
fcl::FCL_REAL BVRSSCapsuleCollide(fcl::BVNode<fcl::OBBRSS> bvA, fcl::BVNode<fcl::OBBRSS> bvB);
fcl::FCL_REAL side2side(fcl::FCL_REAL rA, fcl::Vec3f TrA, fcl::FCL_REAL sideA, fcl::Vec3f axisA, fcl::FCL_REAL rB, fcl::Vec3f TrB, fcl::FCL_REAL sideB, fcl::Vec3f axisB);
fcl::FCL_REAL capsule2capsule(fcl::FCL_REAL rA, fcl::Vec3f TrA, fcl::FCL_REAL sideA, fcl::Vec3f axisA, fcl::FCL_REAL rB, fcl::Vec3f TrB, fcl::FCL_REAL sideB, fcl::Vec3f axisB);
cInterval disbound(cTMVector4 &p, cTMVector4 &q);
cInterval disbound(cTMVector4 &p, cTMVector4 &q, const int& typeA, const int& typeB);

void IDtypedecoding(std::string ID, int &link_type, int &__link);
void IDtypedecodingforyumi(std::string ID, int &link_type, int &__link);

fcl::FCL_REAL BVHTraversal(const int& rootA, const int &rootB, fcl::BVHModel<fcl::SPHERE> &A, fcl::BVHModel<fcl::SPHERE> &B);
fcl::FCL_REAL BVHTraversal(const int& rootA, const int &rootB, fcl::BVHModel<fcl::SPHERE> &A, fcl::BVHModel<fcl::SPHERE> &B, const int& typeA, const int& typeB);
fcl::FCL_REAL SPHERETREETraversal(std::vector<ball> &A, std::vector<ball> &B, bool onlyleaf);
fcl::FCL_REAL SPHERETREETraversal(const int& rootA, const int &rootB, std::vector<ball> &A, std::vector<ball> &B, const int& depth, const int& branch);
fcl::FCL_REAL BVSPHERECollide(fcl::BVNode<fcl::SPHERE> &bvA, fcl::BVNode<fcl::SPHERE> &bvB);
fcl::FCL_REAL BVSPHERECollide(fcl::BVNode<fcl::SPHERE> &bvA, fcl::BVNode<fcl::SPHERE> &bvB, const int &typeA, const int &typeB);
fcl::FCL_REAL BALLCollide(ball &a, ball &b);

void setupDualArm_TM2_HM_movable(int num, cInterval& interval, motion_TM2& sub_motion);
cTM2Matrix33 MEigen2cTM2(Eigen::Matrix3d& R);
cTM2Matrix33 Myaw_TM2(double omega, double q0);
fcl::FCL_REAL BVHTraversal_TM2(const int& rootA, const int &rootB, fcl::BVHModel<fcl::SPHERE> &A, fcl::BVHModel<fcl::SPHERE> &B);
fcl::FCL_REAL BVSPHERECollide_TM2(fcl::BVNode<fcl::SPHERE> &bvA, fcl::BVNode<fcl::SPHERE> &bvB);
cInterval disbound(cTM2Vector4 &p, cTM2Vector4 &q);

/// optimization
fcl::FCL_REAL SPHERETREETraversal(const int& rootA, const int &rootB,
                                  const int& depth, const int& branch, CCDTMData &cdata);
fcl::FCL_REAL BALLCollide(ball &a, ball &b, CCDTMData &cdata);
cInterval disbound(cTMVector4 &p, cTMVector4 &q, CCDTMData &cdata);

#endif // !__TAYLOR_FUNCTION_HPP__