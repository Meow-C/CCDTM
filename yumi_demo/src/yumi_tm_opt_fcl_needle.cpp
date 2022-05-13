//
// Created by monica on 2021/7/26.
//
// joint_state get from the motionx.txt file
// k is the number of lines, k-1 is the number of motion
// for motion i, (i = 1,2,...,k), line i the start state, line i+1 is the final state.
// MotionPlannerActionGoal

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/MoveGroupActionGoal.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <Eigen/Core>
#include <boost/filesystem.hpp>

#include "ModelParameters.h"
#include "Taylor_Function.h"
#include "cInterval.h"

#include "fcl/math/transform.h"
#include "fcl/collision_data.h"
#include "fcl/continuous_collision.h"
#include "fcl/ccd/motion_AM.h"

#include <iostream>
#include <fstream>
#include <vector>

const bool WRITE_REAL = false;
bool flag = false;
std::vector<double> vel;    // used for real distance
double interval = 0.1;
int num_parts = 10;
int num_exe = 10000;    // Release/execution frequency of each movement
double step;    // step = interval / num_exe;
double cur_time = 0;
std::vector<std::vector<motion> > sub_motion;
std::vector<std::vector<result> > collision_res;
const std::string WRITE_TARGET_DIR = "/home/monica/Desktop/LOG_INFO/yumi_demo_data/visual_st_effector";
const std::string MOTION_RESOURCES_DIR = "/home/monica/Desktop/LOG_INFO/motion_record";
const std::string MOTION_NAME = "motion15";
std::string OBJ_RESOURCES_DIR = "/home/monica/moveit_ws/src/yumi_lerp_planner/yumi_description/meshes/gripper/coarse";
const std::string SPHERETREE_RESOURCES_DIR= "/home/monica/spheretree/src/Model";
const std::string SPHERETREE_NAME = "finger-medial-depth8-branch2-unbiased.sph";
const int num_max_iterations = 300;

/// for this demo
int cur_Action = 0; // current action
int cur_Motion = 0;
std::vector<moveit_msgs::MoveGroupActionGoal> ActionGoal;   // save the data of Action, including start and final state of each motion

/// yumi_joint_1_l ,yumi_joint_2_l, yumi_joint_7_l, yumi_joint_3_l, yumi_joint_4_l, yumi_joint_5_l, yumi_joint_6_l
/// gripper_l_joint,         gripper_l_joint_m
/// (gripper_base->finger_r) (gripper_base->finger_l)
/// yumi_joint_1_r, yumi_joint_2_r, yumi_joint_7_r, yumi_joint_3_r, yumi_joint_4_r, yumi_joint_5_r, yumi_joint_6_r
/// gripper_r_joint, gripper_r_joint_m
/// gripper_*_joint: joint between base and right finger
/// gripper_*_joint_m: joint between base and left finger
sensor_msgs::JointState yumi_joint_state;

/// @brief variable of Transformation matrix for real distance, same as HM using Taylor Model
/// hm[i][j], i = 0 or 1, stands for left_arm and right_arm
/// hm[i][j], relative transformation from j-1 to j
Eigen::Matrix4d hm_world;
Eigen::Matrix4d hm[2][10];
Eigen::Matrix4d hm_hand[2];
Eigen::Matrix4d hm_leftfinger[2];
Eigen::Matrix4d hm_rightfinger[2];

/// @brief for fcl
std::vector<double>joint1_begin;
std::vector<double>joint1_end;
std::vector<double>joint2_begin;
std::vector<double>joint2_end;
std::vector<double>finger_joint1_begin;
std::vector<double>finger_joint1_end;
std::vector<double>finger_joint2_begin;
std::vector<double>finger_joint2_end;
std::vector<Eigen::Matrix4d> link1;
std::vector<Eigen::Matrix4d> link2;
Eigen::Matrix4d effector1;
Eigen::Matrix4d effector2;

std::vector<ball> fingerSphereTree;
std::vector<ball> handSphereTree;
int depth, branch;

void init_hm(int joint_num);
void init_fcl();
void fcl_joint_clear();
void init_();
double realdis(sensor_msgs::JointState &joint_state, int joint_num);
void cal_distance_function(std::vector<double> &start_state, std::vector<double> &vel);
void cal_TM_distance(std_msgs::Float64 &TMpoly, std_msgs::Float64 &TMupper, std_msgs::Float64 &TMlower);
void callback(const moveit_msgs::MoveGroupActionGoal& msg);
void sample(const moveit_msgs::MoveGroupActionGoal& msg);
void ReadMotionFile(std::string filename);
void ReadSphereTreeFile(std::string filename, std::vector<ball> &spheret);
void ReadLeafNodesFile(std::string filename, std::vector<ball> &spheret);
void WriteTMdis();
void Writerealdis();
bool collisioncallback(cInterval &T, result &collisionresult);
bool fclcallback(cInterval &T);
void choosefinger(Eigen::Matrix4d& finger1, Eigen::Matrix4d& finger2,
                  double& finger_joint1_begin_, double& finger_joint1_end_, double& finger_joint2_begin_, double& finger_joint2_end_);

bool fclcallback(cInterval &T)
{
    std::string filename = OBJ_RESOURCES_DIR + "/finger.obj";

    std::string outfile;
    outfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "fclnaive_time_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    std::ofstream out(outfile, std::ios::app);

    std::string outfile2;
    outfile2 = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "fclca_time_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    std::ofstream out2(outfile2, std::ios::app);

    // set mesh triangles and vertices indices
    std::vector<fcl::Vec3f> p1;
    std::vector<fcl::Triangle> t1;
    // code to set the vertices and triangles
    loadOBJFile(filename.c_str(), p1, t1);

    typedef fcl::BVHModel<fcl::OBBRSS> Model;
    std::shared_ptr<Model> geom1 = std::make_shared<Model>();
    // add the mesh data into the BVHModel structure
    geom1->beginModel();
    geom1->addSubModel(p1, t1);
    geom1->endModel();

    fcl::Matrix3f R1;
    R1.setIdentity();
    fcl::Vec3f T1;
    T1.setZero();
    fcl::Transform3f pose1;
    pose1.setTransform(R1,T1);
    fcl::CollisionObject* o1 = new fcl::CollisionObject(geom1, pose1);
    fcl::Matrix3f R2;
    R2.setIdentity();
    fcl::Vec3f T2;
    T2.setZero();
    fcl::Transform3f pose2;
    pose2.setTransform(R2,T2);
    fcl::CollisionObject* o2 = new fcl::CollisionObject(geom1, pose2);


    double time[2];
    time[0] = T.i[0];
    time[1] = T.i[1];
    Eigen::Matrix4d finger1, finger2;
    double finger_joint1_begin_, finger_joint1_end_, finger_joint2_begin_, finger_joint2_end_;

    link_type[0] = 2;
    link_type[1] = 2;
    choosefinger(finger1, finger2, finger_joint1_begin_, finger_joint1_end_, finger_joint2_begin_, finger_joint2_end_);
    fcl::MotionAM motion1_1(time, joint1_begin, finger_joint1_begin_, joint1_end, finger_joint1_end_,
                            hm_world, link1, hm_hand[0], finger1, effector1);
    fcl::MotionAM motion2_1(time, joint2_begin, finger_joint2_begin_, joint2_end, finger_joint2_end_,
                            hm_world, link2, hm_hand[1], finger2, effector2);
    motion1_1.setTimeInterval(T.i[0], T.i[1]);
    motion2_1.setTimeInterval(T.i[0], T.i[1]);
    fcl::ContinuousCollisionRequest req(num_max_iterations, 0.00000001, fcl::GST_LIBCCD);
    fcl::ContinuousCollisionResult res;

    clock_t start = clock();
    fcl::FCL_REAL dis = fcl::continuousCollideNaive(o1->getCollisionGeometry(), &motion1_1,
                                                    o2->getCollisionGeometry(), &motion2_1,
                                                    req,res);
    clock_t end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL: Collision found between left_gripper leftfinger & right_gripper leftfinger.");
    }

    ROS_INFO("FCL: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out << double(end-start) / CLOCKS_PER_SEC << " ";

    start = clock();
    dis = fcl::continuousCollideConservativeAdvancement(o1->getCollisionGeometry(), &motion1_1,
                                                        o2->getCollisionGeometry(), &motion2_1,
                                                        req,res);

    end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL_CA: Collision found between left_gripper leftfinger & right_gripper leftfinger.");
    }

    ROS_INFO("FCL_CA: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out2 << double(end-start) / CLOCKS_PER_SEC << " ";

    link_type[0] = 2;
    link_type[1] = 3;
    choosefinger(finger1, finger2, finger_joint1_begin_, finger_joint1_end_, finger_joint2_begin_, finger_joint2_end_);
    fcl::MotionAM motion1_2(time, joint1_begin, finger_joint1_begin_, joint1_end, finger_joint1_end_,
                            hm_world, link1, hm_hand[0], finger1, effector1);
    fcl::MotionAM motion2_2(time, joint2_begin, finger_joint2_begin_, joint2_end, finger_joint2_end_,
                            hm_world, link2, hm_hand[1], finger2, effector2);
    motion1_2.setTimeInterval(T.i[0], T.i[1]);
    motion2_2.setTimeInterval(T.i[0], T.i[1]);
    start = clock();
    dis = fcl::continuousCollideNaive(o1->getCollisionGeometry(), &motion1_2, o2->getCollisionGeometry(), &motion2_2, req,res);
    end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL: Collision found between left_gripper leftfinger & right_gripper rightfinger.");
    }

    ROS_INFO("FCL: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out << double(end-start) / CLOCKS_PER_SEC << " ";

    start = clock();
    dis = fcl::continuousCollideConservativeAdvancement(o1->getCollisionGeometry(), &motion1_2, o2->getCollisionGeometry(), &motion2_2, req,res);
    end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL_CA: Collision found between left_gripper leftfinger & right_gripper rightfinger.");
    }

    ROS_INFO("FCL_CA: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out2 << double(end-start) / CLOCKS_PER_SEC << " ";

    link_type[0] = 3;
    link_type[1] = 2;
    choosefinger(finger1, finger2, finger_joint1_begin_, finger_joint1_end_, finger_joint2_begin_, finger_joint2_end_);
    fcl::MotionAM motion1_3(time, joint1_begin, finger_joint1_begin_, joint1_end, finger_joint1_end_,
                            hm_world, link1, hm_hand[0], finger1, effector1);
    fcl::MotionAM motion2_3(time, joint2_begin, finger_joint2_begin_, joint2_end, finger_joint2_end_,
                            hm_world, link2, hm_hand[1], finger2, effector2);
    motion1_3.setTimeInterval(T.i[0], T.i[1]);
    motion2_3.setTimeInterval(T.i[0], T.i[1]);
    start = clock();
    dis = fcl::continuousCollideNaive(o1->getCollisionGeometry(), &motion1_3, o2->getCollisionGeometry(), &motion2_3, req,res);
    end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL: Collision found between left_gripper rightfinger & right_gripper leftfinger.");
    }

    ROS_INFO("FCL: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out << double(end-start) / CLOCKS_PER_SEC << " ";

    start = clock();
    dis = fcl::continuousCollideConservativeAdvancement(o1->getCollisionGeometry(), &motion1_3, o2->getCollisionGeometry(), &motion2_3, req,res);
    end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL_CA: Collision found between left_gripper rightfinger & right_gripper leftfinger.");
    }

    ROS_INFO("FCL_CA: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out2 << double(end-start) / CLOCKS_PER_SEC << " ";

    link_type[0] = 3;
    link_type[1] = 3;
    choosefinger(finger1, finger2, finger_joint1_begin_, finger_joint1_end_, finger_joint2_begin_, finger_joint2_end_);
    fcl::MotionAM motion1_4(time, joint1_begin, finger_joint1_begin_, joint1_end, finger_joint1_end_,
                            hm_world, link1, hm_hand[0], finger1, effector1);
    fcl::MotionAM motion2_4(time, joint2_begin, finger_joint2_begin_, joint2_end, finger_joint2_end_,
                            hm_world, link2, hm_hand[1], finger2, effector2);
    motion1_4.setTimeInterval(T.i[0], T.i[1]);
    motion2_4.setTimeInterval(T.i[0], T.i[1]);
    start = clock();
    dis = fcl::continuousCollideNaive(o1->getCollisionGeometry(), &motion1_4, o2->getCollisionGeometry(), &motion2_4, req,res);
    end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL: Collision found between left_gripper rightfinger & right_gripper rightfinger.");
    }

    ROS_INFO("FCL: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out << double(end-start) / CLOCKS_PER_SEC << "\n";

    start = clock();
    dis = fcl::continuousCollideConservativeAdvancement(o1->getCollisionGeometry(), &motion1_4, o2->getCollisionGeometry(), &motion2_4, req,res);
    end = clock();

    if (res.is_collide) {
        ROS_INFO("FCL_CA: Collision found between left_gripper rightfinger & right_gripper rightfinger.");
    }

    ROS_INFO("FCL_CA: detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    out2 << double(end-start) / CLOCKS_PER_SEC << "\n";

    out.close();
    out2.close();
}

void choosefinger(Eigen::Matrix4d& finger1, Eigen::Matrix4d& finger2,
                  double& finger_joint1_begin_, double& finger_joint1_end_, double& finger_joint2_begin_, double& finger_joint2_end_) {
    if (link_type[0] == 2) {
        // left finger
        finger1 = hm_leftfinger[0];
        finger_joint1_begin_ = finger_joint1_begin[1];
        finger_joint1_end_ = finger_joint1_end[1];
    }
    else if (link_type[0] == 3) {
        // right finger
        finger1 = hm_rightfinger[0];
        finger_joint1_begin_ = finger_joint1_begin[0];
        finger_joint1_end_ = finger_joint1_end[0];
    }
    else {
        std::cerr << "Error on choosing obj\n";
    }
    if (link_type[1] == 2) {
        // left finger
        finger2 = hm_leftfinger[1];
        finger_joint2_begin_ = finger_joint2_begin[1];
        finger_joint2_end_ = finger_joint2_end[1];
    }
    else if (link_type[1] == 3) {
        // right finger
        finger2 = hm_rightfinger[1];
        finger_joint2_begin_ = finger_joint2_begin[0];
        finger_joint2_end_ = finger_joint2_end[0];
    }
    else {
        std::cerr << "Error on choosing obj\n";
    }
}

void ReadMotionFile(std::string filename)
{
    std::string filepath = MOTION_RESOURCES_DIR + "/" + filename;
    std::vector<std::vector <double> > joint;

    char buffer[256];
    std::ifstream infile;
    infile.open(filepath, std::ios::in);
    if (! infile.is_open())
        std::cerr << "Error opening file";

    std::vector<std::string> joint_name;
    for (int i = 0; i < 2; i++) {
        infile.getline(buffer, 256);
        std::string str(buffer, buffer+strlen(buffer));
        char cstr[256];
        strcpy(cstr, str.c_str());

        char *p = strtok(cstr, ", ");
        while (p!=0) {
            joint_name.push_back(p);
            p = strtok(NULL, ", ");
        }
    }

    while (!infile.eof())
    {
        infile.getline(buffer, 256);
//        std::cout << buffer << "\n";
        std::string s(buffer, buffer+strlen(buffer));
        if (strcmp(s.data(), "") == 0)
            continue;
        char cstr[256];
        std::strcpy(cstr, s.c_str());
        std::vector<double> tmp;

        char *p = std::strtok(cstr, " ");
        while (p!=0) {
            tmp.push_back(atof(p));
            p = strtok(NULL, " ");
        }
        joint.push_back(tmp);
    }

    int cnt = 1;

    // joint into ActionGoal
    for (;cnt < joint.size(); cnt++) {
        moveit_msgs::MoveGroupActionGoal tmp;
        std::vector<moveit_msgs::JointConstraint> joint_constraints;
        moveit_msgs::Constraints goal_constraints;
        tmp.header.stamp = ros::Time::now();
        tmp.goal_id.id = cnt;
        for (int i = 0; i < joint[cnt].size(); i++) {
            tmp.goal.request.start_state.joint_state.name.push_back(joint_name[i]);
            tmp.goal.request.start_state.joint_state.position.push_back(joint[cnt - 1][i]);
            moveit_msgs::JointConstraint joint_tmp;
            joint_tmp.position = joint[cnt][i];
            goal_constraints.joint_constraints.push_back(joint_tmp);
        }
        tmp.goal.request.goal_constraints.push_back(goal_constraints);
        ActionGoal.push_back(tmp);
        // check each ActionGoal whether correct
//        std::cout << "ActionGoal: " << tmp.goal_id.id.data() << "\n";
//        std::cout << "start position:\n";
//        for (int i = 0; i < joint[cnt].size(); i++) {
//            std::cout << ActionGoal[cnt-1].goal.request.start_state.joint_state.position[i] << " ";
//        }
//        std::cout << "\n";
//        std::cout << "goal position: \n";
//        for (int i = 0; i < joint[cnt].size(); i++) {
//            std::cout << ActionGoal[cnt-1].goal.request.goal_constraints[0].joint_constraints[i].position << " ";
//        }
//        std::cout << "\n";
    }

    infile.close();
}

/// @brief read sphere tree file (http://isg.cs.tcd.ie/spheretree)
/// including root and leaf nodes
void ReadSphereTreeFile(std::string filename, std::vector<ball> &spheret) {
    std::ifstream in(filename, std::ios::in);

    if (!in.is_open())
        std::cerr << "Error opening SphereTree file\n";

    char buffer[250];
    int cnt = 0;
    in.getline(buffer, 250);
    sscanf(buffer, "%d %d", &depth, &branch);

    while (!in.eof()) {
        in.getline(buffer, 250);
        ball tmp;
        double singalbit;
        sscanf(buffer, "%lf %lf %lf %lf %lf", &tmp.x, &tmp.y, &tmp.z, &tmp.r, &singalbit);
        spheret.push_back(tmp);
    }

    in.close();
}

/// @brief read sphere tree file (http://isg.cs.tcd.ie/spheretree)
/// only including leaf nodes
void ReadLeafNodesFile(std::string filename, std::vector<ball> &spheret) {
    std::ifstream in(filename, std::ios::in);

    if (!in.is_open())
        std::cerr << "Error opening SphereTree file\n";

    char buffer[250];
    int depth, branch;
    int cnt = 0;
    in.getline(buffer, 250);
    sscanf(buffer, "%d %d", &depth, &branch);
    int firstnode = (pow(branch, depth-1) - 1) / (branch - 1);

    while (!in.eof()) {
        in.getline(buffer, 250);
        if (cnt < firstnode) cnt++;
        else {
            ball tmp;
            double singalbit;
            sscanf(buffer, "%lf %lf %lf %lf %lf", &tmp.x, &tmp.y, &tmp.z, &tmp.r, &singalbit);
            if (tmp.r > 0)
                spheret.push_back(tmp);
            cnt++;
        }
    }

    in.close();
}

void WriteTMdis() {
    std::string TMfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "TMdis_depth" + std::to_string(depth-1) + "branch" + std::to_string(branch) + ".txt";
    std::ofstream out(TMfile, std::ios::out);
    for (int i = 0; i < sub_motion.size(); i++) {
        for (int j = 0; j < sub_motion[i].size(); j++) {
            out << sub_motion[i][j].time_interval.i[0] << " " << sub_motion[i][j].time_interval.i[1]
                << " " << sub_motion[i][j].dis_square.c[0] << " " << sub_motion[i][j].dis_square.c[1]
                << " " << sub_motion[i][j].dis_square.c[2] << " " << sub_motion[i][j].dis_square.c[3]
                << " " << sub_motion[i][j].dis_square.r.i[0] << " " << sub_motion[i][j].dis_square.r.i[1] << "\n";
        }
    }
    out.close();
}

void Writerealdis() {
    std::string hmfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "hml.txt";
    std::ofstream out1(hmfile, std::ios::out);
    // hm_world fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out1 << hm_world(i,j) << " ";
        }
    }
    out1 << "\n";
    // hm dynamic
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                out1 << hm[0][i](j,k) << " ";
            }
        }
        out1 << "\n";
    }
    // hm_hand fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out1 << hm_hand[0](i,j) << " ";
        }
    }
    // hm_leftfinger fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out1 << hm_leftfinger[0](i,j) << " ";
        }
    }
    // hm_rightfinger fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out1 << hm_rightfinger[0](i,j) << " ";
        }
    }
    out1.close();

    hmfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "hmr.txt";
    std::ofstream out2(hmfile, std::ios::out);
    // hm_world fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out2 << hm_world(i,j) << " ";
        }
    }
    out2 << "\n";
    // hm dynamic
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                out2 << hm[1][i](j,k) << " ";
            }
        }
        out2 << "\n";
    }
    // hm_hand fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out2 << hm_hand[1](i,j) << " ";
        }
    }
    // hm_leftfinger fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out2 << hm_leftfinger[1](i,j) << " ";
        }
    }
    // hm_rightfinger fixed
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out2 << hm_rightfinger[1](i,j) << " ";
        }
    }
    out2.close();

    std::string jointfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "joint.txt";
    std::ofstream out3(jointfile, std::ios::out);
    for (int i = 0; i < sub_motion.size(); i++) {
        for (int j = 0; j < sub_motion[i].size(); j++) {
            for (int k = 0; k < sub_motion[i][j].start_state.size(); k++) {
                out3 << sub_motion[i][j].start_state[k];
                if (k < sub_motion[i][j].start_state.size()-1)
                    out3 << " ";
            }
            out3 << "\n";
            for (int k = 0; k < sub_motion[i][j].goal_state.size(); k++) {
                out3 << sub_motion[i][j].goal_state[k];
                if (k < sub_motion[i][j].goal_state.size()-1)
                    out3 << " ";
            }
            out3 << "\n";
        }
    }
    out3.close();
}

/// @brief init for real distance function
void init_hm(int joint_num)
{
    Eigen::Matrix3d  R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    double xyz[3] = {0,0,0.1};

    // world to yumi_body
    hm_world << R(0,0), R(0,1), R(0,2), xyz[0],
            R(1,0), R(1,1), R(1,2), xyz[1],
            R(2,0), R(2,1), R(2,2), xyz[2],
            0,0,0,1;

    // link i-1 to link i
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < joint_num; j++) {
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);

            hm[i][j] << R1(0,0), R1(0,1), R1(0,2), frame[i][j].xyz[0],
                    R1(1,0), R1(1,1), R1(1,2), frame[i][j].xyz[1],
                    R1(2,0), R1(2,1), R1(2,2), frame[i][j].xyz[2],
                    0,0,0,1;
            if (i == 0) {
                link1.push_back(hm[i][j]);
            }
            else {
                link2.push_back(hm[i][j]);
            }
        }
    }

    // link to gripper
    // assume, the gripper is fixed
    Eigen::Matrix3d  R1 = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    double xyz1[3] = {0,0,0.007};
    Eigen::Matrix4d tmp;
    tmp << R1(0,0), R1(0,1), R1(0,2), xyz1[0],
            R1(1,0), R1(1,1), R1(1,2), xyz1[1],
            R1(2,0), R1(2,1), R1(2,2), xyz1[2],
            0,0,0,1;
    hm_hand[0] = tmp;
    hm_hand[1] = tmp;

    // base to finger
    Eigen::Matrix3d Rl = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    double xyzl[3] = {0.0, 0.0065, 0.0837};
    Eigen::Matrix4d leftfinger;
    leftfinger << Rl(0,0), Rl(0,1), Rl(0,2), xyzl[0],
            Rl(1,0), Rl(1,1), Rl(1,2), xyzl[1],
            Rl(2,0), Rl(2,1), Rl(2,2), xyzl[2],
            0,0,0,1;

    Eigen::Matrix3d Rr = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    double xyzr[3] = {0.0, -0.0065, 0.0837};
    Eigen::Matrix4d rightfinger;
    rightfinger << Rr(0,0), Rr(0,1), Rr(0,2), xyzr[0],
            Rr(1,0), Rr(1,1), Rr(1,2), xyzr[1],
            Rr(2,0), Rr(2,1), Rr(2,2), xyzr[2],
            0,0,0,1;

    hm_leftfinger[0] = leftfinger;
    hm_rightfinger[0] = rightfinger;
    hm_leftfinger[1] = leftfinger;
    hm_rightfinger[1] = rightfinger;
}

void init_fcl()
{
    joint1_begin.clear();
    joint1_end.clear();
    joint2_begin.clear();
    joint2_end.clear();
    finger_joint1_begin.clear();
    finger_joint1_end.clear();
    finger_joint2_begin.clear();
    finger_joint2_end.clear();
    link1.clear();
    link2.clear();
    effector1.setIdentity();
    effector2.setIdentity();
}

void fcl_joint_clear() {
    joint1_begin.clear();
    joint1_end.clear();
    joint2_begin.clear();
    joint2_end.clear();
    finger_joint1_begin.clear();
    finger_joint1_end.clear();
    finger_joint2_begin.clear();
    finger_joint2_end.clear();
}

/// @brief init the translation matrix for real distance and frame from txt file and used for distance function
void init_()
{
    std::string fingerfile = SPHERETREE_RESOURCES_DIR + "/" + SPHERETREE_NAME;
    ReadSphereTreeFile(fingerfile, fingerSphereTree);

    std::string outfile0 = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "fclca_time_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    std::string outfile1 = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "fclnaive_time_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    std::string outfile2 = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "time_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    std::string outfile3 = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "num_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    std::ofstream out0(outfile0);
    std::ofstream out1(outfile1);
    std::ofstream out2(outfile2);
    std::ofstream out3(outfile3);
    out0.close();
    out1.close();
    out2.close();
    out3.close();

    init_fcl();

    std::string filename = MOTION_NAME + ".txt";
    ReadMotionFile(filename);

    ReadUrdfFile(file_l,0);
    ReadUrdfFile(file_r,1);

    int joint_num = 7;
    // init and setup the yaw
    init_yaw();

    // init hm
    init_hm(joint_num);

    for (int i = 0; i < ActionGoal.size(); i++) {
        vel.clear();
        sample(ActionGoal[i]);
    }

    WriteTMdis();
    Writerealdis();

    cur_Action = 0;
    ROS_INFO("-------- Initialization & Collision Detection Over! --------");
}

double realdis(sensor_msgs::JointState &joint_state, int joint_num)
{
    Eigen::Matrix4d hm0 = hm_world;
    Eigen::Matrix4d hm1 = hm_world;

    int half = joint_state.position.size() / 2;

    for (int i = 0; i < joint_num; i++) {
        double c0 = cos(joint_state.position[i]);
        double s0 = sin(joint_state.position[i]);

        Eigen::Matrix4d tmp0;
        tmp0 << c0, -s0, 0, 0,
                s0, c0, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        hm0 = hm0 * hm[0][i] * tmp0;
//        std::cout << hm0;
//        std::cout << "\n";

        double c1 = cos(joint_state.position[i + half]);
        double s1 = sin(joint_state.position[i + half]);
        Eigen::Matrix4d tmp1;
        tmp1 << c1, -s1, 0, 0,
                s1, c1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        hm1 = hm1 * hm[1][i] * tmp1;
//        std::cout << hm1;
//        std::cout << "\n";
    }

    // hm_finger_yaw
    Eigen::Matrix4d ll, rr;
    Eigen::Matrix4d lr, rl;
    ll << 1, 0, 0, -1 * joint_state.position[joint_num + 1],
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

//    lr << 1, 0, 0, -1 * joint_state.position[joint_num],
//          0, 1, 0, 0,
//          0, 0, 1, 0,
//          0, 0, 0, 1;
//
//    rl << 1, 0, 0, -1 * joint_state.position[joint_num*2+3],
//          0, 1, 0, 0,
//          0, 0, 1, 0,
//          0, 0, 0, 1;

    rr << 1, 0, 0, -1 * joint_state.position[joint_num*2+2],
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    hm0 = hm0 * hm_hand[0] * hm_leftfinger[0] * ll;
    hm1 = hm1 * hm_hand[1] * hm_rightfinger[1] * rr;

    Eigen::Vector4d p,q;
//    p << 0,0,0,1;
//    q << 0,0,0,1;
    p << -0.0004065, 0.001969, 0.0259665, 1;
    q << -0.0004065, 0.001969, 0.0259665, 1;

    p = hm0 * p;
    q = hm1 * q;

//    std::cout << "\npoint p(ll): \n";
//    std::cout << p;
//    std::cout << "\npoint q(rr): \n";
//    std::cout << q;

    Eigen::Vector4d d = p - q;

    if (WRITE_REAL) {
        std::string filename = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/position_at_time0.txt";
        std::ofstream out(filename, std::ios::app);
        Eigen::Vector4d p_ll,q_rr;
        p_ll << 0,0,0,1;
        q_rr << 0,0,0,1;
        p_ll = hm0 * p_ll;
        q_rr = hm1 * q_rr;
        out << "p: " << p_ll(0) << " " << p_ll(1) << " " << p_ll(2) << "\n";
        out << "q: " << q_rr(0) << " " << q_rr(1) << " " << q_rr(2) << "\n\n";
        out.close();
    }

//    return sqrt(d(0) * d(0) + d(1) * d(1) + d(2) * d(2));
    return sqrt(d(0) * d(0) + d(1) * d(1) + d(2) * d(2)) - 0.026765925437017866 * 2;
}

void cal_distance_function(std::vector<double> &start_state, std::vector<double> &vel)
{
    std::vector<motion> subm;
    std::vector<result> res;
    double segment_interval = interval / num_parts;
    for (int i = 0; i < num_parts; i++) {
        fcl_joint_clear();
        cInterval T(0, segment_interval);
        setGlobalTimeInterval(T.i[0],T.i[1]);

        motion tmp;
        tmp.time_interval.i[0] = segment_interval * i;
        tmp.time_interval.i[1] = segment_interval * (i+1);
//        ROS_INFO("sub-motion during time interval [%lf, %lf]", tmp.time_interval.i[0], tmp.time_interval.i[1]);
        for (int j = 0; j < start_state.size(); j++) {
            tmp.start_state.push_back(start_state[j] + vel[j] * i * segment_interval);
            tmp.goal_state.push_back(start_state[j] + vel[j] * (i + 1.0) * segment_interval);
            tmp.velocity.push_back(vel[j]);
//            ROS_INFO("joint %d: [%lf, %lf], v = %lf m/s", j+1, start_state[j] + vel[j] * i * segment_interval, start_state[j] + vel[j] * (i + 1.0) * segment_interval, vel[j]);
//            ROS_INFO("joint %d: [%lf, %lf], v = %lf m/s", j+1, tmp.start_state[j], tmp.goal_state[j], tmp.velocity[j]);
            // for fcl
            if (j < (start_state.size()/2 - 2)) {
                joint1_begin.push_back(tmp.start_state[j]);
                joint1_end.push_back(tmp.goal_state[j]);
            }
            else if (j < start_state.size()/2) {
                finger_joint1_begin.push_back(tmp.start_state[j]);
                finger_joint1_end.push_back(tmp.goal_state[j]);
            }
            else if (j < (start_state.size() - 2)) {
                joint2_begin.push_back(tmp.start_state[j]);
                joint2_end.push_back(tmp.goal_state[j]);
            }
            else if (j < start_state.size()) {
                finger_joint2_begin.push_back(tmp.start_state[j]);
                finger_joint2_end.push_back(tmp.goal_state[j]);
            }
            else {
                std::cerr << "Error on cal_distance_function\n";
            }
        }

        setupdualHM_movable(7, T, tmp);

//        cTMVector4 p(0,0,0,1);
//        cTMVector4 q(0,0,0,1);
        cTMVector4 p(-0.0004065, 0.001969, 0.0259665,1);
        cTMVector4 q(-0.0004065, 0.001969, 0.0259665,1);
        p = M44multV14(HMleftfinger[0] ,p);
        q = M44multV14(HMrightfinger[1] ,q);
        cTaylorModel3 x = p.P[0] - q.P[0];
        cTaylorModel3 y = p.P[1] - q.P[1];
        cTaylorModel3 z = p.P[2] - q.P[2];
        tmp.dis_square = x * x + y * y + z * z;

//        ROS_INFO("p position: \n");
//        p.P[0].print();
//        p.P[1].print();
//        p.P[2].print();
//        ROS_INFO("q position: \n");
//        q.P[0].print();
//        q.P[1].print();
//        q.P[2].print();

        subm.push_back(tmp);
//        ROS_INFO("dis_quare %d:", i);
//        tmp.dis_square.print();

        ROS_INFO("sub-interval %d in Action %d:", i+1, cur_Action+1);
        result tmp_res;
        collisioncallback(T, tmp_res);
        fclcallback(T);
        res.push_back(tmp_res);
    }
    sub_motion.push_back(subm);
    collision_res.push_back(res);
}

void cal_TM_distance(std_msgs::Float64 &TMpoly, std_msgs::Float64 &TMupper, std_msgs::Float64 &TMlower)
{
    int i = floor(cur_time / (interval / num_parts));
//    ROS_INFO("Now in the No.%d segment motion.", i + 1);

    double t = cur_time - i * (interval / num_parts);

    cInterval disTM_cur = sub_motion[cur_Action][i].dis_square.evaluate(t);

    TMpoly.data = sub_motion[cur_Action][i].dis_square.c[0]
                  + t * (sub_motion[cur_Action][i].dis_square.c[1]
                         + t * (sub_motion[cur_Action][i].dis_square.c[2]
                                + t * (sub_motion[cur_Action][i].dis_square.c[3])));

    if (TMpoly.data < 0) {
        TMpoly.data = -1.0 * sqrt(-1 * TMpoly.data) - 0.026765925437017866 * 2;
    }
    else {
        TMpoly.data = sqrt(TMpoly.data) - 0.026765925437017866 * 2;
    }

    TMlower.data = disTM_cur.i[0];
    if (TMlower.data < 0) {
        TMlower.data = -1.0 * sqrt(-1 * TMlower.data) - 0.026765925437017866 * 2;
    }
    else {
        TMlower.data = sqrt(TMlower.data) - 0.026765925437017866 * 2;
    }

    TMupper.data = disTM_cur.i[1];
    if (TMupper.data < 0) {
        TMupper.data = -1.0 * sqrt(-1 * TMupper.data) - 0.026765925437017866 * 2;
    }
    else {
        TMupper.data = sqrt(TMupper.data) - 0.026765925437017866 * 2;
    }
}

void sample(const moveit_msgs::MoveGroupActionGoal& msg)
{
    actionlib_msgs::GoalID_<std::allocator<void> > goal_id = msg.goal_id;
    ROS_INFO("I read %s", goal_id.id.data());

    /// get the start state from MotionPlannerActionGoal
    moveit_msgs::RobotState start_state = msg.goal.request.start_state;
    int joint_num = start_state.joint_state.position.size();
//    ROS_INFO("start_state:");
//    for (int i = 0; i < joint_num; i++){
//        ROS_INFO("position: %f", msg.goal.request.start_state.joint_state.position[i]);
//    }

    /// get the motion arm group, only one group, left arm or right arm
//    ROS_INFO("MotionPlanRequest:");
    moveit_msgs::MotionPlanRequest req = msg.goal.request;
    std::vector<moveit_msgs::JointConstraint> req_constraints = req.goal_constraints[0].joint_constraints;
//    for (int i = 0; i < joint_num; i++) {
//        ROS_INFO("position: %lf", req_constraints[i].position);
//    }

    /// The above is only to display the information in msg having no practical meaning, and can be annotated
    /// @brief According to the msg, reproduce the uniform velocity motion, and calculate the distance function
    /// using Taylor Model.
    /// Publish the joint_state
    for (int i = 0; i < joint_num; i++) {
        vel.push_back((req_constraints[i].position - start_state.joint_state.position[i]) / interval);
        // init for TM
//        ROS_INFO("frame[i][j]: yaw0, yaw1, yaw_omega.");
        if (i < joint_num/2) {
            frame[0][i].yaw0 = start_state.joint_state.position[i];
            frame[0][i].yaw1 = req_constraints[i].position;
            frame[0][i].yaw_omega = (frame[0][i].yaw1 - frame[0][i].yaw0) / interval;
//            ROS_INFO("frame[0][%d]: %lf, %lf, %lf", i, frame[0][i].yaw0, frame[0][i].yaw1, frame[0][i].yaw_omega);
        }
        else {
            frame[1][i - joint_num/2].yaw0 = start_state.joint_state.position[i];
            frame[1][i - joint_num/2].yaw1 = req_constraints[i].position;
            frame[1][i - joint_num/2].yaw_omega = (frame[1][i - joint_num/2].yaw1 - frame[1][i - joint_num/2].yaw0) / interval;
//            ROS_INFO("frame[1][%d]: %lf, %lf, %lf", i - joint_num/2, frame[1][i - joint_num/2].yaw0,
//                     frame[1][i - joint_num/2].yaw1, frame[1][i - joint_num/2].yaw_omega);
        }
        // init for TM end
    }


//    /// init the joint_state which will be published later.
//    yumi_joint_state.header.stamp = ros::Time::now();
//    for (int i = 0; i < joint_num; i++) {
//        yumi_joint_state.name.push_back(start_state.joint_state.name[i].data());
//        yumi_joint_state.position.push_back(start_state.joint_state.position[i]);
//    }

    step = interval / (num_parts * num_exe);

    cal_distance_function(start_state.joint_state.position, vel);
    cur_Action++;
}

bool collisioncallback(cInterval &T, result &collisionresult) {
    // only check the gripper group
    // for motion : sub_motion
    //     for obj1 : left_gripper
    //         for obj2: right_gripper
    //
    // refer to continuousCollideLerp(const CollisionGeometry* o1, const LerpMotion* motion1,
    //                                const CollisionGeometry* o2, const LerpMotion* motion2,
    //                                const ContinuousCollisionRequest& request,
    //                                ContinuousCollisionResult& result)
    //
    // o1, o2 never used.
    // motion1, motion2, used to calculate the velocity and start&final state, no needed here.
    // request, used for time interval, toc_err, name of pairs(no needed here), marker bit(no needed here)
    // result, need a data struct to save the time and pairs of collision.

    // TODO: change BVH from ball(sphere tree, load data in init_())

    // updata CCDTMData, put the parameters in the struct CCDTMData, no global parameters
    CCDTMData cdata(HMbody, HM, HMhand, HMleftfinger, HMrightfinger);
    cdata.tra_num = 0;
    cdata.A = fingerSphereTree;
    cdata.B = fingerSphereTree;

    /// @brief record running time
    bool only_time = true;
//    std::string outfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "time.txt";
    std::string outfile;
    std::string outfile2;
    if (only_time)
    {
        outfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "time_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
        outfile2 = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "num_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    }
    else
        outfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + "timeandnum_depth" + std::to_string(depth-1)+ "branch" + std::to_string(branch) + ".txt";
    std::ofstream out(outfile, std::ios::app);
    /// record running time
    std::ofstream out2(outfile2, std::ios::app);

    setGlobalTimeInterval(T.i[0],T.i[1]);
//    __T = T;
//    __toc_err = 0.00000001;
    cdata.__T = T;
    cdata.__toc_err = 0.00000001;

//    link_type[0] = 2;
//    link_type[1] = 2;
//    tra_num = 0;
    cdata.link_type[0] = 2;
    cdata.link_type[1] = 2;
    cdata.tra_num = 0;
    clock_t start = clock();
//    fcl::FCL_REAL dis = SPHERETREETraversal(fingerSphereTree, fingerSphereTree, false);
    fcl::FCL_REAL dis = SPHERETREETraversal(0, 0, depth, branch, cdata);
    clock_t end = clock();
//    ROS_INFO("check collision between left_gripper leftfinger & right_gripper leftfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("leftfinger");
        collisionresult.pairB.push_back("leftfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper leftfinger & right_gripper leftfinger.");
    }
    std::cout << "dis: " << dis << " m \n";
    std::cout << "The number of pairs of spheres has been detected: " << cdata.tra_num << "\n";
    ROS_INFO("detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    if (only_time)
    {
        out << double(end-start) / CLOCKS_PER_SEC << " ";
        out2 << cdata.tra_num << " ";
    }
    else
        out << "[" << double(end-start) / CLOCKS_PER_SEC << "," << cdata.tra_num << "," << (double(end-start) / CLOCKS_PER_SEC)/cdata.tra_num << "] ";

    cdata.link_type[0] = 2;
    cdata.link_type[1] = 3;
    cdata.tra_num = 0;
    start = clock();
    dis = SPHERETREETraversal(0, 0, depth, branch, cdata);
    end = clock();
//    ROS_INFO("check collision between left_gripper leftfinger & right_gripper rightfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("leftfinger");
        collisionresult.pairB.push_back("rightfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper leftfinger & right_gripper rightfinger.");
    }
    std::cout << "dis: " << dis << " m \n";
    std::cout << "The number of pairs of spheres has been detected: " << cdata.tra_num << "\n";
    ROS_INFO("detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    if (only_time)
    {
        out << double(end-start) / CLOCKS_PER_SEC << " ";
        out2 << cdata.tra_num << " ";
    }
    else
        out << "[" << double(end-start) / CLOCKS_PER_SEC << "," << cdata.tra_num << "," << (double(end-start) / CLOCKS_PER_SEC)/cdata.tra_num << "] ";

    cdata.link_type[0] = 3;
    cdata.link_type[1] = 2;
    cdata.tra_num = 0;
    start = clock();
    dis = SPHERETREETraversal(0, 0, depth, branch, cdata);
    end = clock();
//    ROS_INFO("check collision between left_gripper rightfinger & right_gripper leftfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("rightfinger");
        collisionresult.pairB.push_back("leftfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper rightfinger & right_gripper leftfinger.");
    }
    std::cout << "dis: " << dis << " m \n";
    std::cout << "The number of pairs of spheres has been detected: " << cdata.tra_num << "\n";
    ROS_INFO("detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    if (only_time)
    {
        out << double(end-start) / CLOCKS_PER_SEC << " ";
        out2 << cdata.tra_num << " ";
    }
    else
        out << "[" << double(end-start) / CLOCKS_PER_SEC << "," << cdata.tra_num << "," << (double(end-start) / CLOCKS_PER_SEC)/cdata.tra_num << "] ";

    cdata.link_type[0] = 3;
    cdata.link_type[1] = 3;
    cdata.tra_num = 0;
    start = clock();
    dis = SPHERETREETraversal(0, 0, depth, branch, cdata);
    end = clock();
//    ROS_INFO("check collision between left_gripper rightfinger & right_gripper rightfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("rightfinger");
        collisionresult.pairB.push_back("rightfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper rightfinger & right_gripper rightfinger.");
    }
    std::cout << "dis: " << dis << " m \n";
    std::cout << "The number of pairs of spheres has been detected: " << cdata.tra_num << "\n";
    ROS_INFO("detection running time: %lf s.\n", double(end-start) / CLOCKS_PER_SEC);
    if (only_time)
    {
        out << double(end-start) / CLOCKS_PER_SEC << "\n";
        out2 << cdata.tra_num << "\n";
    }
    else
        out << "[" << double(end-start) / CLOCKS_PER_SEC << "," << cdata.tra_num << "," << (double(end-start) / CLOCKS_PER_SEC)/cdata.tra_num << "]\n";

    out.close();
    out2.close();
}

int main(int argc, char **argv)
{
//    time_t t=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
//    std::stringstream ss;
//    ss << std::put_time(std::localtime(&t),"%F %X");

    //Initiate ROS
    ros::init(argc, argv, "listenandplot");
    ros::NodeHandle n;

    /// rosparam load
//    ros::param::set("/source_list", "[/move_group/fake_controller_joint_states]");

    init_();

    ros::Publisher dis_pub = n.advertise<std_msgs::Float64>("/plot/Ref", 1);
    ros::Publisher TMdis_poly_pub = n.advertise<std_msgs::Float64>("/plot/TM_poly", 1);
    ros::Publisher TMdis_upper_pub = n.advertise<std_msgs::Float64>("/plot/TM_upper", 1);
    ros::Publisher TMdis_lower_pub = n.advertise<std_msgs::Float64>("/plot/TM_lower", 1);
    ros::Publisher diff_pub = n.advertise<std_msgs::Float64>("/plot/Diff", 1);
    ros::Publisher yumi_joint_state_publish = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    ros::Rate loop_rate(3000);
    ros::spinOnce();

    std_msgs::Float64 dis, TMpoly, TMupper, TMlower, diff;
    dis.data = 0.0;

    /// init the joint_state which will be published later.
    yumi_joint_state.header.stamp = ros::Time::now();
    for (int i = 0; i < ActionGoal[0].goal.request.start_state.joint_state.position.size(); i++) {
        yumi_joint_state.name.push_back(ActionGoal[0].goal.request.start_state.joint_state.name[i].data());
        yumi_joint_state.position.push_back(ActionGoal[0].goal.request.start_state.joint_state.position[i]);
    }

    cur_time = 0;
    flag = true;

    std::string outfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + MOTION_NAME + "_depth" + std::to_string(depth-1) + "branch" + std::to_string(branch) + ".txt";
    std::ofstream out(outfile);

    while (ros::ok()) {
        if (flag) {
//            ROS_INFO("publish joint_state of motion during time interval [0, %lf].", interval);

            /// [t0, t1)
            if (cur_time < interval) {
//                ROS_INFO("current time: %lf", cur_time);
//                ROS_INFO("[%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf]",
//                         yumi_joint_state.position[0], yumi_joint_state.position[1], yumi_joint_state.position[2],
//                         yumi_joint_state.position[3], yumi_joint_state.position[4], yumi_joint_state.position[5],
//                         yumi_joint_state.position[6], yumi_joint_state.position[7], yumi_joint_state.position[8],
//                         yumi_joint_state.position[9], yumi_joint_state.position[10], yumi_joint_state.position[11],
//                         yumi_joint_state.position[12], yumi_joint_state.position[13], yumi_joint_state.position[14],
//                         yumi_joint_state.position[15], yumi_joint_state.position[16], yumi_joint_state.position[17]);

                /// TODO, from urdf2casadi/txt or get from the tf
                /// calculate the real distance from the urdf and joint_state
//                if (cur_time == 0)
//                    WRITE_REAL = true;
//                else
//                    WRITE_REAL = false;
                clock_t start_ = clock();
                dis.data = realdis(yumi_joint_state, 7);
//                ROS_INFO("real distance: %lf", dis.data);
                clock_t end_ = clock();

                /// calculate the TM distance
                cal_TM_distance(TMpoly, TMupper, TMlower);
//                ROS_INFO("TM poly: %lf", TMpoly);
//                ROS_INFO("TM distance bound: [%lf, %lf]", TMlower, TMupper);

                diff.data = dis.data - TMlower.data;

                if (dis.data > TMupper.data || dis.data < TMlower.data) {
                    ROS_INFO("time at %lf in action %d, the data is wrong.", cur_time, cur_Action);
                }

                dis_pub.publish(dis);
                TMdis_poly_pub.publish(TMpoly);
                TMdis_lower_pub.publish(TMlower);
                TMdis_upper_pub.publish(TMupper);
                diff_pub.publish(diff);
                out << cur_time << " " << dis.data << " " << TMpoly.data << " " << TMlower.data << " " << TMupper.data << " " << dis.data - TMlower.data << "\n";

                for (int k = 0; k < yumi_joint_state.position.size(); k++) {
                    yumi_joint_state.position[k] += sub_motion[cur_Action][cur_Motion].velocity[k] * step;
//                    ROS_INFO("%s: position: %lf", yumi_joint_state.name[k].data(), yumi_joint_state.position[k]);
                }
                cur_time += step;
            } else {
                if (cur_Action < ActionGoal.size() - 1) {
                    ROS_INFO("The Action %d is over.", cur_Action+1);
                    cur_time = 0;
                    flag = false;
                    cur_Action++;
                    yumi_joint_state.velocity.clear();
                    yumi_joint_state.position.clear();
//                    yumi_joint_state.name.clear();
                    yumi_joint_state.header.stamp = ros::Time::now();
                    for (int i = 0; i < ActionGoal[cur_Action].goal.request.start_state.joint_state.position.size(); i++) {
//                        yumi_joint_state.name.push_back(ActionGoal[cur_Action].goal.request.start_state.joint_state.name[i].data());
                        yumi_joint_state.position.push_back(ActionGoal[cur_Action].goal.request.start_state.joint_state.position[i]);
                    }
                    flag = true;
                }
                else {
                    out.close();
                }
            }

            yumi_joint_state_publish.publish(yumi_joint_state);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


