#include "fcl/tm/Taylor_Function.h"
#include <fstream>
#include <cassert>
#include <iostream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include "fcl/tm/genetic_algorithm.h"
#include "fcl/math/vec_3f.h"

std::string file  = "/home/monica/source_code/panda/scripts/raw_data_panda.txt";
//std::string OBJ_RESOURCES_DIR = "/opt/ros/kinetic/share/franka_description/meshes/obj";
std::string OBJ_RESOURCES_DIR = "/home/monica/moveit_ws/src/yumi_lerp_planner/yumi_description/meshes/obj";
std::string file_l = "/home/monica/moveit_ws/src/yumi_lerp_planner/scripts/raw_data_l.txt";
std::string file_r = "/home/monica/moveit_ws/src/yumi_lerp_planner/scripts/raw_data_r.txt";


void ReadUrdfFile()
{
    int read_ptr = 0;
    std::ifstream infile;
    infile.open(file.data());
    assert(infile.is_open());

    std::string s;
    double ROS_P[3], ROS_Q[4];
    while (getline(infile, s))
    {
        if (s.find("Transform") == 0) {
            for (int line = 0; line < 2; line++) {
                getline(infile, s);

                if (s.find("position") == 0) {
                    int start = s.find('[');
                    int end = s.find(']');
                    std::string array = s.substr(start, end);
//                    std::cout << array << std::endl;

                    std::string x = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string y = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string z = array.substr(1, array.size() - 2);

                    ROS_P[0] = atof(x.c_str());
                    ROS_P[1] = atof(y.c_str());
                    ROS_P[2] = atof(z.c_str());

                    frame[0][read_ptr].xyz[0] = ROS_P[0];
                    frame[0][read_ptr].xyz[1] = ROS_P[1];
                    frame[0][read_ptr].xyz[2] = ROS_P[2];

                    // single-arm frame[0] same as frame[1]
                    frame[1][read_ptr].xyz[0] = ROS_P[0];
                    frame[1][read_ptr].xyz[1] = ROS_P[1];
                    frame[1][read_ptr].xyz[2] = ROS_P[2];

//                    std::cout << "ROS_P : " << ROS_P[0] << '\t' << ROS_P[1] << '\t' << ROS_P[2] << std::endl;
                }
                else if (s.find("quaternion") == 0) {

                    int start = s.find('[');
                    int end = s.find(']');
                    std::string array = s.substr(start, end);
//                    std::cout << array << std::endl;

                    std::string x = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string y = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string z = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string w = array.substr(1, array.size() - 2);

                    ROS_Q[0] = atof(x.c_str());
                    ROS_Q[1] = atof(y.c_str());
                    ROS_Q[2] = atof(z.c_str());
                    ROS_Q[3] = atof(w.c_str());

                    frame[0][read_ptr].quaternion[0] = ROS_Q[0];
                    frame[0][read_ptr].quaternion[1] = ROS_Q[1];
                    frame[0][read_ptr].quaternion[2] = ROS_Q[2];
                    frame[0][read_ptr].quaternion[3] = ROS_Q[3];

                    // single-arm frame[0] same as frame[1]
                    frame[1][read_ptr].quaternion[0] = ROS_Q[0];
                    frame[1][read_ptr].quaternion[1] = ROS_Q[1];
                    frame[1][read_ptr].quaternion[2] = ROS_Q[2];
                    frame[1][read_ptr].quaternion[3] = ROS_Q[3];

//                    std::cout << "quaternion : " << ROS_Q[0] << '\t' << ROS_Q[1] << '\t' << ROS_Q[2] << '\t' << ROS_Q[3] << std::endl;
                }
            }
            
            read_ptr++;

        }

    }
//    std::cout<<"read file end!\n"<<"\n";
    infile.close();
}

void ReadUrdfFile(std::string filename, int i)
{
    int read_ptr = 0;
    std::ifstream infile;
    infile.open(filename.data());
    assert(infile.is_open());

    std::string s;
    double ROS_P[3], ROS_Q[4];
    while (getline(infile, s))
    {
        if (s.find("Transform") == 0) {
            for (int line = 0; line < 2; line++) {
                getline(infile, s);

                if (s.find("position") == 0) {
                    int start = s.find('[');
                    int end = s.find(']');
                    std::string array = s.substr(start, end);
//                    std::cout << array << std::endl;

                    std::string x = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string y = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string z = array.substr(1, array.size() - 2);

                    ROS_P[0] = atof(x.c_str());
                    ROS_P[1] = atof(y.c_str());
                    ROS_P[2] = atof(z.c_str());

                    frame[i][read_ptr].xyz[0] = ROS_P[0];
                    frame[i][read_ptr].xyz[1] = ROS_P[1];
                    frame[i][read_ptr].xyz[2] = ROS_P[2];

//                    std::cout << "ROS_P : " << ROS_P[0] << '\t' << ROS_P[1] << '\t' << ROS_P[2] << std::endl;
                }
                else if (s.find("quaternion") == 0) {

                    int start = s.find('[');
                    int end = s.find(']');
                    std::string array = s.substr(start, end);
//                    std::cout << array << std::endl;

                    std::string x = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string y = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string z = array.substr(1, array.find_first_of(',') - 1);
                    array = array.substr(array.find_first_of(',') + 1, array.size());

                    std::string w = array.substr(1, array.size() - 2);

                    ROS_Q[0] = atof(x.c_str());
                    ROS_Q[1] = atof(y.c_str());
                    ROS_Q[2] = atof(z.c_str());
                    ROS_Q[3] = atof(w.c_str());

                    frame[i][read_ptr].quaternion[0] = ROS_Q[0];
                    frame[i][read_ptr].quaternion[1] = ROS_Q[1];
                    frame[i][read_ptr].quaternion[2] = ROS_Q[2];
                    frame[i][read_ptr].quaternion[3] = ROS_Q[3];

//                    std::cout << "quaternion : " << ROS_Q[0] << '\t' << ROS_Q[1] << '\t' << ROS_Q[2] << '\t' << ROS_Q[3] << std::endl;
                }
            }

            read_ptr++;

        }

    }
//    std::cout<<"read file end!\n"<<"\n";
    infile.close();
}

void init_frame_hm() {
    // frame_para frame[2][10];
    // cTM3Matrix33 Rinit[2][10];
    // cTM3Matrix33 Ryaw[2][10];
    // cTM3Matrix44 HM[2][10];
    // cTM3Matrix44 HMhand[2];
    // cTM3Matrix44 HMleftfinger[2];
    // cTM3Matrix44 HMrightfinger[2];
    // cTM3Matrix44 HMleftyaw[2];
    // cTM3Matrix44 HMrightyaw[2];

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 10; j++) {
            frame[i][j].init();
            Rinit[i][j].setZero();
            Ryaw[i][j].setZero();
            HM[i][j].init();
        }
    }
    for (int i = 0; i < 2; i++) {
         HMhand[i].init();
         HMleftfinger[i].init();
         HMrightfinger[i].init();
         HMleftyaw[i].init();
         HMrightyaw[i].init();
    }
}

void init_yaw() {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 10; j++) {
            frame[i][j].yaw0 = 0;
            frame[i][j].yaw1 = 0;
            frame[i][j].yaw_omega = 0;
        }
    }
}

void init_yaw(double t) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 10; j++) {
            frame[i][j].yaw0 = 0;
            frame[i][j].yaw1 = 0;
            frame[i][j].yaw_omega = (frame[i][j].yaw1 - frame[i][j].yaw0) / t;
        }
    }
}

void setupyaw(double t, const fcl::LerpMotion* m1, const fcl::LerpMotion* m2, int &num)
{
    // for link
    std::vector<float> m1_joint1;
    std::vector<float> m1_joint2;
    std::vector<float> m1_vel;
    m1->getJointState(m1_joint1, m1_joint2);
    m1->getJointVel(m1_vel);

    for(unsigned int i = 0; i < m1_joint1.size(); i++)
    {
        frame[0][i].yaw0 = m1_joint1[i];
        frame[0][i].yaw1 = m1_joint2[i];
        frame[0][i].yaw_omega = m1_vel[i];
    }

    std::vector<float> m2_joint1;
    std::vector<float> m2_joint2;
    std::vector<float> m2_vel;
    m2->getJointState(m2_joint1, m2_joint2);
    m2->getJointVel(m2_vel);

    for(unsigned int i = 0; i < m2_joint1.size(); i++)
    {
        frame[1][i].yaw0 = m2_joint1[i];
        frame[1][i].yaw1 = m2_joint2[i];
        frame[1][i].yaw_omega = m2_vel[i];
    }

    // for hand & finger
    // Modbyme
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);
    cTaylorModel3 delta_left, delta_right;
    // robot0
    delta_left.setZero();
    delta_right.setZero();
    delta_left.c[0] = m1_joint1[num];
    delta_left.c[1] = m1_vel[num];
    delta_right.c[0] = m1_joint1[num];
    delta_right.c[1] = m1_vel[num];
    cTM3Matrix44 H0left(one,zero,zero,zero,zero,one,zero,delta_left,zero,zero,one,zero,zero,zero,zero,one);
    cTM3Matrix44 H0right(one,zero,zero,zero,zero,one,zero,delta_right,zero,zero,one,zero,zero,zero,zero,one);
    HMleftyaw[0] = H0left;
    HMrightyaw[0] = H0right;
    // robot1
    delta_left.setZero();
    delta_right.setZero();
    delta_left.c[0] = m2_joint1[num];
    delta_left.c[1] = m2_vel[num];
    delta_right.c[0] = m2_joint1[num];
    delta_right.c[1] = m2_vel[num];
    cTM3Matrix44 H1left(one,zero,zero,zero,zero,one,zero,delta_left,zero,zero,one,zero,zero,zero,zero,one);
    cTM3Matrix44 H1right(one,zero,zero,zero,zero,one,zero,delta_right,zero,zero,one,zero,zero,zero,zero,one);
    HMleftyaw[1] = H1left;
    HMrightyaw[1] = H1right;
//    num = m1_vel.size();
}

void setupyaw(const fcl::LerpMotion* m1, const fcl::LerpMotion* m2, int &num)
{
    // dual-arm
    // for link
    std::vector<float> m1_joint1;
    std::vector<float> m1_joint2;
    std::vector<float> m1_vel;
    m1->getJointState(m1_joint1, m1_joint2);
    m1->getJointVel(m1_vel);

    for(unsigned int i = 0; i < m1_joint1.size(); i++)
    {
        frame[0][i].yaw0 = m1_joint1[i];
        frame[0][i].yaw1 = m1_joint2[i];
        frame[0][i].yaw_omega = m1_vel[i];
    }

    std::vector<float> m2_joint1;
    std::vector<float> m2_joint2;
    std::vector<float> m2_vel;
    m2->getJointState(m2_joint1, m2_joint2);
    m2->getJointVel(m2_vel);

    for(unsigned int i = 0; i < m2_joint1.size(); i++)
    {
        frame[1][i].yaw0 = m2_joint1[i];
        frame[1][i].yaw1 = m2_joint2[i];
        frame[1][i].yaw_omega = m2_vel[i];
    }

    // for gripper
    // assume the gripper is static
}

void setupHM(int num)
{
    // for Link
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < num; j++)
        {
            // Rinit
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);
            cMatrix33 r1;
            Rinit[i][j] = MEigen2cTM(R1);

            // Ryaw
            Ryaw[i][j] = Myaw(frame[i][j].yaw_omega, frame[i][j].yaw0);

            // HM
            cTM3Matrix33 R = Rinit[i][j] * Ryaw[i][j];
            cTM3Matrix44 temp;
            cTaylorModel3 x(frame[i][j].xyz[0]);
            cTaylorModel3 y(frame[i][j].xyz[1]);
            cTaylorModel3 z(frame[i][j].xyz[2]);

            temp.M[0][0] = R.i[0][0];
            temp.M[0][1] = R.i[0][1];
            temp.M[0][2] = R.i[0][2];
            temp.M[0][3] = x;
            temp.M[1][0] = R.i[1][0];
            temp.M[1][1] = R.i[1][1];
            temp.M[1][2] = R.i[1][2];
            temp.M[1][3] = y;
            temp.M[2][0] = R.i[2][0];
            temp.M[2][1] = R.i[2][1];
            temp.M[2][2] = R.i[2][2];
            temp.M[2][3] = z;
            temp.M[3][0] = zero;
            temp.M[3][1] = zero;
            temp.M[3][2] = zero;
            temp.M[3][3] = one;

            HM[i][j] = temp;
        }
    }
    // for Hand
    setupHMhand(num);
    // for finger
    setupHMfinger(num);
}

void setupdualHM(int num)
{
    // for world to body
    Eigen::Matrix3d  R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.1};
    cTM3Matrix44 HMtmp(r, xyz);
    HMbody = HMtmp;

    // for Link
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < num; j++)
        {
            // Rinit
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);
            cMatrix33 r1;
            Rinit[i][j] = MEigen2cTM(R1);

            // Ryaw
            Ryaw[i][j] = Myaw(frame[i][j].yaw_omega, frame[i][j].yaw0);

            // HM
            cTM3Matrix33 R = Rinit[i][j] * Ryaw[i][j];
            cTM3Matrix44 temp;
            cTaylorModel3 x(frame[i][j].xyz[0]);
            cTaylorModel3 y(frame[i][j].xyz[1]);
            cTaylorModel3 z(frame[i][j].xyz[2]);

            temp.M[0][0] = R.i[0][0];
            temp.M[0][1] = R.i[0][1];
            temp.M[0][2] = R.i[0][2];
            temp.M[0][3] = x;
            temp.M[1][0] = R.i[1][0];
            temp.M[1][1] = R.i[1][1];
            temp.M[1][2] = R.i[1][2];
            temp.M[1][3] = y;
            temp.M[2][0] = R.i[2][0];
            temp.M[2][1] = R.i[2][1];
            temp.M[2][2] = R.i[2][2];
            temp.M[2][3] = z;
            temp.M[3][0] = zero;
            temp.M[3][1] = zero;
            temp.M[3][2] = zero;
            temp.M[3][3] = one;

            if (i == 0) {
                HM[i][j] = temp;
            }
            else {
                HM[i][j] = HM[i][j-1] * temp;
            }
        }
    }

    // assume, the gripper is fixed
    // for gripper
    // yumi_link_7 to gripper_base
    Eigen::Matrix3d  R1 = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 r1 = MEigen2cM(R1);
    double xyz1[3] = {0,0,0.007};
    cTM3Matrix44 HMgripperbase(r1, xyz1);
    HMhand[0] = HM[0][num - 1] * HMgripperbase;
    HMhand[1] = HM[1][num - 1] * HMgripperbase;


    // for finger
    Eigen::Matrix3d Rl = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 rl = MEigen2cM(Rl);
    double xyzl[3] = {0.0, 0.0065, 0.0837};
    cTM3Matrix44 leftfinger(rl, xyzl);
    Eigen::Matrix3d Rr = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 rr = MEigen2cM(Rr);
    double xyzr[3] = {0.0, -0.0065, 0.0837};
    cTM3Matrix44 rightfinger(rr, xyzr);
    HMleftfinger[0] = HMhand[0] * leftfinger;
    HMrightfinger[0] = HMhand[0] * rightfinger;
    HMleftfinger[1] = HMhand[1] * leftfinger;
    HMrightfinger[1] = HMhand[1] * rightfinger;
}

void setupdualHM(int num, cInterval& interval)
{
    setGlobalTimeInterval(interval.i[0], interval.i[1]);
    // for world to body
    Eigen::Matrix3d  R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.1};
    cTM3Matrix44 HMtmp(r, xyz);
    HMbody = HMtmp;

    // for Link
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < num; j++)
        {
            // Rinit
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);
            cMatrix33 r1;
            Rinit[i][j] = MEigen2cTM(R1);

            // Ryaw
            Ryaw[i][j] = Myaw(frame[i][j].yaw_omega, frame[i][j].yaw0);

            // HM
            cTM3Matrix33 R = Rinit[i][j] * Ryaw[i][j];
            cTM3Matrix44 temp;
            cTaylorModel3 x(frame[i][j].xyz[0]);
            cTaylorModel3 y(frame[i][j].xyz[1]);
            cTaylorModel3 z(frame[i][j].xyz[2]);

            temp.M[0][0] = R.i[0][0];
            temp.M[0][1] = R.i[0][1];
            temp.M[0][2] = R.i[0][2];
            temp.M[0][3] = x;
            temp.M[1][0] = R.i[1][0];
            temp.M[1][1] = R.i[1][1];
            temp.M[1][2] = R.i[1][2];
            temp.M[1][3] = y;
            temp.M[2][0] = R.i[2][0];
            temp.M[2][1] = R.i[2][1];
            temp.M[2][2] = R.i[2][2];
            temp.M[2][3] = z;
            temp.M[3][0] = zero;
            temp.M[3][1] = zero;
            temp.M[3][2] = zero;
            temp.M[3][3] = one;

            if (j == 0) {
                HM[i][j] = HMbody * temp;
            }
            else {
                HM[i][j] = HM[i][j-1] * temp;
            }
        }
    }

    // assume, the gripper is fixed
    // for gripper
    // yumi_link_7 to gripper_base
    Eigen::Matrix3d  R1 = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 r1 = MEigen2cM(R1);
    double xyz1[3] = {0,0,0.007};
    cTM3Matrix44 HMgripperbase(r1, xyz1);
    HMhand[0] = HM[0][num - 1] * HMgripperbase;
    HMhand[1] = HM[1][num - 1] * HMgripperbase;


    // for finger
    Eigen::Matrix3d Rl = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 rl = MEigen2cM(Rl);
    double xyzl[3] = {0.0, 0.0065, 0.0837};
    cTM3Matrix44 leftfinger(rl, xyzl);
    Eigen::Matrix3d Rr = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 rr = MEigen2cM(Rr);
    double xyzr[3] = {0.0, -0.0065, 0.0837};
    cTM3Matrix44 rightfinger(rr, xyzr);
    HMleftfinger[0] = HMhand[0] * leftfinger;
    HMrightfinger[0] = HMhand[0] * rightfinger;
    HMleftfinger[1] = HMhand[1] * leftfinger;
    HMrightfinger[1] = HMhand[1] * rightfinger;
}

void setupdualHM(int num, cInterval& interval, motion& sub_motion)
{
    setGlobalTimeInterval(interval.i[0], interval.i[1]);
    // for world to body
    Eigen::Matrix3d  R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.1};
    cTM3Matrix44 HMtmp(r, xyz);
    HMbody = HMtmp;

    // for Link
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < num; j++)
        {
            // Rinit
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);
            cMatrix33 r1;
            Rinit[i][j] = MEigen2cTM(R1);

            // Ryaw
//            Ryaw[i][j] = Myaw(frame[i][j].yaw_omega, frame[i][j].yaw0);
            int joint_num = sub_motion.start_state.size() / 2;
            if (i == 0) {
                // left_arm
                Ryaw[i][j] = Myaw(sub_motion.velocity[j], sub_motion.start_state[j]);
            }
            else {
                // right_arm
                Ryaw[i][j] = Myaw(sub_motion.velocity[j + joint_num], sub_motion.start_state[j + joint_num]);
            }

            // HM
            cTM3Matrix33 R = Rinit[i][j] * Ryaw[i][j];
            cTM3Matrix44 temp;
            cTaylorModel3 x(frame[i][j].xyz[0]);
            cTaylorModel3 y(frame[i][j].xyz[1]);
            cTaylorModel3 z(frame[i][j].xyz[2]);

            temp.M[0][0] = R.i[0][0];
            temp.M[0][1] = R.i[0][1];
            temp.M[0][2] = R.i[0][2];
            temp.M[0][3] = x;
            temp.M[1][0] = R.i[1][0];
            temp.M[1][1] = R.i[1][1];
            temp.M[1][2] = R.i[1][2];
            temp.M[1][3] = y;
            temp.M[2][0] = R.i[2][0];
            temp.M[2][1] = R.i[2][1];
            temp.M[2][2] = R.i[2][2];
            temp.M[2][3] = z;
            temp.M[3][0] = zero;
            temp.M[3][1] = zero;
            temp.M[3][2] = zero;
            temp.M[3][3] = one;

            if (j == 0) {
                HM[i][j] = HMbody * temp;
            }
            else {
                HM[i][j] = HM[i][j-1] * temp;
            }
        }
    }

    // assume, the gripper is fixed
    // for gripper
    // yumi_link_7 to gripper_base
    Eigen::Matrix3d  R1 = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 r1 = MEigen2cM(R1);
    double xyz1[3] = {0,0,0.007};
    cTM3Matrix44 HMgripperbase(r1, xyz1);
    HMhand[0] = HM[0][num - 1] * HMgripperbase;
    HMhand[1] = HM[1][num - 1] * HMgripperbase;


    // for finger
    Eigen::Matrix3d Rl = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 rl = MEigen2cM(Rl);
    double xyzl[3] = {0.0, 0.0065, 0.0837};
    cTM3Matrix44 leftfinger(rl, xyzl);
    Eigen::Matrix3d Rr = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 rr = MEigen2cM(Rr);
    double xyzr[3] = {0.0, -0.0065, 0.0837};
    cTM3Matrix44 rightfinger(rr, xyzr);
    HMleftfinger[0] = HMhand[0] * leftfinger;
    HMrightfinger[0] = HMhand[0] * rightfinger;
    HMleftfinger[1] = HMhand[1] * leftfinger;
    HMrightfinger[1] = HMhand[1] * rightfinger;
}

void setupdualHM_movable(int num, cInterval& interval, motion& sub_motion)
{
    setGlobalTimeInterval(interval.i[0], interval.i[1]);
    // for world to body
    Eigen::Matrix3d R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.1};
    cTM3Matrix44 HMtmp(r, xyz);
    HMbody = HMtmp;

    // for Link
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < num; j++)
        {
            // Rinit
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);
            cMatrix33 r1;
            Rinit[i][j] = MEigen2cTM(R1);

            // Ryaw
//            Ryaw[i][j] = Myaw(frame[i][j].yaw_omega, frame[i][j].yaw0);
            int joint_num = sub_motion.start_state.size() / 2;
            if (i == 0) {
                // left_arm
                Ryaw[i][j] = Myaw(sub_motion.velocity[j], sub_motion.start_state[j]);
            }
            else {
                // right_arm
                Ryaw[i][j] = Myaw(sub_motion.velocity[j + joint_num], sub_motion.start_state[j + joint_num]);
            }

            // HM
            cTM3Matrix33 R = Rinit[i][j] * Ryaw[i][j];
            cTM3Matrix44 temp;
            cTaylorModel3 x(frame[i][j].xyz[0]);
            cTaylorModel3 y(frame[i][j].xyz[1]);
            cTaylorModel3 z(frame[i][j].xyz[2]);

            temp.M[0][0] = R.i[0][0];
            temp.M[0][1] = R.i[0][1];
            temp.M[0][2] = R.i[0][2];
            temp.M[0][3] = x;
            temp.M[1][0] = R.i[1][0];
            temp.M[1][1] = R.i[1][1];
            temp.M[1][2] = R.i[1][2];
            temp.M[1][3] = y;
            temp.M[2][0] = R.i[2][0];
            temp.M[2][1] = R.i[2][1];
            temp.M[2][2] = R.i[2][2];
            temp.M[2][3] = z;
            temp.M[3][0] = zero;
            temp.M[3][1] = zero;
            temp.M[3][2] = zero;
            temp.M[3][3] = one;

            if (j == 0) {
                HM[i][j] = HMbody * temp;
            }
            else {
                HM[i][j] = HM[i][j-1] * temp;
            }
        }
    }

    // assume, the gripper is fixed
    // for gripper
    // yumi_link_7 to gripper_base
    Eigen::Matrix3d  R1 = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 r1 = MEigen2cM(R1);
    double xyz1[3] = {0,0,0.007};
    cTM3Matrix44 HMgripperbase(r1, xyz1);
    HMhand[0] = HM[0][num - 1] * HMgripperbase;
    HMhand[1] = HM[1][num - 1] * HMgripperbase;

    // for finger
    Eigen::Matrix3d Rl = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 rl = MEigen2cM(Rl);
    double xyzl[3] = {0.0, 0.0065, 0.0837};
    cTM3Matrix44 leftfinger(rl, xyzl);
    Eigen::Matrix3d Rr = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 rr = MEigen2cM(Rr);
    double xyzr[3] = {0.0, -0.0065, 0.0837};
    cTM3Matrix44 rightfinger(rr, xyzr);

    // for movable joint
    // yumi: gripper_joint: <axis xyz="-1,0,0">
    cTaylorModel3 dll, dlr, drl, drr;
    dll.setZero();
    dlr.setZero();
    drl.setZero();
    drr.setZero();
    dll.c[0] = -1 * sub_motion.start_state[num+1];
    dll.c[1] = -1 * sub_motion.velocity[num+1];
    dlr.c[0] = -1 * sub_motion.start_state[num];
    dlr.c[1] = -1 * sub_motion.velocity[num];
    drl.c[0] = -1 * sub_motion.start_state[num*2+3];
    drl.c[1] = -1 * sub_motion.velocity[num*2+3];
    drr.c[0] = -1 * sub_motion.start_state[num*2+2];
    drr.c[1] = -1 * sub_motion.velocity[num*2+2];
    cTM3Matrix44 finger_ll(one, zero, zero, dll,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM3Matrix44 finger_lr(one, zero, zero, dlr,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM3Matrix44 finger_rl(one, zero, zero, drl,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM3Matrix44 finger_rr(one, zero, zero, drr,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    
    HMleftfinger[0] = HMhand[0] * leftfinger * finger_ll;
    HMrightfinger[0] = HMhand[0] * rightfinger * finger_lr;
    HMleftfinger[1] = HMhand[1] * leftfinger * finger_rl;
    HMrightfinger[1] = HMhand[1] * rightfinger * finger_rr;
}

void setupdualHM_movable_uncertain(int num, cInterval& interval, motion& sub_motion, std::vector<double> &limit_l, std::vector<double> &limit_u)
{
    setGlobalTimeInterval(interval.i[0], interval.i[1]);
    // for world to body
    Eigen::Matrix3d R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.1};
    cTM3Matrix44 HMtmp(r, xyz);
    HMbody = HMtmp;

    // for Link
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < num; j++)
        {
            // Rinit
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);
            cMatrix33 r1;
            Rinit[i][j] = MEigen2cTM(R1);

            // Ryaw
//            Ryaw[i][j] = Myaw(frame[i][j].yaw_omega, frame[i][j].yaw0);
            int joint_num = sub_motion.start_state.size() / 2;
            if (i == 0) {
                // left_arm
                Ryaw[i][j] = Myaw_uncertain(sub_motion.velocity[j], sub_motion.start_state[j], limit_l[i], limit_u[i]);
            }
            else {
                // right_arm
                Ryaw[i][j] = Myaw_uncertain(sub_motion.velocity[j + joint_num], sub_motion.start_state[j + joint_num], limit_l[i], limit_u[i]);
            }

            // HM
            cTM3Matrix33 R = Rinit[i][j] * Ryaw[i][j];
            cTM3Matrix44 temp;
            cTaylorModel3 x(frame[i][j].xyz[0]);
            cTaylorModel3 y(frame[i][j].xyz[1]);
            cTaylorModel3 z(frame[i][j].xyz[2]);

            temp.M[0][0] = R.i[0][0];
            temp.M[0][1] = R.i[0][1];
            temp.M[0][2] = R.i[0][2];
            temp.M[0][3] = x;
            temp.M[1][0] = R.i[1][0];
            temp.M[1][1] = R.i[1][1];
            temp.M[1][2] = R.i[1][2];
            temp.M[1][3] = y;
            temp.M[2][0] = R.i[2][0];
            temp.M[2][1] = R.i[2][1];
            temp.M[2][2] = R.i[2][2];
            temp.M[2][3] = z;
            temp.M[3][0] = zero;
            temp.M[3][1] = zero;
            temp.M[3][2] = zero;
            temp.M[3][3] = one;

            if (j == 0) {
                HM[i][j] = HMbody * temp;
            }
            else {
                HM[i][j] = HM[i][j-1] * temp;
            }
        }
    }

    // assume, the gripper is fixed
    // for gripper
    // yumi_link_7 to gripper_base
    Eigen::Matrix3d  R1 = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 r1 = MEigen2cM(R1);
    double xyz1[3] = {0,0,0.007};
    cTM3Matrix44 HMgripperbase(r1, xyz1);
    HMhand[0] = HM[0][num - 1] * HMgripperbase;
    HMhand[1] = HM[1][num - 1] * HMgripperbase;

    // for finger
    Eigen::Matrix3d Rl = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 rl = MEigen2cM(Rl);
    double xyzl[3] = {0.0, 0.0065, 0.0837};
    cTM3Matrix44 leftfinger(rl, xyzl);
    Eigen::Matrix3d Rr = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 rr = MEigen2cM(Rr);
    double xyzr[3] = {0.0, -0.0065, 0.0837};
    cTM3Matrix44 rightfinger(rr, xyzr);

    // for movable joint
    // yumi: gripper_joint: <axis xyz="-1,0,0">
    cTaylorModel3 dll, dlr, drl, drr;
    dll.setZero();
    dlr.setZero();
    drl.setZero();
    drr.setZero();
    dll.c[0] = -1 * sub_motion.start_state[num+1];
    dll.c[1] = -1 * sub_motion.velocity[num+1];
    dlr.c[0] = -1 * sub_motion.start_state[num];
    dlr.c[1] = -1 * sub_motion.velocity[num];
    drl.c[0] = -1 * sub_motion.start_state[num*2+3];
    drl.c[1] = -1 * sub_motion.velocity[num*2+3];
    drr.c[0] = -1 * sub_motion.start_state[num*2+2];
    drr.c[1] = -1 * sub_motion.velocity[num*2+2];
    cTM3Matrix44 finger_ll(one, zero, zero, dll,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM3Matrix44 finger_lr(one, zero, zero, dlr,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM3Matrix44 finger_rl(one, zero, zero, drl,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM3Matrix44 finger_rr(one, zero, zero, drr,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);

    HMleftfinger[0] = HMhand[0] * leftfinger * finger_ll;
    HMrightfinger[0] = HMhand[0] * rightfinger * finger_lr;
    HMleftfinger[1] = HMhand[1] * leftfinger * finger_rl;
    HMrightfinger[1] = HMhand[1] * rightfinger * finger_rr;
}

void setupHMhand(int num)
{
    // hand relatively fixed
    // panda_link7 to panda_link8
    Eigen::Matrix3d  R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.107};
    cTM3Matrix44 HM8(r, xyz);
    // panda_link8 to panda_hand
    double c = cos(-CTM3_PI/4);
    double s = sin(-CTM3_PI/4);
    cMatrix33 r1(c, -s, 0, s, c, 0, 0, 0, 1);
    double xyz1[3] = {0.0, 0.0, 0.0};
    cTM3Matrix44 HM82hand(r1, xyz1);
    HMhand[0] = HM[0][0];
    HMhand[1] = HM[1][0];
    for(int i = 1; i < num; i++)
    {
        HMhand[0] = HMhand[0] * HM[0][i];
        HMhand[1] = HMhand[1] * HM[1][i];
    }
    HMhand[0] = HMhand[0] * HM8 * HM82hand;
    HMhand[1] = HMhand[1] * HM8 * HM82hand;
}

void setupHMfinger(int num)
{
    Eigen::Matrix3d  R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.0584};
    cTM3Matrix44 lefthandstatic(r, xyz);
    // rotate the right finger 180° around the z axis
    // basis rotation matrix = [cosx, -sinx, 0; sinx, cosx, 0; 0, 0, 1]
    // r = [-1, 0, 0; 0 ,-1, 0; 0, 0, 1]
    cMatrix33 r1(-1, 0, 0, 0 ,-1, 0, 0, 0, 1);
    cMatrix33 rr = r*r1;
    cTM3Matrix44 righthandstatic(rr, xyz);
    HMleftfinger[0] = HMhand[0] * lefthandstatic * HMleftyaw[0];
    HMrightfinger[0] = HMhand[0] * righthandstatic * HMrightyaw[0];
    HMleftfinger[1] = HMhand[1] * lefthandstatic * HMleftyaw[1];
    HMrightfinger[1] = HMhand[1] * righthandstatic * HMrightyaw[1];
}

void showHM(int num)
{
//    for(int i = 0; i < 2; i++)
//    {
//        for(int j = 0; j < num; j++)
//        {
//            std::cout<<"HM["<<i<<"]["<<j<<"]:\n";
//            for(int m = 0; m < 4; m++){
//                for(int n = 0; n < 4; n++)
//                {
//                    HM[i][j].M[m][n].print();
//                }
//            }
//        }
//    }
    cTMVector4 p(0.0, 0.0, 0.0, 1.0);  // [x,y,z,1]
    cTMVector4 p1(0.0, 0.04, 0.0, 1.0);
    cTMVector4 pp;
    // link 1
    cTM3Matrix44 temp = HMbody * HM[0][0];
    pp = M44multV14(temp, p);
    std::cout << "link1 " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // link 2
    temp = temp * HM[0][1];
    pp = M44multV14(temp, p);
    std::cout << "link2 " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // link 3
    temp = temp * HM[0][2];
    pp = M44multV14(temp, p);
    std::cout << "link3 " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // link 4
    temp = temp * HM[0][3];
    pp = M44multV14(temp, p);
    std::cout << "link4 " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // link 5
    temp = temp * HM[0][4];
//    std::cout<<"HMlink5:\n";
//    for(int m = 0; m < 4; m++){
//        for(int n = 0; n < 4; n++)
//        {
//            std::cout << "HMlink5[" << m << ", " << n << "]\n";
//            temp.M[m][n].print();
//        }
//    }
    pp = M44multV14(temp, p);
    std::cout << "link5 " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // link 6
    temp = temp * HM[0][5];
    pp = M44multV14(temp, p);
    std::cout << "link6 " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // link 7
    temp = temp * HM[0][6];
    pp = M44multV14(temp, p);
    std::cout << "link7 " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // hand
//    std::cout<<"HMhand:\n";
//    for(int m = 0; m < 4; m++){
//        for(int n = 0; n < 4; n++)
//        {
//            std::cout << "HMhand[" << m << ", " << n << "]\n";
//            HMhand[0].M[m][n].print();
//        }
//    }
    pp = M44multV14(HMhand[0], p);
    std::cout << "hand " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // left finger
    pp = M44multV14(HMleftfinger[0], p);
    std::cout << "left finger " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();
    // right finger
    pp = M44multV14(HMrightfinger[0], p);
    std::cout << "right finger " << " position :\n";
    pp.P[0].print();
    pp.P[1].print();
    pp.P[2].print();

//    std::cout << "panda_finger_joint1 = 0.04\n" << "panda_finger_joint2 = 0.04\n" << "\n";
//    // left finger
//    pp = M44multV14(HMleftfinger[0], p1);
//    std::cout << "left finger " << " position :\n";
//    pp.P[0].print();
//    pp.P[1].print();
//    pp.P[2].print();
//    // right finger
//    pp = M44multV14(HMrightfinger[0], p1);
//    std::cout << "right finger " << " position :\n";
//    pp.P[0].print();
//    pp.P[1].print();
//    pp.P[2].print();

    std::cout << "showHM over!\n";
}

Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y, const double z, const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
//    std::cout << "Quaternion2RotationMatrix result is:" << std::endl;
//    std::cout << "R = " << std::endl << R << std::endl << std::endl;
    return R;
}

cMatrix33 MEigen2cM(Eigen::Matrix3d& R) {
    double a00 = R(0, 0);
    double a01 = R(0, 1);
    double a02 = R(0, 2);
    double a10 = R(1, 0);
    double a11 = R(1, 1);
    double a12 = R(1, 2);
    double a20 = R(2, 0);
    double a21 = R(2, 1);
    double a22 = R(2, 2);
    cMatrix33 r(a00, a01, a02, a10, a11, a12, a20, a21, a22);

    return r;
}

cTM3Matrix33 MEigen2cTM(Eigen::Matrix3d& R) {
    double a00 = R(0, 0);
    double a01 = R(0, 1);
    double a02 = R(0, 2);
    double a10 = R(1, 0);
    double a11 = R(1, 1);
    double a12 = R(1, 2);
    double a20 = R(2, 0);
    double a21 = R(2, 1);
    double a22 = R(2, 2);
    cMatrix33 r(a00, a01, a02, a10, a11, a12, a20, a21, a22);
    cTM3Matrix33 rTM(r);
    return rTM;
}

cTM3Matrix33 Myaw(double omega, double q0) {
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);

    cTaylorModel3 c, s;
    c.cosModel(omega, q0);
    s.sinModel(omega, q0);

    cTaylorModel3 tm[3][3];
    tm[0][0] = c;
    tm[0][1] = -s;
    tm[0][2] = zero;
    tm[1][0] = s;
    tm[1][1] = c;
    tm[1][2] = zero;
    tm[2][0] = zero;
    tm[2][1] = zero;
    tm[2][2] = one;
    cTM3Matrix33 M(tm);
    return M;
}

cTM3Matrix33 Myaw_uncertain(double omega, double q0, double limit_l, double limit_u)
{
    cTaylorModel3 zero(0);
    cTaylorModel3 one(1);

    cTaylorModel3 c, s;
    c.cosModel(omega, q0);
    s.sinModel(omega, q0);

    c.r.i[0] += limit_l;
    c.r.i[1] += limit_u;
    s.r.i[0] += limit_l;
    s.r.i[1] += limit_u;

    cTaylorModel3 tm[3][3];
    tm[0][0] = c;
    tm[0][1] = -s;
    tm[0][2] = zero;
    tm[1][0] = s;
    tm[1][1] = c;
    tm[1][2] = zero;
    tm[2][0] = zero;
    tm[2][1] = zero;
    tm[2][2] = one;
    cTM3Matrix33 M(tm);
    return M;
}

// ********************************************************************************** //
void model_load(std::string filename, fcl::BVHModel<fcl::OBBRSS> &obj){
    // set mesh triangles and vertices indices
    std::vector<fcl::Vec3f> p1;
    std::vector<fcl::Triangle> t1;

    // code to set the vertices and triangles
    boost::filesystem::path path(OBJ_RESOURCES_DIR);
    // from test_fcl_utility.h target_link_library
//    std::cout<<"filename: "<<filename<<std::endl;

    loadOBJFile((path / filename).string().c_str(), p1, t1);

    // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
    obj.bv_splitter.reset(new fcl::BVSplitter<fcl::OBBRSS>(fcl::SPLIT_METHOD_MEAN));

    // add the mesh data into the BVHModel structure
    obj.beginModel();
    obj.addSubModel(p1, t1);
    obj.endModel();

//    std::cout<<"OK! NO PROBLEM!"<<std::endl;
}

void model_load(std::string filename, fcl::BVHModel<fcl::SPHERE> &obj){
    // set mesh triangles and vertices indices
    std::vector<fcl::Vec3f> p1;
    std::vector<fcl::Triangle> t1;

    // code to set the vertices and triangles
    boost::filesystem::path path(OBJ_RESOURCES_DIR);
    // from test_fcl_utility.h target_link_library
//    std::cout<<"filename: "<<filename<<std::endl;

    loadOBJFile((path / filename).string().c_str(), p1, t1);

    // BVHModel is a template class for mesh geometry, for default OBBRSS template is used
    obj.bv_splitter.reset(new fcl::BVSplitter<fcl::SPHERE>(fcl::SPLIT_METHOD_MEAN));

    // add the mesh data into the BVHModel structure
    obj.beginModel();
    obj.addSubModel(p1, t1);
    obj.endModel();

//    std::cout<<"OK! NO PROBLEM!"<<std::endl;
}

void filename_load(std::vector<std::string> &filenames, const std::string folder)
{
    boost::filesystem::path directory(folder);
    boost::filesystem::directory_iterator itr(directory), end_itr;
    std::string current_file = itr->path().string();

    for (; itr != end_itr; ++itr)
    {
        if (is_regular_file(itr->path()))
        {
            std::string filename = itr->path().filename().string(); // returns just filename
            std::cout << "filename: " << filename << std::endl;
            filenames.push_back(filename);
        }
    }

}

void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles)
{

    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cerr << "file not exist" << std::endl;
        return;
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
            case 'v':
            {
                if(first_token[1] == 'n')
                {
                    strtok(NULL, "\t ");
                    strtok(NULL, "\t ");
                    strtok(NULL, "\t ");
                    has_normal = true;
                }
                else if(first_token[1] == 't')
                {
                    strtok(NULL, "\t ");
                    strtok(NULL, "\t ");
                    has_texture = true;
                }
                else
                {
                    fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                    fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                    fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                    fcl::Vec3f p(x, y, z);
                    points.push_back(p);
                }
            }
                break;
            case 'f':
            {
                fcl::Triangle tri;
                char* data[30];
                int n = 0;
                while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
                {
                    if(strlen(data[n]))
                        n++;
                }

                for(int t = 0; t < (n - 2); ++t)
                {
                    if((!has_texture) && (!has_normal))
                    {
                        tri[0] = atoi(data[0]) - 1;
                        tri[1] = atoi(data[1]) - 1;
                        tri[2] = atoi(data[2]) - 1;
                    }
                    else
                    {
                        const char *v1;
                        for(int i = 0; i < 3; i++)
                        {
                            // vertex ID
                            if(i == 0)
                                v1 = data[0];
                            else
                                v1 = data[t + i];

                            tri[i] = atoi(v1) - 1;
                        }
                    }
                    triangles.push_back(tri);
                }
            }
        }
    }
}

fcl::FCL_REAL isoverlarp(int i, int j, cTMVector4 p, cTMVector4 q, cInterval T, fcl::FCL_REAL toc_err)
{
    // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
    cTM3Matrix44 HMp = HM[0][0];
    for(int i_ = 1; i_ < i; i_++){
        HMp = HMp * HM[0][i_];
    }

    cTMVector4 p1 = M44multV14(HMp, p);

    // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
    cTM3Matrix44 HMq = HM[0][0];
    for(int j_ = 1; j_ < j; j_++){
        HMp = HMp * HM[0][j_];
    }

    cTMVector4 q1 = M44multV14(HMq, q);

    // calculate the distance between two point
    cTaylorModel3 dis = distance_square_cal(p1, q1);

    // find minimum of the distance, t = argmin(distance^2 - (r1 + r2 + epsilon)^2)
//    cInterval T;
    ga_res res = findminimum(dis,T);

    // return result, epsilon = 0
    if (res.minimum < toc_err)
        return -1;
    else return res.minimum;
}

fcl::FCL_REAL BVHTraversal(int rootA, int rootB, fcl::BVHModel<fcl::OBBRSS> A, fcl::BVHModel<fcl::OBBRSS> B)
{
    fcl::BVNode<fcl::OBBRSS> bvA, bvB;
    bvA = A.getBV(rootA);
    bvB = B.getBV(rootB);

//    fcl::FCL_REAL dis = BVRSSCollide(bvA,bvB);
//    std::cout << "bv " << rootA << " vs bv " << rootB << ": \n";
    fcl::FCL_REAL dis = BVRSSCapsuleCollide(bvA,bvB);
//    std::cout << "dis: " << dis << "\n";
//    std::cout << "r: " << bvA.bv.rss.r << ", " << bvB.bv.rss.r << "\n";
//    std::cout << "\n";

    if(dis<0)
    {
        if (bvA.isLeaf() || bvA.bv.rss.r < __toc_err) {
            if (bvB.isLeaf() || bvB.bv.rss.r < __toc_err) {
                // return
                return dis;
            } else {
                // A, B.leftchild()
                // A, B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(rootA, bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal(rootA, bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_.at(0);
                }
            }
        } else {
            if (bvB.isLeaf() || bvB.bv.rss.r < __toc_err) {
                // A.leftchild(), B
                // A.rightchild(), B
                std::vector<fcl::FCL_REAL> dis_;
                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), rootB, A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal(bvA.rightChild(), rootB, A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_.at(0);
                }
            } else {
                // A.leftchild(), B.leftchild()
                // A.leftchild(), B.rightchild()
                // A.rightchild(), B.leftchild()
                // A.rightchild(), B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.leftChild(), bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_.at(0);
                }
            }
        }
    }
    else return dis;
}

fcl::FCL_REAL BVRSSCollide(fcl::BVNode<fcl::OBBRSS> bvA, fcl::BVNode<fcl::OBBRSS> bvB)
{
    // RSS RSS

    // minimum outside phere of obb is no-collide, return
    cTMVector4 pA(bvA.bv.obb.To[0], bvA.bv.obb.To[1], bvA.bv.obb.To[2], 1);
    cTMVector4 pB(bvB.bv.obb.To[0], bvB.bv.obb.To[1], bvB.bv.obb.To[2], 1);
    cInterval judgbase = disbound(pA, pB) - pow((sqrt(bvA.bv.obb.size()) + sqrt(bvB.bv.obb.size()) + __toc_err), 2);
    if(judgbase.i[0] > 0)
        return judgbase.i[0];

    fcl::FCL_REAL sideA[2], sideB[2];  // side[0] long side, side[1] short side
    if (bvA.bv.rss.l[0] > bvA.bv.rss.l[1]) // l[2]: Side lengths of rectangle
    {
        sideA[0] = bvA.bv.rss.l[0];
        sideA[1] = bvA.bv.rss.l[1];
    } else {
        sideA[0] = bvA.bv.rss.l[1];
        sideA[1] = bvA.bv.rss.l[0];
    }
    if (bvB.bv.rss.l[0] > bvB.bv.rss.l[1]) {
        sideB[0] = bvB.bv.rss.l[0];
        sideB[1] = bvB.bv.rss.l[1];
    } else {
        sideB[0] = bvB.bv.rss.l[1];
        sideB[1] = bvB.bv.rss.l[0];
    }
    // |             side 3
    // |     s ******************** s
    // | a   i *                  * i
    // V x   d *                  * d
    //   i   e *                  * e
    //   s   2 ******************** 4
    //   1    Tr     side 1
    //          ------>axis0
    //
    // Tr: Origin of the rectangle in RSS
    // axis[3]: Orientation of RSS. axis[i] is the ith column of the orientation matrix for the RSS;
    //          it is also the i-th principle direction of the RSS.
    //          We assume that axis[0] corresponds to the axis with the longest length,
    //          axis[1] corresponds to the shorter one and axis[2] corresponds to the shortest one.
    // r: Radius of sphere summed with rectangle to form RSS.
    // the shortest axis forming the rectangle face normal
    //
    //  edge    origin          length  axis
    //  1       Tr              side0   axis0
    //  2       Tr              side1   axis1
    //  3       Tr+side1*axis1  side0   axis0
    //  4       Tr+side0*axis0  side1   axis1
    fcl::FCL_REAL ans;
    // A1 vs B1
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A1 vs B2
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A1 vs B3
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A1 vs B4
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A2 vs B1
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A2 vs B2
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A2 vs B3
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A2 vs B4
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A3 vs B1
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A3 vs B2
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A3 vs B3
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A3 vs B4
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A4 vs B1
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A4 vs B2
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A4 vs B3
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A4 vs B4
    ans = side2side(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                    bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;

    return ans;
}

fcl::FCL_REAL BVRSSCapsuleCollide(fcl::BVNode<fcl::OBBRSS> bvA, fcl::BVNode<fcl::OBBRSS> bvB)
{
    // RSS RSS

    // minimum outside phere of obb is no-collide, return
    cTMVector4 pA(bvA.bv.obb.To[0], bvA.bv.obb.To[1], bvA.bv.obb.To[2], 1);
    cTMVector4 pB(bvB.bv.obb.To[0], bvB.bv.obb.To[1], bvB.bv.obb.To[2], 1);
    cInterval judgbase = disbound(pA, pB) - pow((sqrt(bvA.bv.obb.size()) + sqrt(bvB.bv.obb.size()) + __toc_err), 2);
    std::cout << "disbound: [" << judgbase.i[0] << ", " << judgbase.i[1] << "] \n";
    if(judgbase.i[0] >= 0)
        return judgbase.i[0];

    fcl::FCL_REAL sideA[2], sideB[2];  // side[0] long side, side[1] short side
    if (bvA.bv.rss.l[0] > bvA.bv.rss.l[1]) // l[2]: Side lengths of rectangle
    {
        sideA[0] = bvA.bv.rss.l[0];
        sideA[1] = bvA.bv.rss.l[1];
    } else {
        sideA[0] = bvA.bv.rss.l[1];
        sideA[1] = bvA.bv.rss.l[0];
    }
    if (bvB.bv.rss.l[0] > bvB.bv.rss.l[1]) {
        sideB[0] = bvB.bv.rss.l[0];
        sideB[1] = bvB.bv.rss.l[1];
    } else {
        sideB[0] = bvB.bv.rss.l[1];
        sideB[1] = bvB.bv.rss.l[0];
    }
    // |             side 3
    // |     s ******************** s
    // | a   i *                  * i
    // V x   d *                  * d
    //   i   e *                  * e
    //   s   2 ******************** 4
    //   1    Tr     side 1
    //          ------>axis0
    //
    // Tr: Origin of the rectangle in RSS
    // axis[3]: Orientation of RSS. axis[i] is the ith column of the orientation matrix for the RSS;
    //          it is also the i-th principle direction of the RSS.
    //          We assume that axis[0] corresponds to the axis with the longest length,
    //          axis[1] corresponds to the shorter one and axis[2] corresponds to the shortest one.
    // r: Radius of sphere summed with rectangle to form RSS.
    // the shortest axis forming the rectangle face normal
    //
    //  edge    origin          length  axis
    //  1       Tr              side0   axis0
    //  2       Tr              side1   axis1
    //  3       Tr+side1*axis1  side0   axis0
    //  4       Tr+side0*axis0  side1   axis1
    fcl::FCL_REAL ans;
    // A1 vs B1
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A1 vs B2
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A1 vs B3
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A1 vs B4
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A2 vs B1
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A2 vs B2
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A2 vs B3
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A2 vs B4
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr, sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A3 vs B1
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A3 vs B2
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A3 vs B3
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A3 vs B4
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[1] * bvA.bv.rss.axis[1], sideA[0], bvA.bv.rss.axis[0],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A4 vs B1
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A4 vs B2
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr, sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;
    // A4 vs B3
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[1] * bvB.bv.rss.axis[1], sideB[0], bvB.bv.rss.axis[0]);
    if (ans < 0) return ans;
    // A4 vs B4
    ans = capsule2capsule(bvA.bv.rss.r, bvA.bv.rss.Tr + sideA[0] * bvA.bv.rss.axis[0], sideA[1], bvA.bv.rss.axis[1],
                          bvB.bv.rss.r, bvB.bv.rss.Tr + sideB[0] * bvB.bv.rss.axis[0], sideB[1], bvB.bv.rss.axis[1]);
    if (ans < 0) return ans;

    return ans;
}

fcl::FCL_REAL side2side(fcl::FCL_REAL rA, fcl::Vec3f TrA, fcl::FCL_REAL sideA, fcl::Vec3f axisA,
                        fcl::FCL_REAL rB, fcl::Vec3f TrB, fcl::FCL_REAL sideB, fcl::Vec3f axisB)
{
    // rough judgment
    // minimum outside phere of obb is no-collide, return
    fcl::Vec3f cenA, cenB;
    cenA = TrA + 0.5 * sideA * axisA;
    cenB = TrB + 0.5 * sideB * axisB;
    cTMVector4 pA(cenA[0], cenA[1], cenA[2], 1);
    cTMVector4 pB(cenB[0], cenB[1], cenB[2], 1);
    cInterval judg = disbound(pA, pB) - pow((0.5 * sideA + rA + 0.5 * sideB +rB + __toc_err), 2);
    if(judg.i[0] > 0)
        return judg.i[0];

    // calculate the step
    double stepA = sideA / (ceil(sideA / rA) + 1);
    double stepB = sideB / (ceil(sideB / rB) + 1);
    cInterval judgbase;
    for(int i = 0; stepA*i <= sideA; i++)
    {
        for(int j = 0; stepB*j <= sideB; j++)
        {
            fcl::Vec3f pointA = TrA + i*stepA*axisA;
            fcl::Vec3f pointB = TrB + j*stepB*axisB;
            cTMVector4 pA(pointA.data[0], pointA.data[1], pointA.data[2], 1);
            cTMVector4 pB(pointB.data[0], pointB.data[1], pointB.data[2], 1);
            judgbase = disbound(pA, pB) - (rA + rB + __toc_err)*(rA + rB + __toc_err);
            if(judgbase.i[0] < 0)
            {
                judgbase.print();
                return judgbase.i[0];
            }
        }
    }
    return judgbase.i[0];
}

fcl::FCL_REAL capsule2capsule(fcl::FCL_REAL rA, fcl::Vec3f TrA, fcl::FCL_REAL sideA, fcl::Vec3f axisA,
                              fcl::FCL_REAL rB, fcl::Vec3f TrB, fcl::FCL_REAL sideB, fcl::Vec3f axisB)
{
    fcl::Vec3f pointA = TrA + 0.5 * sideA * axisA;
    fcl::Vec3f pointB = TrB + 0.5 * sideB * axisB;
    cTMVector4 pA(pointA.data[0], pointA.data[1], pointA.data[2], 1);
    cTMVector4 pB(pointB.data[0], pointB.data[1], pointB.data[2], 1);
    cInterval judgbase = disbound(pA, pB) - pow(((0.5 * sideA + rA) + (0.5 * sideB + rB) + __toc_err), 2);

    if (judgbase.i[0] >= 0) {
        // no-collide
        return judgbase.i[0];
    } else {
        // collide
        if (sideA > rA) {
            if (sideB > rB) {
                // capsuleA split into two capsules
                // capsuleB split into two capsules
                // A left B left
                fcl::FCL_REAL ans1 = capsule2capsule(rA, TrA, 0.5 * sideA, axisA, rB, TrB, 0.5 * sideB, axisB);
                if (ans1 < 0) return ans1;
                // A left B right
                fcl::FCL_REAL ans2 = capsule2capsule(rA, TrA, 0.5 * sideA, axisA, rB,TrB + 0.5 * sideB * axisB, 0.5 * sideB, axisB);
                if (ans2 < 0) return ans2;
                // A right B left
                fcl::FCL_REAL ans3 = capsule2capsule(rA, TrA + 0.5 * sideA * axisA, 0.5 * sideA, axisA, rB, TrB, 0.5 * sideB, axisB);
                if (ans3 < 0) return ans3;
                // A right B right
                fcl::FCL_REAL ans4 = capsule2capsule(rA, TrA + 0.5 * sideA * axisA, 0.5 * sideA, axisA, rB,TrB + 0.5 * sideB * axisB, 0.5 * sideB, axisB);
                if (ans4 < 0) return ans4;
                else return std::min(ans1, std::min(ans2, std::min(ans3, ans4)));
            } else {
                // capsuleA split into two capsules
                // capsuleB
                fcl::FCL_REAL ans1 = capsule2capsule(rA, TrA, 0.5 * sideA, axisA, rB, TrB, sideB, axisB);
                if (ans1 < 0) return ans1;
                fcl::FCL_REAL ans2 = capsule2capsule(rA, TrA + 0.5 * sideA * axisA, 0.5 * sideA, axisA, rB, TrB, sideB, axisB);
                if (ans2 < 0) return ans2;
                else return std::min(ans1, ans2);
            }
        } else {
            if (sideB > rB) {
                // capsuleA
                // capsuleB split into two capsules
                fcl::FCL_REAL ans1 = capsule2capsule(rA, TrA, sideA, axisA, rB, TrB, 0.5 * sideB, axisB);
                if (ans1 < 0) return ans1;
                fcl::FCL_REAL ans2 = capsule2capsule(rA, TrA, sideA, axisA, rB,TrB + 0.5 * sideB * axisB, 0.5 * sideB, axisB);
                if (ans2 < 0) return ans2;
                else return std::min(ans1, ans2);
            } else {
                // capsuleA
                // capsuleB
                return judgbase.i[0];
            }
        }
    }
}

cInterval disbound(cTMVector4 &p, cTMVector4 &q)
{
    setGlobalTimeInterval(__T.i[0], __T.i[1]);

    if (link_type[0]==0 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }
        HMp = HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }
        HMp = HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-hand
//            std::cout << "link" << __linki << "-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }

        HMp = HMbody * HMp;

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-finger
//            std::cout << "link" << __linki << "-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }

        HMp = HMbody * HMp;

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-finger
//            std::cout << "link" << __linki << "-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-link
//            std::cout << "hand-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-hand
//            std::cout << "hand-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-finger
//            std::cout << "hand-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-finger
//            std::cout << "hand-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-link
//            std::cout << "left finger-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-hand
//            std::cout << "left finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "left finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "left finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-link
//            std::cout << "right finger-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-hand
//            std::cout << "right finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "right finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "right finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }
        HMp = HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==3) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==2) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==1) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==0) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else {
        // unreachable area
        return std::numeric_limits<fcl::FCL_REAL>::max();
    }
}

cInterval disbound(cTMVector4 &p, cTMVector4 &q, const int& typeA, const int& typeB)
{
    setGlobalTimeInterval(__T.i[0], __T.i[1]);

    if (typeA==0 && typeB==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }
        HMp = HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==0 && typeB==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }
        HMp = HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-hand
//            std::cout << "link" << __linki << "-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==0 && typeB==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }

        HMp = HMbody * HMp;

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-finger
//            std::cout << "link" << __linki << "-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==0 && typeB==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }

        HMp = HMbody * HMp;

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-finger
//            std::cout << "link" << __linki << "-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==1 && typeB==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-link
//            std::cout << "hand-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==1 && typeB==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-hand
//            std::cout << "hand-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==1 && typeB==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-finger
//            std::cout << "hand-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==1 && typeB==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-finger
//            std::cout << "hand-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==2 && typeB==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-link
//            std::cout << "left finger-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==2 && typeB==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-hand
//            std::cout << "left finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==2 && typeB==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "left finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==2 && typeB==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "left finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==3 && typeB==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-link
//            std::cout << "right finger-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==3 && typeB==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-hand
//            std::cout << "right finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==3 && typeB==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "right finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==3 && typeB==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "right finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==0 && typeB==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            HMp = HMp * HM[0][i_];
        }
        HMp = HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==1 && typeB==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMhand[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==2 && typeB==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMleftfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==3 && typeB==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = HMbody * HMrightfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==4 && typeB==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==4 && typeB==3) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==4 && typeB==2) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==4 && typeB==1) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HMbody * HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (typeA==4 && typeB==0) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            HMq = HMq * HM[0][j_];
        }
        HMq = HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else {
        // unreachable area
        return std::numeric_limits<fcl::FCL_REAL>::max();
    }
}

void IDtypedecoding(std::string ID, int &link_type, int &__link) {
    char s[20];
    for (int i = 0; i < ID.length(); i++) {
        s[i] = ID.at(i);
    }
    char *token = strtok(s, "_");
    token = strtok(NULL, "\0");

    std::string ss = token;

    if (ss.compare(0, 4, "link", 0, 4) == 0) {
        link_type = 0;
        __link = ss.at(4) - 48;
    } else if (ss.compare(0, 4, "hand", 0, 4) == 0) {
        link_type = 1;
    } else if (ss.compare(0, 4, "left", 0, 4) == 0) {
        link_type = 2;
    } else if (ss.compare(0, 5, "right", 0, 5) == 0) {
        link_type = 3;
    } else {

    }
}

void IDtypedecodingforyumi(std::string ID, int &link_type, int &__link)
{
    std::string s = ID;

    if (s.compare(0, 7, "gripper", 0, 7) == 0) {
        if (s.compare(10, 4, "base", 0, 4) == 0) {
            link_type = 1;
        } else if (s.compare(17, 1, "l", 0, 1) == 0) {
            link_type = 2;
        } else if (s.compare(17, 1, "r", 0, 1) == 0) {
            link_type = 3;
        } else {

        }
    } else if (s.compare(5, 4, "link", 0, 4) == 0) {
        link_type = 0;
        char c = s.at(10);
        __link = c - '0';
    } else if (s.compare(5, 4, "body", 0, 4) == 0) {
        link_type = 4;
    } else {

    }
}

fcl::FCL_REAL BVHTraversal(const int& rootA, const int& rootB, fcl::BVHModel<fcl::SPHERE> &A, fcl::BVHModel<fcl::SPHERE> &B)
{
    fcl::BVNode<fcl::SPHERE> &bvA = A.getBV(rootA);
    fcl::BVNode<fcl::SPHERE> &bvB = B.getBV(rootB);

//    fcl::FCL_REAL dis = BVRSSCollide(bvA,bvB);
//    std::cout << "bv " << rootA << " vs bv " << rootB << ": \n";
    clock_t start = clock();
    fcl::FCL_REAL dis = BVSPHERECollide(bvA,bvB);
    clock_t end = clock();
    running_time_TM3 += double(end-start)/CLOCKS_PER_SEC;
    tra_num++;
//    std::cout << "dis: " << dis << "\n";
//    std::cout << "r: " << bvA.bv.r << ", " << bvB.bv.r << "\n";
//    std::cout << "\n";

    if(dis<0)
    {
        if (bvA.isLeaf() || bvA.bv.r < __toc_err) {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // return
                return dis;
            } else {
                // A, B.leftchild()
                // A, B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(rootA, bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal(rootA, bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            }
        } else {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // A.leftchild(), B
                // A.rightchild(), B
                std::vector<fcl::FCL_REAL> dis_;
                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), rootB, A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal(bvA.rightChild(), rootB, A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            } else {
                // A.leftchild(), B.leftchild()
                // A.leftchild(), B.rightchild()
                // A.rightchild(), B.leftchild()
                // A.rightchild(), B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.leftChild(), bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            }
        }
    }
    else return dis;
}

fcl::FCL_REAL SPHERETREETraversal(std::vector<ball> &A, std::vector<ball> &B, bool onlyleaf)
{
    /// TODO: SPHERETREE Traversal
    /// if onlyleaf is true, spheretree A and spheretree B only have leaf nodes
    /// else spheretree A and B have root and leaf nodes
    double dis;
    if (onlyleaf) {
        for (int i = 0; i < A.size(); i++) {
            for (int j = 0; j < B.size(); j++) {
                dis = BALLCollide(A[i], B[j]);
                tra_num++;
                if (dis < 0)
                    return dis;
            }
        }
        return dis;
    }
    else {
        dis = BALLCollide(A[0], B[0]);
        tra_num++;
        if (dis > 0) return dis;
        for (int i = 1; i < A.size(); i++) {
            for (int j = 1; j < B.size(); j++) {
                dis = BALLCollide(A[i], B[j]);
                tra_num++;
                if (dis < 0)
                    return dis;
            }
        }
    }

}

/// @brief BVHTraversal for SPHERETREE (http://isg.cs.tcd.ie/spheretree/)
/// rootA && rootB: current node index
/// sum: total nodes of whole spheretree, sum = (branch^depth - 1)/(branch - 1)
/// index of last node is sum - 1
fcl::FCL_REAL SPHERETREETraversal(const int& rootA, const int &rootB, std::vector<ball> &A, std::vector<ball> &B,
                                  const int& depth, const int& branch)
{
    double dis;
    dis = BALLCollide(A[rootA], B[rootB]);
    tra_num++;
    if (dis > 0) return dis;

    int lastnodeA = (rootA + 1) * branch;
    int lastnodeB = (rootB + 1) * branch;
    std::vector<double> dis_;
    if (lastnodeA < A.size() && lastnodeB < B.size()) {
        for (int i = rootA * branch + 1; i <= lastnodeA; i++) {
            for (int j = rootB * branch + 1; j <= lastnodeB; j++) {
                double tmp = SPHERETREETraversal(i, j, A, B, depth, branch);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
            }
        }
    }
    else if (lastnodeA < A.size()) {
        for (int i = rootA * branch + 1; i <= lastnodeA; i++) {
            double tmp = SPHERETREETraversal(i, rootB, A, B, depth, branch);
            if (tmp < 0) return tmp;
            else dis_.push_back(tmp);
        }
    }
    else if (lastnodeB < B.size()) {
        for (int j = rootB * branch + 1; j <= lastnodeB; j++) {
            double tmp = SPHERETREETraversal(rootA, j, A, B, depth, branch);
            if (tmp < 0) return tmp;
            else dis_.push_back(tmp);
        }
    }
    else {
        return dis;
    }

    if (dis_.empty()) {
        return dis;
    } else {
        std::sort(dis_.begin(), dis_.begin()+dis_.size());
        return dis_[0];
    }
}

fcl::FCL_REAL SPHERETREETraversal(const int& rootA, const int &rootB,
                                  const int& depth, const int& branch, CCDTMData &cdata)
{
    double dis;
    dis = BALLCollide(cdata.A[rootA], cdata.B[rootB], cdata);
    cdata.tra_num++;
    if (dis > 0) return dis;

    int lastnodeA = (rootA + 1) * branch;
    int lastnodeB = (rootB + 1) * branch;
    std::vector<double> dis_;
    if (lastnodeA < cdata.A.size() && lastnodeB < cdata.B.size()) {
        for (int i = rootA * branch + 1; i <= lastnodeA; i++) {
            for (int j = rootB * branch + 1; j <= lastnodeB; j++) {
                double tmp = SPHERETREETraversal(i, j, depth, branch, cdata);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
            }
        }
    }
    else if (lastnodeA < cdata.A.size()) {
        for (int i = rootA * branch + 1; i <= lastnodeA; i++) {
            double tmp = SPHERETREETraversal(i, rootB, depth, branch, cdata);
            if (tmp < 0) return tmp;
            else dis_.push_back(tmp);
        }
    }
    else if (lastnodeB < cdata.B.size()) {
        for (int j = rootB * branch + 1; j <= lastnodeB; j++) {
            double tmp = SPHERETREETraversal(rootA, j, depth, branch, cdata);
            if (tmp < 0) return tmp;
            else dis_.push_back(tmp);
        }
    }
    else {
        return dis;
    }

    if (dis_.empty()) {
        return dis;
    } else {
        std::sort(dis_.begin(), dis_.begin()+dis_.size());
        return dis_[0];
    }
}

fcl::FCL_REAL BVHTraversal(const int& rootA, const int &rootB, fcl::BVHModel<fcl::SPHERE> &A, fcl::BVHModel<fcl::SPHERE> &B, const int& typeA, const int& typeB)
{
    fcl::BVNode<fcl::SPHERE> &bvA = A.getBV(rootA);
    fcl::BVNode<fcl::SPHERE> &bvB = B.getBV(rootB);

//    fcl::FCL_REAL dis = BVRSSCollide(bvA,bvB);
//    std::cout << "bv " << rootA << " vs bv " << rootB << ": \n";
    fcl::FCL_REAL dis = BVSPHERECollide(bvA,bvB, typeA, typeB);
//    std::cout << "dis: " << dis << "\n";
//    std::cout << "r: " << bvA.bv.r << ", " << bvB.bv.r << "\n";
//    std::cout << "\n";

    if(dis<0)
    {
        if (bvA.isLeaf() || bvA.bv.r < __toc_err) {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // return
                return dis;
            } else {
                // A, B.leftchild()
                // A, B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(rootA, bvB.leftChild(), A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal(rootA, bvB.rightChild(), A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            }
        } else {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // A.leftchild(), B
                // A.rightchild(), B
                std::vector<fcl::FCL_REAL> dis_;
                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), rootB, A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal(bvA.rightChild(), rootB, A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            } else {
                // A.leftchild(), B.leftchild()
                // A.leftchild(), B.rightchild()
                // A.rightchild(), B.leftchild()
                // A.rightchild(), B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), bvB.leftChild(), A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.leftChild(), bvB.rightChild(), A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.leftChild(), A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.rightChild(), A, B, typeA, typeB);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            }
        }
    }
    else return dis;
}

fcl::FCL_REAL BVSPHERECollide(fcl::BVNode<fcl::SPHERE> &bvA, fcl::BVNode<fcl::SPHERE> &bvB)
{
    // SPHERE SPHERE

    cTMVector4 pA(bvA.bv.Tr[0], bvA.bv.Tr[1], bvA.bv.Tr[2], 1);
    cTMVector4 pB(bvB.bv.Tr[0], bvB.bv.Tr[1], bvB.bv.Tr[2], 1);
    cInterval judgbase = disbound(pA, pB) - pow((bvA.bv.r + bvB.bv.r + __toc_err), 2);

    return judgbase.i[0];
}

fcl::FCL_REAL BVSPHERECollide(fcl::BVNode<fcl::SPHERE> &bvA, fcl::BVNode<fcl::SPHERE> &bvB, const int &typeA, const int &typeB)
{
    // SPHERE SPHERE

    cTMVector4 pA(bvA.bv.Tr[0], bvA.bv.Tr[1], bvA.bv.Tr[2], 1);
    cTMVector4 pB(bvB.bv.Tr[0], bvB.bv.Tr[1], bvB.bv.Tr[2], 1);
    cInterval judgbase = disbound(pA, pB, typeA, typeB) - pow((bvA.bv.r + bvB.bv.r + __toc_err), 2);

    return judgbase.i[0];
}

fcl::FCL_REAL BALLCollide(ball &a, ball &b){
    // Ball vs Ball
    // Ball : x,y,z,r
    cTMVector4 pA(a.x, a.y, a.z, 1);
    cTMVector4 pB(b.x, b.y, b.z, 1);
    cInterval judgebase = disbound(pA, pB) - pow(a.r + b.r + __toc_err, 2);

    return judgebase.i[0];
}

fcl::FCL_REAL BALLCollide(ball &a, ball &b, CCDTMData &cdata){
    // Ball vs Ball
    // Ball : x,y,z,r
    cTMVector4 pA(a.x, a.y, a.z, 1);
    cTMVector4 pB(b.x, b.y, b.z, 1);
    cInterval judgebase = disbound(pA, pB, cdata) - pow(a.r + b.r + cdata.__toc_err, 2);

    return judgebase.i[0];
}

void setupDualArm_TM2_HM_movable(int num, cInterval& interval, motion_TM2& sub_motion)
{
    setGlobalTimeInterval(interval.i[0], interval.i[1]);
    // for world to body
    Eigen::Matrix3d R = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 r = MEigen2cM(R);
    double xyz[3] = {0,0,0.1};
    cTM2Matrix44 TM2_HMtmp(r, xyz);
    TM2_HMbody = TM2_HMtmp;

    // for Link
    cTaylorModel2 zero(0);
    cTaylorModel2 one(1);
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < num; j++)
        {
            // Rinit
            Eigen::Matrix3d R1 = Quaternion2RotationMatrix(frame[i][j].quaternion[0], frame[i][j].quaternion[1], frame[i][j].quaternion[2], frame[i][j].quaternion[3]);
            cMatrix33 r1;
            TM2_Rinit[i][j] = MEigen2cTM2(R1);

            // Ryaw
//            Ryaw[i][j] = Myaw(frame[i][j].yaw_omega, frame[i][j].yaw0);
            int joint_num = sub_motion.start_state.size() / 2;
            if (i == 0) {
                // left_arm
                TM2_Ryaw[i][j] = Myaw_TM2(sub_motion.velocity[j], sub_motion.start_state[j]);
            }
            else {
                // right_arm
                TM2_Ryaw[i][j] = Myaw_TM2(sub_motion.velocity[j + joint_num], sub_motion.start_state[j + joint_num]);
            }

            // HM
            cTM2Matrix33 R = TM2_Rinit[i][j] * TM2_Ryaw[i][j];
            cTM2Matrix44 temp;
            cTaylorModel2 x(frame[i][j].xyz[0]);
            cTaylorModel2 y(frame[i][j].xyz[1]);
            cTaylorModel2 z(frame[i][j].xyz[2]);

            temp.M[0][0] = R.i[0][0];
            temp.M[0][1] = R.i[0][1];
            temp.M[0][2] = R.i[0][2];
            temp.M[0][3] = x;
            temp.M[1][0] = R.i[1][0];
            temp.M[1][1] = R.i[1][1];
            temp.M[1][2] = R.i[1][2];
            temp.M[1][3] = y;
            temp.M[2][0] = R.i[2][0];
            temp.M[2][1] = R.i[2][1];
            temp.M[2][2] = R.i[2][2];
            temp.M[2][3] = z;
            temp.M[3][0] = zero;
            temp.M[3][1] = zero;
            temp.M[3][2] = zero;
            temp.M[3][3] = one;

            if (j == 0) {
                TM2_HM[i][j] = TM2_HMbody * temp;
            }
            else {
                TM2_HM[i][j] = TM2_HM[i][j-1] * temp;
            }
        }
    }

    // assume, the gripper is fixed
    // for gripper
    // yumi_link_7 to gripper_base
    Eigen::Matrix3d  R1 = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 r1 = MEigen2cM(R1);
    double xyz1[3] = {0,0,0.007};
    cTM2Matrix44 TM2_HMgripperbase(r1, xyz1);
    TM2_HMhand[0] = TM2_HM[0][num - 1] * TM2_HMgripperbase;
    TM2_HMhand[1] = TM2_HM[1][num - 1] * TM2_HMgripperbase;

    // for finger
    Eigen::Matrix3d Rl = Quaternion2RotationMatrix(0.0, 0.0, 1.0, -1.0341155355510722e-13);
    cMatrix33 rl = MEigen2cM(Rl);
    double xyzl[3] = {0.0, 0.0065, 0.0837};
    cTM2Matrix44 leftfinger(rl, xyzl);
    Eigen::Matrix3d Rr = Quaternion2RotationMatrix(0.0, 0.0, 0.0, 1.0);
    cMatrix33 rr = MEigen2cM(Rr);
    double xyzr[3] = {0.0, -0.0065, 0.0837};
    cTM2Matrix44 rightfinger(rr, xyzr);

    // for movable joint
    // yumi: gripper_joint: <axis xyz="-1,0,0">
    cTaylorModel2 dll, dlr, drl, drr;
    dll.setZero();
    dlr.setZero();
    drl.setZero();
    drr.setZero();
    dll.c[0] = -1 * sub_motion.start_state[num+1];
    dll.c[1] = -1 * sub_motion.velocity[num+1];
    dlr.c[0] = -1 * sub_motion.start_state[num];
    dlr.c[1] = -1 * sub_motion.velocity[num];
    drl.c[0] = -1 * sub_motion.start_state[num*2+3];
    drl.c[1] = -1 * sub_motion.velocity[num*2+3];
    drr.c[0] = -1 * sub_motion.start_state[num*2+2];
    drr.c[1] = -1 * sub_motion.velocity[num*2+2];
    cTM2Matrix44 finger_ll(one, zero, zero, dll,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM2Matrix44 finger_lr(one, zero, zero, dlr,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM2Matrix44 finger_rl(one, zero, zero, drl,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);
    cTM2Matrix44 finger_rr(one, zero, zero, drr,
                           zero, one, zero, zero,
                           zero, zero, one, zero,
                           zero, zero, zero, one);

    TM2_HMleftfinger[0] = TM2_HMhand[0] * leftfinger * finger_ll;
    TM2_HMrightfinger[0] = TM2_HMhand[0] * rightfinger * finger_lr;
    TM2_HMleftfinger[1] = TM2_HMhand[1] * leftfinger * finger_rl;
    TM2_HMrightfinger[1] = TM2_HMhand[1] * rightfinger * finger_rr;
}

cTM2Matrix33 MEigen2cTM2(Eigen::Matrix3d& R)
{
    double a00 = R(0, 0);
    double a01 = R(0, 1);
    double a02 = R(0, 2);
    double a10 = R(1, 0);
    double a11 = R(1, 1);
    double a12 = R(1, 2);
    double a20 = R(2, 0);
    double a21 = R(2, 1);
    double a22 = R(2, 2);
    cMatrix33 r(a00, a01, a02, a10, a11, a12, a20, a21, a22);
    cTM2Matrix33 rTM(r);
    return rTM;
}

cTM2Matrix33 Myaw_TM2(double omega, double q0) {
    cTaylorModel2 zero(0);
    cTaylorModel2 one(1);

    cTaylorModel2 c, s;
    c.cosModel(omega, q0);
    s.sinModel(omega, q0);

    cTaylorModel2 tm[3][3];
    tm[0][0] = c;
    tm[0][1] = -s;
    tm[0][2] = zero;
    tm[1][0] = s;
    tm[1][1] = c;
    tm[1][2] = zero;
    tm[2][0] = zero;
    tm[2][1] = zero;
    tm[2][2] = one;
    cTM2Matrix33 M(tm);
    return M;
}

fcl::FCL_REAL BVHTraversal_TM2(const int& rootA, const int& rootB, fcl::BVHModel<fcl::SPHERE> &A, fcl::BVHModel<fcl::SPHERE> &B)
{
    fcl::BVNode<fcl::SPHERE> &bvA = A.getBV(rootA);
    fcl::BVNode<fcl::SPHERE> &bvB = B.getBV(rootB);

//    fcl::FCL_REAL dis = BVRSSCollide(bvA,bvB);
//    std::cout << "bv " << rootA << " vs bv " << rootB << ": \n";
    clock_t start = clock();
    fcl::FCL_REAL dis = BVSPHERECollide_TM2(bvA,bvB);
    clock_t end = clock();
    running_time_TM2 += double(end-start)/CLOCKS_PER_SEC;
    tra_num++;
//    std::cout << "dis: " << dis << "\n";
//    std::cout << "r: " << bvA.bv.r << ", " << bvB.bv.r << "\n";
//    std::cout << "\n";

    if(dis<0)
    {
        if (bvA.isLeaf() || bvA.bv.r < __toc_err) {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // return
                return dis;
            } else {
                // A, B.leftchild()
                // A, B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal_TM2(rootA, bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal(rootA, bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            }
        } else {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // A.leftchild(), B
                // A.rightchild(), B
                std::vector<fcl::FCL_REAL> dis_;
                fcl::FCL_REAL tmp = BVHTraversal_TM2(bvA.leftChild(), rootB, A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                tmp = BVHTraversal_TM2(bvA.rightChild(), rootB, A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            } else {
                // A.leftchild(), B.leftchild()
                // A.leftchild(), B.rightchild()
                // A.rightchild(), B.leftchild()
                // A.rightchild(), B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal_TM2(bvA.leftChild(), bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal_TM2(bvA.leftChild(), bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal_TM2(bvA.rightChild(), bvB.leftChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);
                tmp = BVHTraversal_TM2(bvA.rightChild(), bvB.rightChild(), A, B);
                if (tmp < 0) return tmp;
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return dis;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    return dis_[0];
                }
            }
        }
    }
    else return dis;
}

fcl::FCL_REAL BVSPHERECollide_TM2(fcl::BVNode<fcl::SPHERE> &bvA, fcl::BVNode<fcl::SPHERE> &bvB)
{
    // SPHERE SPHERE

    cTM2Vector4 pA(bvA.bv.Tr[0], bvA.bv.Tr[1], bvA.bv.Tr[2], 1);
    cTM2Vector4 pB(bvB.bv.Tr[0], bvB.bv.Tr[1], bvB.bv.Tr[2], 1);
    cInterval judgbase = disbound(pA, pB) - pow((bvA.bv.r + bvB.bv.r + __toc_err), 2);

    return judgbase.i[0];
}

cInterval disbound(cTM2Vector4 &p, cTM2Vector4 &q)
{
    setGlobalTimeInterval(__T.i[0], __T.i[1]);

    if (link_type[0]==0 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            TM2_HMp = TM2_HMp * TM2_HM[0][i_];
        }
        TM2_HMp = TM2_HMbody * TM2_HMp;
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            TM2_HMq = TM2_HMq * TM2_HM[0][j_];
        }
        TM2_HMq = TM2_HMbody * TM2_HMq;
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            TM2_HMp = TM2_HMp * TM2_HM[0][i_];
        }
        TM2_HMp = TM2_HMbody * TM2_HMp;
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMhand[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-hand
//            std::cout << "link" << __linki << "-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            TM2_HMp = TM2_HMp * TM2_HM[0][i_];
        }

        TM2_HMp = TM2_HMbody * TM2_HMp;

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMleftfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-finger
//            std::cout << "link" << __linki << "-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            TM2_HMp = TM2_HMp * TM2_HM[0][i_];
        }

        TM2_HMp = TM2_HMbody * TM2_HMp;

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMrightfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-finger
//            std::cout << "link" << __linki << "-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMhand[0];
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            TM2_HMq = TM2_HMq * TM2_HM[0][j_];
        }
        TM2_HMq = TM2_HMbody * TM2_HMq;
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-link
//            std::cout << "hand-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMhand[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMhand[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-hand
//            std::cout << "hand-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMhand[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMleftfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-finger
//            std::cout << "hand-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMhand[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMrightfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // hand-finger
//            std::cout << "hand-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMleftfinger[0];
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            TM2_HMq = TM2_HMq * TM2_HM[0][j_];
        }
        TM2_HMq = TM2_HMbody * TM2_HMq;
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-link
//            std::cout << "left finger-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMleftfinger[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMhand[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-hand
//            std::cout << "left finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMleftfinger[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMleftfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "left finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMleftfinger[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMrightfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "left finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMrightfinger[0];
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            TM2_HMq = TM2_HMq * TM2_HM[0][j_];
        }
        TM2_HMq = TM2_HMbody * TM2_HMq;
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-link
//            std::cout << "right finger-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMrightfinger[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMhand[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-hand
//            std::cout << "right finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMrightfinger[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMleftfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "right finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMrightfinger[0];

        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMrightfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // finger-finger
//            std::cout << "right finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==0 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HM[0][0];
        for(int i_ = 1; i_ < __linki; i_++){
            TM2_HMp = TM2_HMp * TM2_HM[0][i_];
        }
        TM2_HMp = TM2_HMbody * TM2_HMp;
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Vector4 q1 = M44multV14(TM2_HMbody, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==1 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMhand[0];
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Vector4 q1 = M44multV14(TM2_HMbody, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==2 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMleftfinger[0];
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Vector4 q1 = M44multV14(TM2_HMbody, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==3 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Matrix44 TM2_HMp = TM2_HMbody * TM2_HMrightfinger[0];
        cTM2Vector4 p1 = M44multV14(TM2_HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Vector4 q1 = M44multV14(TM2_HMbody, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Vector4 p1 = M44multV14(TM2_HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Vector4 q1 = M44multV14(TM2_HMbody, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==3) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Vector4 p1 = M44multV14(TM2_HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMrightfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==2) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Vector4 p1 = M44multV14(TM2_HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMleftfinger[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==1) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Vector4 p1 = M44multV14(TM2_HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HMbody * TM2_HMhand[1];
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (link_type[0]==4 && link_type[1]==0) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM2Vector4 p1 = M44multV14(TM2_HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM2Matrix44 TM2_HMq = TM2_HM[0][0];
        for(int j_ = 1; j_ < __linkj; j_++){
            TM2_HMq = TM2_HMq * TM2_HM[0][j_];
        }
        TM2_HMq = TM2_HMbody * TM2_HMq;
        cTM2Vector4 q1 = M44multV14(TM2_HMq, q);

        // calculate the distance between two point
        cTaylorModel2 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < __toc_err){
//            // link-link
//            std::cout << "link" << __linki << "-link" << __linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else {
        // unreachable area
        return std::numeric_limits<fcl::FCL_REAL>::max();
    }
}

cInterval disbound(cTMVector4 &p, cTMVector4 &q, CCDTMData &cdata)
{
    setGlobalTimeInterval(cdata.__T.i[0], cdata.__T.i[1]);

    if (cdata.link_type[0]==0 && cdata.link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HM[0][0];
        for(int i_ = 1; i_ < cdata.__linki; i_++){
            HMp = HMp * cdata.HM[0][i_];
        }
        HMp = cdata.HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HM[0][0];
        for(int j_ = 1; j_ < cdata.__linkj; j_++){
            HMq = HMq * cdata.HM[0][j_];
        }
        HMq = cdata.HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==0 && cdata.link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HM[0][0];
        for(int i_ = 1; i_ < cdata.__linki; i_++){
            HMp = HMp * cdata.HM[0][i_];
        }
        HMp = cdata.HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-hand
//            std::cout << "link" << cdata.__linki << "-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==0 && cdata.link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HM[0][0];
        for(int i_ = 1; i_ < cdata.__linki; i_++){
            HMp = HMp * cdata.HM[0][i_];
        }

        HMp = cdata.HMbody * HMp;

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-finger
//            std::cout << "link" << cdata.__linki << "-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==0 && cdata.link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HM[0][0];
        for(int i_ = 1; i_ < cdata.__linki; i_++){
            HMp = HMp * cdata.HM[0][i_];
        }

        HMp = cdata.HMbody * HMp;

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-finger
//            std::cout << "link" << cdata.__linki << "-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==1 && cdata.link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMhand[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HM[0][0];
        for(int j_ = 1; j_ < cdata.__linkj; j_++){
            HMq = HMq * cdata.HM[0][j_];
        }
        HMq = cdata.HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // hand-link
//            std::cout << "hand-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==1 && cdata.link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // hand-hand
//            std::cout << "hand-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==1 && cdata.link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // hand-finger
//            std::cout << "hand-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==1 && cdata.link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMhand[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // hand-finger
//            std::cout << "hand-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==2 && cdata.link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMleftfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HM[0][0];
        for(int j_ = 1; j_ < cdata.__linkj; j_++){
            HMq = HMq * cdata.HM[0][j_];
        }
        HMq = cdata.HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-link
//            std::cout << "left finger-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==2 && cdata.link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMleftfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-hand
//            std::cout << "left finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==2 && cdata.link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        register cTM3Matrix44 HMp = cdata.HMbody * cdata.HMleftfinger[0];

        register cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        register cTM3Matrix44 HMq = cdata.HMbody * cdata.HMleftfinger[1];
        register cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        register cTaylorModel3 dis = distance_square_cal(p1, q1);

        register cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-finger
//            std::cout << "left finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==2 && cdata.link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        register cTM3Matrix44 HMp = cdata.HMbody * cdata.HMleftfinger[0];

        register cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        register cTM3Matrix44 HMq = cdata.HMbody * cdata.HMrightfinger[1];
        register cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        register cTaylorModel3 dis = distance_square_cal(p1, q1);

        register cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-finger
//            std::cout << "left finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==3 && cdata.link_type[1]==0)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMrightfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HM[0][0];
        for(int j_ = 1; j_ < cdata.__linkj; j_++){
            HMq = HMq * cdata.HM[0][j_];
        }
        HMq = cdata.HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-link
//            std::cout << "right finger-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==3 && cdata.link_type[1]==1)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMrightfinger[0];

        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-hand
//            std::cout << "right finger-hand"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==3 && cdata.link_type[1]==2)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        register cTM3Matrix44 HMp = cdata.HMbody * cdata.HMrightfinger[0];

        register cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        register cTM3Matrix44 HMq = cdata.HMbody * cdata.HMleftfinger[1];
        register cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        register cTaylorModel3 dis = distance_square_cal(p1, q1);

        register cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-finger
//            std::cout << "right finger-left finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==3 && cdata.link_type[1]==3)
    {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        register cTM3Matrix44 HMp = cdata.HMbody * cdata.HMrightfinger[0];

        register cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        register cTM3Matrix44 HMq = cdata.HMbody * cdata.HMrightfinger[1];
        register cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        register cTaylorModel3 dis = distance_square_cal(p1, q1);

        register cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // finger-finger
//            std::cout << "right finger-right finger"<<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==0 && cdata.link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HM[0][0];
        for(int i_ = 1; i_ < cdata.__linki; i_++){
            HMp = HMp * cdata.HM[0][i_];
        }
        HMp = cdata.HMbody * HMp;
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(cdata.HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==1 && cdata.link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMhand[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(cdata.HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==2 && cdata.link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMleftfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(cdata.HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==3 && cdata.link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTM3Matrix44 HMp = cdata.HMbody * cdata.HMrightfinger[0];
        cTMVector4 p1 = M44multV14(HMp, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(cdata.HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==4 && cdata.link_type[1]==4) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(cdata.HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTMVector4 q1 = M44multV14(cdata.HMbody, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==4 && cdata.link_type[1]==3) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(cdata.HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMrightfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==4 && cdata.link_type[1]==2) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(cdata.HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMleftfinger[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==4 && cdata.link_type[1]==1) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(cdata.HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HMbody * cdata.HMhand[1];
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else if (cdata.link_type[0]==4 && cdata.link_type[1]==0) {
        // calculate the point p' relative to frame 0, [p'; 1] = H*[p; 1]
        cTMVector4 p1 = M44multV14(cdata.HMbody, p);

        // calculate the point q' relative to frame 0, [q'; 1] = H*[q; 1]
        cTM3Matrix44 HMq = cdata.HM[0][0];
        for(int j_ = 1; j_ < cdata.__linkj; j_++){
            HMq = HMq * cdata.HM[0][j_];
        }
        HMq = cdata.HMbody * HMq;
        cTMVector4 q1 = M44multV14(HMq, q);

        // calculate the distance between two point
        cTaylorModel3 dis = distance_square_cal(p1, q1);

        cInterval bound = dis.bound();

//        if(bound.i[0] < cdata.__toc_err){
//            // link-link
//            std::cout << "link" << cdata.__linki << "-link" << cdata.__linkj <<"\n";
//            bound.print();
//        }

        // return result, epsilon = 0
        return bound;
    }
    else {
        // unreachable area
        return std::numeric_limits<fcl::FCL_REAL>::max();
    }
}
