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
#include <iostream>
#include <fstream>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <functional>
#include "ModelParameters.h"
#include "Taylor_Function.h"
#include "cInterval.h"

bool flag = false;
std::vector<double> vel;    // used for real distance
double interval = 0.2;
int num_parts = 10;
int num_exe = 1000;    // Release/execution frequency of each movement
double step;    // step = interval / num_exe;
double cur_time = 0;
std::vector<std::vector<motion> > sub_motion;
std::vector<std::vector<result> > collision_res;
std::string WRITE_TARGET_DIR = "/home/monica/Desktop/LOG_INFO/yumi_demo_data/visual_demo";
std::string MOTION_RESOURCES_DIR = "/home/monica/Desktop/LOG_INFO/motion_record";
std::string MOTION_NAME = "motion4";

/// for this demo
int cur_Action = 0; // current action
int cur_Motion = 0;
std::vector<moveit_msgs::MoveGroupActionGoal> ActionGoal;   // save the data of Action, including start and final state of each motion

/// yumi_joint_1_l ,yumi_joint_2_l, yumi_joint_7_l, yumi_joint_3_l, yumi_joint_4_l, yumi_joint_5_l, yumi_joint_6_l
/// gripper_l_joint,         gripper_l_joint_m
/// (gripper_base->finger_r) (gripper_base->finger_l)
/// yumi_joint_1_r, yumi_joint_2_r, yumi_joint_7_r, yumi_joint_3_r, yumi_joint_4_r, yumi_joint_5_r, yumi_joint_6_r
/// gripper_r_joint, gripper_r_joint_m
sensor_msgs::JointState yumi_joint_state;

/// @brief variable of Transformation matrix for real distance, same as HM using Taylor Model
/// hm[i][j], i = 0 or 1, stands for left_arm and right_arm
/// hm[i][j], relative transformation from j-1 to j
Eigen::Matrix4d hm_world;
Eigen::Matrix4d hm[2][10];
Eigen::Matrix4d hm_hand[2];
Eigen::Matrix4d hm_leftfinger[2];
Eigen::Matrix4d hm_rightfinger[2];

fcl::BVHModel<fcl::SPHERE> finger;
fcl::BVHModel<fcl::SPHERE> hand;

fcl::FCL_REAL dis[4];

void init_hm(int joint_num);
void init_();
double realdis(sensor_msgs::JointState &joint_state, int joint_num);
void cal_distance_function(std::vector<double> &start_state, std::vector<double> &vel);
void cal_TM_distance(std_msgs::Float64 &TMpoly, std_msgs::Float64 &TMupper, std_msgs::Float64 &TMlower);
void callback(const moveit_msgs::MoveGroupActionGoal& msg);
void sample(const moveit_msgs::MoveGroupActionGoal& msg);
void ReadMotionFile(std::string filename);
bool collisioncallback(cInterval &T, result &collisionresult);
bool collisioncallbackthread(cInterval &T, result &collisionresult);

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
        infile.getline(buffer, 250);
        std::string str(buffer, buffer+strlen(buffer));
        char cstr[250];
        strcpy(cstr, str.c_str());

        char *p = strtok(cstr, ", ");
        while (p!=0) {
            joint_name.push_back(p);
            p = strtok(NULL, ", ");
        }
    }

    while (!infile.eof())
    {
        infile.getline(buffer, 240);
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

/// @brief init the translation matrix for real distance and frame from txt file and used for distance function
void init_()
{
    OBJ_RESOURCES_DIR = "/home/monica/moveit_ws/src/yumi_lerp_planner/yumi_description/meshes/gripper/coarse";

    std::string filename = MOTION_NAME + ".txt";
    ReadMotionFile(filename);

    ReadUrdfFile(file_l,0);
    ReadUrdfFile(file_r,1);

    std::string filenames = "base.obj";
    model_load(filenames, hand);
    filenames = "finger.obj";
    model_load(filenames, finger);

    int joint_num = 7;
    // init and setup the yaw
    init_yaw();

    // init hm
    init_hm(joint_num);

    for (int i = 0; i < ActionGoal.size(); i++) {
        vel.clear();
        sample(ActionGoal[i]);
    }

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

//    return sqrt(d(0) * d(0) + d(1) * d(1) + d(2) * d(2));
    return sqrt(d(0) * d(0) + d(1) * d(1) + d(2) * d(2)) - 0.026765925437017866 * 2;
}

void cal_distance_function(std::vector<double> &start_state, std::vector<double> &vel)
{
    std::vector<motion> subm;
    std::vector<result> res;
    double segment_interval = interval / num_parts;
    for (int i = 0; i < num_parts; i++) {
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

        struct timeval tStartTime, tEndTime;

        ROS_INFO("sub-interval %d in Action %d:", i+1, cur_Action+1);
        result tmp_res;

        gettimeofday(&tStartTime,NULL);
        collisioncallback(T, tmp_res);
        gettimeofday(&tEndTime,NULL);
        std::cout << "running time(one thread): " <<
                    tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000
                    << " s \n";

        gettimeofday(&tStartTime,NULL);
        collisioncallbackthread(T, tmp_res);
        gettimeofday(&tEndTime,NULL);
        std::cout << "running time(4 threads): " <<
                  tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000
                  << " s \n";

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
    setGlobalTimeInterval(T.i[0],T.i[1]);
    __T = T;
    __toc_err = 0.00001;
    struct timeval tStartTime, tEndTime;

//    fcl::BVHModel<fcl::SPHERE> hand;
//    fcl::BVHModel<fcl::SPHERE> finger;
//    std::string filenames = "base.obj";
//    model_load(filenames, hand);
//    filenames = "finger.obj";
//    model_load(filenames, finger);

//    link_type[0] = 1;
//    link_type[1] = 1;
//    clock_t gettimeofday(&tStartTime, NULL);
//    fcl::FCL_REAL dis = BVHTraversal(0, 0, hand, hand);
//    clock_t gettimeofday(&tEndTime, NULL);
////    ROS_INFO("check collision between left_gripper hand & right_gripper hand: \n");
//    if (dis < 0) {
//        collisionresult.num ++;
//        collisionresult.pairA.push_back("hand");
//        collisionresult.pairB.push_back("hand");
////        ROS_INFO("collide!\n");
//        ROS_INFO("Collision found between left_gripper hand & right_gripper hand.");
//
//    }
////    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);
//
//    link_type[0] = 1;
//    link_type[1] = 2;
//    gettimeofday(&tStartTime, NULL);
//    dis = BVHTraversal(0, 0, hand, finger);
//    gettimeofday(&tEndTime, NULL);
////    ROS_INFO("check collision between left_gripper hand & right_gripper leftfinger.");
//    if (dis < 0) {
//        collisionresult.num ++;
//        collisionresult.pairA.push_back("hand");
//        collisionresult.pairB.push_back("leftfinger");
////        ROS_INFO("collide!\n");
//        ROS_INFO("Collision found between left_gripper hand & right_gripper leftfinger: \n");
//    }
////    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);
//
//    link_type[0] = 1;
//    link_type[1] = 3;
//    gettimeofday(&tStartTime, NULL);
//    dis = BVHTraversal(0, 0, hand, finger);
//    gettimeofday(&tEndTime, NULL);
////    ROS_INFO("check collision between left_gripper hand & right_gripper rightfinger.");
//    if (dis < 0) {
//        collisionresult.num ++;
//        collisionresult.pairA.push_back("hand");
//        collisionresult.pairB.push_back("rightfinger");
////        ROS_INFO("collide!\n");
//        ROS_INFO("Collision found between left_gripper hand & right_gripper rightfinger: \n");
//    }
////    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);
//
//    link_type[0] = 2;
//    link_type[1] = 1;
//    gettimeofday(&tStartTime, NULL);
//    dis = BVHTraversal(0, 0, finger, hand);
//    gettimeofday(&tEndTime, NULL);
////    ROS_INFO("check collision between left_gripper leftfinger & right_gripper hand: \n");
//    if (dis < 0) {
//        collisionresult.num ++;
//        collisionresult.pairA.push_back("leftfinger");
//        collisionresult.pairB.push_back("hand");
////        ROS_INFO("collide!\n");
//        ROS_INFO("Collision found between left_gripper leftfinger & right_gripper hand.");
//    }
////    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);

    link_type[0] = 2;
    link_type[1] = 2;
    gettimeofday(&tStartTime,NULL);
    fcl::FCL_REAL dis = BVHTraversal(0, 0, finger, finger);
    gettimeofday(&tEndTime,NULL);
//    ROS_INFO("check collision between left_gripper leftfinger & right_gripper leftfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("leftfinger");
        collisionresult.pairB.push_back("leftfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper leftfinger & right_gripper leftfinger.");
    }
    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);

    link_type[0] = 2;
    link_type[1] = 3;
    gettimeofday(&tStartTime, NULL);
    dis = BVHTraversal(0, 0, finger, finger);
    gettimeofday(&tEndTime, NULL);
//    ROS_INFO("check collision between left_gripper leftfinger & right_gripper rightfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("leftfinger");
        collisionresult.pairB.push_back("rightfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper leftfinger & right_gripper rightfinger.");
    }
    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);

//    link_type[0] = 3;
//    link_type[1] = 1;
//    gettimeofday(&tStartTime, NULL);
//    dis = BVHTraversal(0, 0, finger, hand);
//    gettimeofday(&tEndTime, NULL);
////    ROS_INFO("check collision between left_gripper rightfinger & right_gripper hand: \n");
//    if (dis < 0) {
//        collisionresult.num ++;
//        collisionresult.pairA.push_back("rightfinger");
//        collisionresult.pairB.push_back("hand");
////        ROS_INFO("collide!\n");
//        ROS_INFO("Collision found between left_gripper rightfinger & right_gripper hand.");
//    }
////    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);

    link_type[0] = 3;
    link_type[1] = 2;
    gettimeofday(&tStartTime, NULL);
    dis = BVHTraversal(0, 0, finger, finger);
    gettimeofday(&tEndTime, NULL);
//    ROS_INFO("check collision between left_gripper rightfinger & right_gripper leftfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("rightfinger");
        collisionresult.pairB.push_back("leftfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper rightfinger & right_gripper leftfinger.");
    }
    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);

    link_type[0] = 3;
    link_type[1] = 3;
    gettimeofday(&tStartTime, NULL);
    dis = BVHTraversal(0, 0, finger, finger);
    gettimeofday(&tEndTime, NULL);
//    ROS_INFO("check collision between left_gripper rightfinger & right_gripper rightfinger: \n");
    if (dis < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("rightfinger");
        collisionresult.pairB.push_back("rightfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper rightfinger & right_gripper rightfinger.");
    }
    ROS_INFO("detection running time: %lf s.\n", tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000);
}

void BVHTraversalthread(const int& rootA, const int &rootB, const int &typeA, const int &typeB, const int& idx)
{
    fcl::BVHModel<fcl::SPHERE> &A = finger;
    fcl::BVHModel<fcl::SPHERE> &B = finger;

    fcl::BVNode<fcl::SPHERE> &bvA = A.getBV(rootA);
    fcl::BVNode<fcl::SPHERE> &bvB = B.getBV(rootB);

//    fcl::FCL_REAL dis[idx] = BVRSSCollide(bvA,bvB);
//    std::cout << "bv " << rootA << " vs bv " << rootB << ": \n";
    dis[idx] = BVSPHERECollide(bvA,bvB, typeA, typeB);
//    std::cout << "dis[idx]: " << dis[idx] << "\n";
//    std::cout << "r: " << bvA.bv.r << ", " << bvB.bv.r << "\n";
//    std::cout << "\n";

    if(dis[idx]<0)
    {
        if (bvA.isLeaf() || bvA.bv.r < __toc_err) {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // return
                return ;
            } else {
                // A, B.leftchild()
                // A, B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(rootA, bvB.leftChild(), A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);

                tmp = BVHTraversal(rootA, bvB.rightChild(), A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    dis[idx] = dis_[0];
                    return;
                }
            }
        } else {
            if (bvB.isLeaf() || bvB.bv.r < __toc_err) {
                // A.leftchild(), B
                // A.rightchild(), B
                std::vector<fcl::FCL_REAL> dis_;
                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), rootB, A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);

                tmp = BVHTraversal(bvA.rightChild(), rootB, A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    dis[idx] = dis_[0];
                    return;
                }
            } else {
                // A.leftchild(), B.leftchild()
                // A.leftchild(), B.rightchild()
                // A.rightchild(), B.leftchild()
                // A.rightchild(), B.rightchild()
                std::vector<fcl::FCL_REAL> dis_;

                fcl::FCL_REAL tmp = BVHTraversal(bvA.leftChild(), bvB.leftChild(), A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.leftChild(), bvB.rightChild(), A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.leftChild(), A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);
                tmp = BVHTraversal(bvA.rightChild(), bvB.rightChild(), A, B, typeA, typeB);
                if (tmp < 0) { dis[idx] = tmp; return; }
                else dis_.push_back(tmp);

                if (dis_.empty()) {
                    return;
                } else {
                    std::sort(dis_.begin(), dis_.begin()+dis_.size());
                    dis[idx] = dis_[0];
                    return;
                }
            }
        }
    }
    else return;
}

bool collisioncallbackthread(cInterval &T, result &collisionresult)
{
    setGlobalTimeInterval(T.i[0],T.i[1]);
    __T = T;
    __toc_err = 0.00001;

//    fcl::BVHModel<fcl::SPHERE> hand;
//    fcl::BVHModel<fcl::SPHERE> finger;
//    std::string filenames = "base.obj";
//    model_load(filenames, hand);
//    filenames = "finger.obj";
//    model_load(filenames, finger);

    std::thread t0,t1,t2,t3;

    t0 = std::thread(BVHTraversalthread, 0, 0, 2, 2, 0);
    t1 = std::thread(BVHTraversalthread, 0, 0, 2, 3, 1);
    t2 = std::thread(BVHTraversalthread, 0, 0, 3, 2, 2);
    t3 = std::thread(BVHTraversalthread, 0, 0, 3, 3, 3);

    t0.join();
    t1.join();
    t2.join();
    t3.join();

    if (dis[0] < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("leftfinger");
        collisionresult.pairB.push_back("leftfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper leftfinger & right_gripper leftfinger.");
    }

    if (dis[1] < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("leftfinger");
        collisionresult.pairB.push_back("rightfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper leftfinger & right_gripper rightfinger.");
    }

    if (dis[2] < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("rightfinger");
        collisionresult.pairB.push_back("leftfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper rightfinger & right_gripper leftfinger.");
    }

    if (dis[3] < 0) {
        collisionresult.num ++;
        collisionresult.pairA.push_back("rightfinger");
        collisionresult.pairB.push_back("rightfinger");
//        ROS_INFO("collide!\n");
        ROS_INFO("Collision found between left_gripper rightfinger & right_gripper rightfinger.");
    }
}

int main(int argc, char **argv)
{
//    time_t t=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
//    std::stringstream ss;
//    ss<<std::put_time(std::localtime(&t),"%F %X");
//    std::string outfile = WRITE_TARGET_DIR + "/" + MOTION_NAME + "/" + ss.str() + ".txt";
//    std::ofstream out(outfile);

    //Initiate ROS
    ros::init(argc, argv, "listenandplot");
    ros::NodeHandle n;

    /// rosparam load
//    ros::param::set("/source_list", "[/move_group/fake_controller_joint_states]");

    init_();

    ros::Publisher dis_pub = n.advertise<std_msgs::Float64>("/plot/dis", 1);
    ros::Publisher TMdis_poly_pub = n.advertise<std_msgs::Float64>("/plot/TM_poly", 1);
    ros::Publisher TMdis_upper_pub = n.advertise<std_msgs::Float64>("/plot/TM_upper", 1);
    ros::Publisher TMdis_lower_pub = n.advertise<std_msgs::Float64>("/plot/TM_lower", 1);
    ros::Publisher diff_pub = n.advertise<std_msgs::Float64>("/plot/diff", 1);
    ros::Publisher yumi_joint_state_publish = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    ros::Rate loop_rate(500);
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
                dis.data = realdis(yumi_joint_state, 7);
//                ROS_INFO("real distance: %lf", dis.data);

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
//                out << cur_time << " " << dis.data << " " << TMpoly.data << " " << TMlower.data << " " << TMupper.data << " " << dis.data - TMlower.data << "\n";

                for (int k = 0; k < yumi_joint_state.position.size(); k++) {
                    yumi_joint_state.position[k] += sub_motion[cur_Action][cur_Motion].velocity[k] * step;
//                    ROS_INFO("%s: position: %lf", yumi_joint_state.name[k].data(), yumi_joint_state.position[k]);
                }
                cur_time += step;
            } else {
//                ROS_INFO("The motion has been completed and remains in the goal state.");
                if (cur_Action < ActionGoal.size() - 1) {
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
//                    out.close();
                }
            }

            yumi_joint_state_publish.publish(yumi_joint_state);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
