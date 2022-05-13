#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/MoveGroupActionGoal.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "ModelParameters.h"
#include "Taylor_Function.h"
#include "cInterval.h"

#include <iostream>

bool flag = false;
std::vector<double> vel;
double interval = 0.5;
int num_exe = 100;    // step = interval / num_exe;
double step;
double cur_time;
cTaylorModel3 dis_square;

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

void init_()
{
    // init for TM
    cInterval T(0, interval);
    setGlobalTimeInterval(T.i[0],T.i[1]);
    double t = T.i[1]-T.i[0];

    ReadUrdfFile(file_l,0);
    ReadUrdfFile(file_r,1);

    int joint_num = 7;
    init_yaw(t);
    // init hm
    init_hm(joint_num);
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

    hm0 = hm0 * hm_hand[0] * hm_leftfinger[0];
    hm1 = hm1 * hm_hand[1] * hm_rightfinger[1];

    Eigen::Vector4d p,q;
    p << 0,0,0,1;
    q << 0,0,0,1;

    p = hm0 * p;
    q = hm1 * q;

//    std::cout << "\npoint p: \n";
//    std::cout << p;
//    std::cout << "\npoint q: \n";
//    std::cout << q;

    Eigen::Vector4d d = p - q;

    return sqrt(d(0) * d(0) + d(1) * d(1) + d(2) * d(2));
}

void callback(const moveit_msgs::MoveGroupActionGoal& msg)
{
    actionlib_msgs::GoalID_<std::allocator<void> > goal_id = msg.goal_id;
    ROS_INFO("I heard %s", goal_id.id.data());

    /// get the start state from MotionPlannerActionGoal
    moveit_msgs::RobotState start_state = msg.goal.request.start_state;
    int joint_num = start_state.joint_state.name.size();
    ROS_INFO("start_state:");
    for (int i = 0; i < joint_num; i++){
        ROS_INFO("%s : position: %f", msg.goal.request.start_state.joint_state.name.at(i).data(), msg.goal.request.start_state.joint_state.position[i]);
    }

    /// get the motion arm group, only one group, left arm or right arm
    ROS_INFO("MotionPlanRequest:");
    moveit_msgs::MotionPlanRequest req = msg.goal.request;
    ROS_INFO("Group Name: %s", req.group_name.data());
    std::vector<moveit_msgs::JointConstraint> req_constraints = req.goal_constraints[0].joint_constraints;
    int group_joint_num = req_constraints.size();
    for (int i = 0; i < group_joint_num; i++) {
        ROS_INFO("%s: position: %lf, tolerance: (%lf, %lf)", req_constraints[i].joint_name.data(), req_constraints[i].position,
                 -1.0 * req_constraints[i].tolerance_below, req_constraints[i].tolerance_above);
    }

    /// The above is only to display the information in msg having no practical meaning, and can be annotated
    /// @brief According to the msg, reproduce the uniform velocity motion, and calculate the distance function
    /// using Taylor Model.
    /// Publish the joint_state
    std::vector<double> yumi_goal_state;
    int num = 0.5 * joint_num;
    if (req.group_name.compare(0, 8, "left_arm", 0, 8) == 0) {
        ROS_INFO("The current motion planning group is left_arm.");
        /// for left arm [req[0]..req[i], 0..0]
        for (int i = 0; i < group_joint_num; i++)
        {
            yumi_goal_state.push_back(req_constraints[i].position);
        }
        for (int i = group_joint_num; i < joint_num; i++)
            yumi_goal_state.push_back(start_state.joint_state.position[i]);
    }
    else {
        ROS_INFO("The current motion planning group is right_arm.");
        /// for right arm [0..0, req[0]..req[i], 0..0]
        for (int i = 0; i < num; i++)
            yumi_goal_state.push_back(start_state.joint_state.position[i]);
        for (int i = 0; i < group_joint_num; i++)
        {
            yumi_goal_state.push_back(req_constraints[i].position);
        }
        for (int i = num + group_joint_num; i < joint_num; i++)
            yumi_goal_state.push_back(start_state.joint_state.position[i]);
    }
    for (int i = 0; i < joint_num; i++) {
        vel.push_back((yumi_goal_state[i] - start_state.joint_state.position[i]) / interval);
        // init for TM
//        ROS_INFO("frame[i][j]: yaw0, yaw1, yaw_omega.");
        if (i < joint_num/2) {
            frame[0][i].yaw0 = start_state.joint_state.position[i];
            frame[0][i].yaw1 = yumi_goal_state[i];
            frame[0][i].yaw_omega = (frame[0][i].yaw1 - frame[0][i].yaw0) / interval;
//            ROS_INFO("frame[0][%d]: %lf, %lf, %lf", i, frame[0][i].yaw0, frame[0][i].yaw1, frame[0][i].yaw_omega);
        }
        else {
            frame[1][i - joint_num/2].yaw0 = start_state.joint_state.position[i];
            frame[1][i - joint_num/2].yaw1 = yumi_goal_state[i];
            frame[1][i - joint_num/2].yaw_omega = (frame[1][i - joint_num/2].yaw1 - frame[1][i - joint_num/2].yaw0) / interval;
//            ROS_INFO("frame[1][%d]: %lf, %lf, %lf", i - joint_num/2, frame[1][i - joint_num/2].yaw0,
//                     frame[1][i - joint_num/2].yaw1, frame[1][i - joint_num/2].yaw_omega);
        }
        // init for TM end
    }

    /// init the joint_state which will be published later.
    yumi_joint_state.header.stamp = ros::Time::now();
    for (int i = 0; i < joint_num; i++) {
        yumi_joint_state.name.push_back(start_state.joint_state.name[i].data());
        yumi_joint_state.position.push_back(start_state.joint_state.position[i]);
    }

    cInterval T(0, interval);
    setGlobalTimeInterval(T.i[0],T.i[1]);
    setupdualHM(7, T);   // init for TM

    /// choose two points or two sphere, calculate the distance between two points
    cTMVector4 p(0,0,0,1);
    cTMVector4 q(0,0,0,1);
    p = M44multV14(HMleftfinger[0], p);
    q = M44multV14(HMrightfinger[1], q);
    cTaylorModel3 x = p.P[0] - q.P[0];
    cTaylorModel3 y = p.P[1] - q.P[1];
    cTaylorModel3 z = p.P[2] - q.P[2];
    cTM3Vector3 pq(x, y, z);
    dis_square = pq.dot(pq);

    step = interval / num_exe;
    cur_time = 0;
    flag = true;
}

void test()
{
    // test Eigen Matrix4d
    Eigen::Matrix4d one, H;
    one << 1,0,0,0,
           0,1,0,0,
           0,0,1,0,
           0,0,0,1;
    H << 1,2,3,4,
         5,6,7,8,
         9,10,11,12,
         13,14,15,16;
    Eigen::Matrix4d tmp;
    tmp << -8, -7, -6, -5,
           -4, -3, -2, -1,
           0, 1, 2, 3,
           4, 5, 6, 7;
    std::cout << H * tmp;
    Eigen::Matrix4d res = H * tmp;
    std::cout << "\n\nm(0,3): \n" << res(0,3);
    std::cout << "\n\nm(3,3): \n" << res(3,3);
    Eigen::Vector4d p;
    p << 0,0,0,1;
    Eigen::Vector4d q = H * p;
    std::cout << "\n\n";
    std::cout << q;
}

void testTM() {
    setGlobalTimeInterval(0,CTM3_HALF_PI);
    cTaylorModel3 a1, b1, c1;
    a1.cosModel(1,CTM3_HALF_PI);
    b1.sinModel(1,CTM3_HALF_PI);
    c1 = a1 * b1;
    c1.print();

    setGlobalTimeInterval(CTM3_HALF_PI,CTM3_PI);
    cTaylorModel3 a2, b2, c2;
    a2.cosModel(1,0);
    b2.sinModel(1,0);
    c2 = a2 * b2;
    c2.print();
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "listenandplot");
    ros::NodeHandle n;

    /// rosparam load, TODO
//    ros::param::set("/source_list", "[/move_group/fake_controller_joint_states]");

    init_();

    ROS_INFO("begin: please drag the interactive marker and plan");

    ros::Publisher dis_pub = n.advertise<std_msgs::Float64>("/plot/dis", 1);
    ros::Publisher TMdis_poly_pub = n.advertise<std_msgs::Float64>("/plot/TM_poly", 1);
    ros::Publisher TMdis_upper_pub = n.advertise<std_msgs::Float64>("/plot/TM_upper", 1);
    ros::Publisher TMdis_lower_pub = n.advertise<std_msgs::Float64>("/plot/TM_lower", 1);
    ros::Publisher yumi_joint_state_publish = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    tf::TransformListener tf_listener;
    ros::Subscriber goal_sub = n.subscribe("/move_group/goal", 1000, callback);


    ros::Rate loop_rate(10);
    ros::spinOnce();

    std_msgs::Float64 dis, TMpoly, TMupper, TMlower;
    dis.data = 0.0;

    while (ros::ok()) {
        if (flag) {
//            ROS_INFO("publish joint_state of motion during time interval [0, %lf].", interval);

            if (cur_time < interval + step) {
                ROS_INFO("current time: %lf", cur_time);
                for (int k = 0; k < yumi_joint_state.position.size(); k++) {
                    yumi_joint_state.position[k] += vel[k] * step;
//                    ROS_INFO("%s: position: %lf", yumi_joint_state.name[k].data(), yumi_joint_state.position[k]);
                }
//                ROS_INFO("[%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf \n , %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf]",
//                         yumi_joint_state.position[0], yumi_joint_state.position[1], yumi_joint_state.position[2],
//                         yumi_joint_state.position[3], yumi_joint_state.position[4], yumi_joint_state.position[5],
//                         yumi_joint_state.position[6], yumi_joint_state.position[7], yumi_joint_state.position[8],
//                         yumi_joint_state.position[9], yumi_joint_state.position[10], yumi_joint_state.position[11],
//                         yumi_joint_state.position[12], yumi_joint_state.position[13], yumi_joint_state.position[14],
//                         yumi_joint_state.position[15], yumi_joint_state.position[16], yumi_joint_state.position[17]);

                /// TODO, from urdf2casadi/txt or get from the tf
                dis.data = realdis(yumi_joint_state, 7);
                ROS_INFO("real distance: %lf", dis.data);

                /// calculate the real distance from the urdf and joint_state
                cInterval disTM_cur = dis_square.evaluate(cur_time);

                TMpoly.data = dis_square.c[0] + dis_square.c[1] * cur_time + dis_square.c[2] * cur_time * cur_time
                              + dis_square.c[3] * cur_time * cur_time * cur_time;
                if (TMpoly.data < 0) {
                    TMpoly.data = -1.0 * sqrt(-1 * TMpoly.data);
                }
                else TMpoly.data = sqrt(TMpoly.data);

                TMlower.data = disTM_cur.i[0];
                if (TMlower.data < 0) {
                    TMlower.data = -1.0 * sqrt(-1 * TMlower.data);
                }
                else TMlower.data = sqrt(TMlower.data);
                TMupper.data = disTM_cur.i[1];
                if (TMupper.data < 0) {
                    TMupper.data = -1.0 * sqrt(-1 * TMupper.data);
                }
                else TMupper.data = sqrt(TMupper.data);
                ROS_INFO("TM poly: %lf", TMpoly);
                ROS_INFO("TM distance bound: [%lf, %lf]", TMlower, TMupper);

                cur_time += step;
                dis_pub.publish(dis);
                TMdis_poly_pub.publish(TMpoly);
                TMdis_lower_pub.publish(TMlower);
                TMdis_upper_pub.publish(TMupper);
            } else {
                ROS_INFO("The motion has been completed and remains in the goal state.");
            }

            yumi_joint_state_publish.publish(yumi_joint_state);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


/// mark: 2021/04/23 13:51 version: TM_dis OK, real_dis: TODO
/// mark: 2021/04/26 19:36 version: TM_dis and real_dis OK, divide motion into n parts: TODO