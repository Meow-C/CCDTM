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

#include <iostream>

bool flag = false;
sensor_msgs::JointState yumi_joint_state;
std::vector<double> vel;
double interval = 0.5;
int num_exe = 1000;    // step = interval / num_exe;
double step;
double cur_time;
// yumi_joint_1_l ,yumi_joint_2_l, yumi_joint_7_l, yumi_joint_3_l, yumi_joint_4_l, yumi_joint_5_l, yumi_joint_6_l
// gripper_l_joint,         gripper_l_joint_m
// (gripper_base->finger_r) (gripper_base->finger_l)
// yumi_joint_1_r, yumi_joint_2_r, yumi_joint_7_r, yumi_joint_3_r, yumi_joint_4_r, yumi_joint_5_r, yumi_joint_6_r
// gripper_r_joint, gripper_r_joint_m

void callback(const moveit_msgs::MoveGroupActionGoal& msg)
{
    actionlib_msgs::GoalID_<std::allocator<void> > goal_id = msg.goal_id;
    ROS_INFO("I heard %s", goal_id.id.data());
    moveit_msgs::RobotState start_state = msg.goal.request.start_state;
    int joint_num = start_state.joint_state.name.size();
    ROS_INFO("start_state:");
    for (int i = 0; i < joint_num; i++){
        ROS_INFO("%s : position: %f", msg.goal.request.start_state.joint_state.name.at(i).data(), msg.goal.request.start_state.joint_state.position[i]);
    }
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
        for (int i = 0; i < group_joint_num; i++)
            yumi_goal_state.push_back(req_constraints[i].position);
        for (int i = group_joint_num; i < joint_num; i++)
            yumi_goal_state.push_back(0);
    }
    else {
        ROS_INFO("The current motion planning group is right_arm.");
        for (int i = 0; i < num; i++)
            yumi_goal_state.push_back(0);
        for (int i = 0; i < group_joint_num; i++)
            yumi_goal_state.push_back(req_constraints[i].position);
        for (int i = num + group_joint_num; i < joint_num; i++)
            yumi_goal_state.push_back(0);
    }
    for (int i = 0; i < joint_num; i++) {
        vel.push_back((yumi_goal_state[i] - start_state.joint_state.position[i]) / interval);
    }
    yumi_joint_state.header.stamp = ros::Time::now();
    for (int i = 0; i < joint_num; i++) {
        yumi_joint_state.name.push_back(start_state.joint_state.name[i].data());
        yumi_joint_state.position.push_back(start_state.joint_state.position[i]);
    }

    step = interval / num_exe;
    cur_time = step;
    flag = true;
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "listenandplot");
    ros::NodeHandle n;

    /// rosparam load, TODO
//    ros::param::set("/source_list", "[/move_group/fake_controller_joint_states]");

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
//            ROS_INFO("publish distance using Taylor Model.");
            /// calculate the value of joint_state that needs to be published
            TMpoly.data = 0.01;
            TMupper.data = TMpoly.data + 0.1;
            TMlower.data = TMpoly.data - 0.1;
            TMdis_poly_pub.publish(TMpoly);
            TMdis_upper_pub.publish(TMupper);
            TMdis_lower_pub.publish(TMlower);

//            ROS_INFO("publish joint_state of motion during time interval [0, %lf].", interval);

            if (cur_time <= interval) {
                ROS_INFO("current time: %lf", cur_time);
                for (int k = 0; k < yumi_joint_state.position.size(); k++) {
                    yumi_joint_state.position[k] += vel[k] * step;
//                    ROS_INFO("%s: position: %lf", yumi_joint_state.name[k].data(), yumi_joint_state.position[k]);
                }
                cur_time += step;
            } else {
                ROS_INFO("The motion has been completed and remains in the goal state.");
            }

            yumi_joint_state_publish.publish(yumi_joint_state);

            // calculate the real distance from the tf
            // ROS_INFO("publish real distance.");
            tf::StampedTransform transform;
            geometry_msgs::TransformStamped odom;

            try {
                tf_listener.waitForTransform("gripper_l_finger_l", "gripper_r_finger_l", ros::Time(0), ros::Duration(10.0) );
                tf_listener.lookupTransform("gripper_l_finger_l", "gripper_r_finger_l", ros::Time(0), transform);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            tf::Vector3 d = transform.getOrigin();
            dis.data = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

            ROS_INFO("real distance: %lf", dis.data);
            dis_pub.publish(dis);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}