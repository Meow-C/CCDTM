//
// Created by monica on 2021/7/27.
//

#ifndef SRC_MOTION_AM_H
#define SRC_MOTION_AM_H

#include "fcl/math/transform.h"
#include "fcl/ccd/taylor_matrix.h"
#include "fcl/ccd/taylor_vector.h"
#include "fcl/BV/RSS.h"
#include <Eigen/Core>

namespace fcl{

class MotionAM;

class MotionAM
{
public:
    MotionAM() {}
    MotionAM(double time[2],
             std::vector<double>& joint1_, double finger_joint1_,
             std::vector<double>& joint2_, double finger_joint2_,
             Eigen::Matrix4d& world_, std::vector<Eigen::Matrix4d>& link_,
             Eigen::Matrix4d& hand_, Eigen::Matrix4d& finger_, Eigen::Matrix4d& effector_)
    {
        time_interval[0] = time[0];
        time_interval[1] = time[1];
        joint1 = joint1_;
        finger_joint1 = finger_joint1_;
        joint2 = joint2_;
        finger_joint2 = finger_joint2_;
        world = world_;
        link = link_;
        hand = hand_;
        finger = finger_;
        effector = effector_;
        computeVelocity();
    }
    ~MotionAM() {}

    bool integrate(double dt) const
    {
        if (dt < time_interval[0]) dt = time_interval[0];
        if (dt > time_interval[1]) dt = time_interval[1];

        Eigen::Matrix4d cur_hm = world;
        for (size_t i = 0; i < joint1.size(); i++) {
            Eigen::Matrix4d tmp;
            double theta = joint_vel[i] * dt + joint1[i];
            tmp << cos(theta), -sin(theta), 0, 0,
                   sin(theta), cos(theta), 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;
            cur_hm = cur_hm * link[i] * tmp;
        }
        Eigen::Matrix4d tmp;
        tmp << 1, 0, 0, -1 * (joint_finger_vel * dt + finger_joint1),
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
        cur_hm = cur_hm * hand * finger * tmp;
        Matrix3f R(cur_hm(0,0), cur_hm(0,1),cur_hm(0,2),
                   cur_hm(1,0), cur_hm(1,1),cur_hm(1,2),
                   cur_hm(2,0), cur_hm(2,1),cur_hm(2,2));
        Vec3f T(cur_hm(0,3), cur_hm(1,3), cur_hm(2,3));
        tf.setTransform(R,T);

        return true;
    }

    /// @brief Get the rotation and translation in current step
    void getCurrentTransform(Transform3f& tf_) const
    {
        tf_ = tf;
    }

    void getTF0(Transform3f& tf_) const
    {
        tf_ = tf0;
    }

    void getTF1(Transform3f& tf_) const
    {
        tf_ = tf1;
    }

    void setTimeInterval(double t0, double t1)
    {
        time_interval[0] = t0;
        time_interval[1] = t1;
    }

    void computetf0() const
    {
        double dt = time_interval[0];

        Eigen::Matrix4d cur_hm = world;
        for (size_t i = 0; i < joint1.size(); i++) {
            Eigen::Matrix4d tmp;
            double theta = joint_vel[i] * dt + joint1[i];
            tmp << cos(theta), -sin(theta), 0, 0,
                    sin(theta), cos(theta), 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
            cur_hm = cur_hm * link[i] * tmp;
        }
        Eigen::Matrix4d tmp;
        tmp << 1, 0, 0, -1 * (joint_finger_vel * dt + finger_joint1),
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        cur_hm = cur_hm * hand * finger * tmp;
        Matrix3f R(cur_hm(0,0), cur_hm(0,1),cur_hm(0,2),
                   cur_hm(1,0), cur_hm(1,1),cur_hm(1,2),
                   cur_hm(2,0), cur_hm(2,1),cur_hm(2,2));
        Vec3f T(cur_hm(0,3), cur_hm(1,3), cur_hm(2,3));
        tf0.setTransform(R,T);
    }

    void computetf1() const
    {
        double dt = time_interval[1];

        Eigen::Matrix4d cur_hm = world;
        for (size_t i = 0; i < joint1.size(); i++) {
            Eigen::Matrix4d tmp;
            double theta = joint_vel[i] * dt + joint1[i];
            tmp << cos(theta), -sin(theta), 0, 0,
                    sin(theta), cos(theta), 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
            cur_hm = cur_hm * link[i] * tmp;
        }
        Eigen::Matrix4d tmp;
        tmp << 1, 0, 0, -1 * (joint_finger_vel * dt + finger_joint1),
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        cur_hm = cur_hm * hand * finger * tmp;
        Matrix3f R(cur_hm(0,0), cur_hm(0,1),cur_hm(0,2),
                   cur_hm(1,0), cur_hm(1,1),cur_hm(1,2),
                   cur_hm(2,0), cur_hm(2,1),cur_hm(2,2));
        Vec3f T(cur_hm(0,3), cur_hm(1,3), cur_hm(2,3));
        tf1.setTransform(R,T);
    }

protected:
    std::vector<double> joint1;
    std::vector<double> joint2;
    double finger_joint1;
    double finger_joint2;

    Eigen::Matrix4d world;
    std::vector<Eigen::Matrix4d> link;
    Eigen::Matrix4d hand;
    Eigen::Matrix4d finger;
    Eigen::Matrix4d effector;

    double time_interval[2];
    std::vector<double> joint_vel;
    double joint_finger_vel;

    /// @brief The transformation at current time t
    mutable Transform3f tf;

    mutable Transform3f tf0;
    mutable Transform3f tf1;

    void computeVelocity()
    {
        double interval = time_interval[1] - time_interval[0];
        for (size_t i = 0; i < joint1.size(); i++) {
            joint_vel.push_back((joint2[i]-joint1[i])/interval);
        }
        joint_finger_vel = (finger_joint2-finger_joint1)/interval;
    }

};

typedef std::shared_ptr<MotionAM> MotionAMPtr;

}

#endif //SRC_MOTION_AM_H
