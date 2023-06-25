#include <cmath>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// #include <ur_kinematic/generic_float.h>
#include <ros_impedance_controller/generic_float.h>

#include "parameters.h"
#include "utils/direct.cpp"
#include "utils/inverse.cpp"
#include "utils/jacobian.cpp"

#define PI 3.14159265

/**
 * if true  -> soft gripper
 * if false -> hard gripper
*/
#define soft_gripper false

/// Number of joints of the robot
#define ROBOT_JOINTS 6
/// Number of joints of the hard gripper
#define EE_HARD_JOINTS 3

Eigen::VectorXd currentJoint(ROBOT_JOINTS);
Eigen::Vector3d gripper_joint(EE_HARD_JOINTS);
bool DEBUG = false;

void move(eepos);
void move_2(eepos);
void move_3(eepos);

Eigen::VectorXd sistema_angoli(Eigen::VectorXd joint)
{
    if (joint.rows() == 6)
    {
        while (joint(0) >= 2 * PI)
        {
            // std::cout << "1" << std::endl;
            joint(0) -= 2 * PI;
        }
        while (joint(0) <= -2 * PI)
        {
            // std::cout << "2" << std::endl;
            joint(0) += 2 * PI;
        }
        while (joint(1) >= 0)
        {
            // std::cout << "3 - " << joint(1) << std::endl;
            joint(1) -= PI;
        }
        while (joint(1) <= -PI)
        {
            // std::cout << "4" << std::endl;
            joint(1) += PI;
        }
        while (joint(2) >= 0)
        {
            // std::cout << "5" << std::endl;
            joint(2) -= PI;
        }
        while (joint(2) <= -PI)
        {
            // std::cout << "6" << std::endl;
            joint(2) += PI;
        }
        while (joint(3) >= 2 * PI)
        {
            // std::cout << "7" << std::endl;
            joint(3) -= 2 * PI;
        }
        while (joint(3) <= -2 * PI)
        {
            // std::cout << "8" << std::endl;
            joint(3) += 2 * PI;
        }
        while (joint(4) >= 2 * PI)
        {
            // std::cout << "9" << std::endl;
            joint(4) -= 2 * PI;
        }
        while (joint(4) <= -2 * PI)
        {
            // std::cout << "10" << std::endl;
            joint(4) += 2 * PI;
        }
        while (joint(5) >= 2 * PI)
        {
            // std::cout << "11" << std::endl;
            joint(5) -= 2 * PI;
        }
        while (joint(5) <= -2 * PI)
        {
            // std::cout << "12" << std::endl;
            joint(5) += 2 * PI;
        }
    }
    return joint;
}

void send_joint(Eigen::VectorXd joints_value)
{
    if (joints_value.rows() == 6)
    {
        std_msgs::Float64MultiArray msg;
        msg.data.resize(ROBOT_JOINTS + EE_HARD_JOINTS);
        std::cout << "send: ";
        for (int i = 0; i < ROBOT_JOINTS; i++)
        {
            msg.data[i] = (float)joints_value[i];
            std::cout << msg.data[i] << " ";
        }
        
        msg.data[ROBOT_JOINTS + 0] = gripper_joint(0);
        msg.data[ROBOT_JOINTS + 1] = gripper_joint(1);
        if (!soft_gripper) {
            msg.data[ROBOT_JOINTS + 2] = gripper_joint(2);
        }

        std::cout << std::endl;
        pub_joint.publish(msg);
    }
}

void plan(eepos end_pose, ros::Rate rate)
{
    eepos start_pose;
    start_pose = direct_kinematic_eepos(currentJoint);
    move_3(end_pose);
    int n;
    cin >> n;
    move_3(start_pose);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ur_prova");
    ros::NodeHandle n;
    ros::Rate rate(1000);
    joint_pos << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    currentJoint << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    gripper_joint << 1.8, 1.8, 1.8;

    pub_joint = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::ServiceClient gripperClient = n.serviceClient<ros_impedance_controller::generic_float>("move_gripper");

    // send_joint(currentJoint);

    // // PROVA DIRECT KINEMATIC
    // Eigen::VectorXd th(6);
    // //th << -0.32,-0.78, -2.56,-1.63, -1.57, 3.49;
    // th << 0.159299, -0.411665, -1.98958, -2.3035, -1.28793, 4.55308;
    // eepos dir = direct_kinematic_eepos(th);
    // std::cout << dir.ee_pos << std::endl;
    // std::cout << dir.ee_rotm << std::endl;

    // // // PROVA INVERSE KIN
    // Eigen::Vector3d pos;
    // Eigen::Matrix3d rot;
    // pos << 0.0174999, -0.12265, 0.733;
    // rot << 1, 0, 0,
    //     0, 1, 0,
    //     -0, 0, 1;
    // Eigen::MatrixXd tmp = inverse_kinematic(pos, rot);
    // std::cout << tmp << std::endl;

    // Eigen::MatrixXd direct;
    // for (int i = 0; i < tmp.rows(); i++)
    // {
    //     cout << "------------------" << endl;
    //     // cout << tmp.row(i) << endl;
    //     direct = direct_kinematic(tmp.row(i));
    //     std::cout << direct << std::endl;
    // }

    // // Eigen::VectorXd tmp(6);
    // tmp << 0.00392766, 0.0524709, -2.59869, -2.18202, -2.41544, 4.70847;
    // std::cout << direct_kinematic(tmp) << std::endl;

    // currentJoint << -0.604099, -1.7731, -1.84075, 2.04305, 1.5708, 2.1749;
    // eepos end_pose;
    // end_pose.ee_pos << -0.2, 0.25, 0.5;
    // end_pose.ee_eul << 0, 0, 0;
    // end_pose.ee_rotm = eul2rotm(end_pose.ee_eul);

    // Eigen::MatrixXd end_joint = inverse_kinematic(end_pose);
    // // std::cout << end_joint.rows() << " - " << end_joint.cols() << std::endl;
    // for (int i = 0; i < end_joint.rows(); i++)
    // {
    //     std::cout << (currentJoint.transpose() - end_joint.row(i)).norm() << std::endl;
    // }

    // // PROVA EUL2ROMT
    // Eigen::Vector3d eul;
    // eul << 0, 0, 0;
    // std::cout << eul2rotm(eul) << std::endl;

    // PROVIAMO A SPOSTARE IL ROBOT DALLA POSIZIONE INIZIALE ALLA POSIZIONE FINALE DELL BLOCCO 112
    // LA POSIZIONE FINALE È (0.9, 0.25, 0.9)

    // currentJoint << -1.30093, -1.91718, -2.1435, -0.651708, -1.5708, 6.01332;
    // currentJoint << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    //
    // eepos end_pose;
    //
    // end_pose.ee_pos << 0.5, 0.5, 0.9;
    // end_pose.ee_eul << 0, 0, 0;
    // end_pose.ee_rotm = eul2rotm(end_pose.ee_eul);
    //
    // plan(end_pose, rate);

    // PROVA CHIUSURA DEL 3 DITA

    eepos start_pose;
    start_pose.ee_pos << 0.00829988, -0.13265, 0.720;
    // target.ee_pos(1) += (0.031 / 2);
    start_pose.ee_eul = Eigen::Vector3d::Zero();

    eepos move1;
    move1 = start_pose;
    move1.ee_pos(1) -= (0.031 / 2);
    move1.ee_pos(2) -= 0.2;

    move_2(move1);

    eepos move2;
    move2 = start_pose;
    move2.ee_pos(1) -= (0.031 / 2);
    move2.ee_pos(2) -= 0.035;

    move_2(move2);

    float diameter = 35;
    for (int i = 0; i < 5; i++)
    {
        float alpha = (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
        std::cout << "alpha : " << alpha << std::endl;
        gripper_joint = Vector3d::Ones(3) * alpha;

        send_joint(currentJoint);

        rate.sleep();
    }

    eepos move3;
    move3 = start_pose;
    move3.ee_pos(1) -= 0.031;
    move3.ee_pos(2) -= 0.2;

    move_2(move3);

    eepos target;
    target.ee_pos << 0.4, -0.20, 0.720;
    target.ee_eul = Eigen::Vector3d::Zero();

    eepos move4;
    move4 = target;
    move4.ee_pos(2) -= 0.2;

    move_2(move4);

    eepos move5;
    move5 = target;
    move5.ee_pos(2) -= 0.035;

    move_2(move5);

    diameter = 100;
    for (int i = 0; i < 2; i++)
    {
        float alpha = (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
        std::cout << "alpha : " << alpha << std::endl;
        gripper_joint = Vector3d::Ones(3) * alpha;

        send_joint(currentJoint);

        rate.sleep();
    }

    eepos move6;
    move6 = target;
    move6.ee_pos(2) -= 0.2;

    move_2(move6);

    ros::shutdown();

    return 0;
}

void move(eepos end_pose)
{
    Eigen::VectorXd endJoint = sistema_angoli(inverse_kinematic(end_pose.ee_pos, end_pose.ee_rotm).row(6));

    double dt = 0.001, MOVEMENT_VELOCITY = 0.005;

    eepos start_pose = direct_kinematic_eepos(currentJoint);
    Eigen::Vector3d delta = end_pose.ee_pos - start_pose.ee_pos;

    double velo = delta.norm() / MOVEMENT_VELOCITY;

    Eigen::Vector3d xyz_tempo;
    Eigen::VectorXd joint_tempo;

    // std::ofstream outfile;
    // outfile.open("joint.log");

    for (double i = dt; i <= velo; i += dt)
    {
        xyz_tempo = start_pose.ee_pos + delta * i / velo;
        joint_tempo = sistema_angoli(inverse_kinematic(xyz_tempo, end_pose.ee_rotm).row(6));
        // std::cout << xyz_tempo.transpose() << std::endl;
        if (DEBUG)
        {
            std::cout << joint_tempo << std::endl;
        }
        // outfile << joint_tempo.transpose() << "\n";
        send_joint(joint_tempo);
    }
    // outfile.close();
}

Eigen::Vector3d computeOrientationError(Eigen::Matrix3d re, Eigen::Matrix3d wRd)
{
    Eigen::Matrix3d relative_orientation;
    relative_orientation = re.transpose() * wRd;

    double cos_theta = (relative_orientation(0, 0) + relative_orientation(1, 1) + relative_orientation(2, 2) - 1) / 2;

    Eigen::MatrixXd tmp(3, 2);
    tmp << relative_orientation(2, 1), -relative_orientation(1, 2),
        relative_orientation(0, 2), -relative_orientation(2, 0),
        relative_orientation(1, 0), -relative_orientation(0, 1);

    double sin_theta = tmp.norm() / 2;

    double theta = atan2(sin_theta, cos_theta);

    if (theta == 0)
    {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d aux;
    aux << relative_orientation(2, 1) - relative_orientation(1, 2),
        relative_orientation(0, 2) - relative_orientation(2, 0),
        relative_orientation(1, 0) - relative_orientation(0, 1);

    return (re * (1 / (2 * sin_theta) * aux) * theta);
}

void move_2(eepos end_pose)
{
    double dt = 0.001, MOVEMENT_VELOCITY = 0.005;

    eepos start_pose = direct_kinematic_eepos(currentJoint);

    Eigen::Vector3d delta = end_pose.ee_pos - start_pose.ee_pos;

    double velo = delta.norm() / MOVEMENT_VELOCITY;

    eepos temp_pose;
    Eigen::VectorXd joint_tempo = currentJoint;

    for (double i = dt; i <= velo; i += dt)
    {
        temp_pose = direct_kinematic_eepos(joint_tempo);

        Eigen::Vector3d vd = (((i / velo) * end_pose.ee_pos + (1 - (i / velo)) * start_pose.ee_pos) - (((i - dt) / velo) * end_pose.ee_pos + (1 - ((i - dt) / velo)) * start_pose.ee_pos)) / dt;
        Eigen::Vector3d Xarg = ((i / velo) * end_pose.ee_pos + (1 - (i / velo)) * start_pose.ee_pos);

        Eigen::Matrix3d kp = Eigen::Matrix3d::Identity() * 40;  // kp
        Eigen::Matrix3d kphi = Eigen::Matrix3d::Identity() * 5; // kphi

        // Eigen::MatrixXd wRd(6, 6);
        Eigen::Matrix3d wRd;
        wRd = Eigen::AngleAxisd(end_pose.ee_eul(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(end_pose.ee_eul(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(end_pose.ee_eul(2), Eigen::Vector3d::UnitX());

        Eigen::Vector3d errorVector;
        errorVector = computeOrientationError(temp_pose.ee_rotm, wRd);

        if (errorVector.norm() > 0.1)
        {
            errorVector = 0.1 * errorVector.normalized();
        }

        Eigen::MatrixXd J;
        J = jacobian(joint_tempo);

        Eigen::VectorXd dot_q(6), ve(6);

        double k = pow(10, -6);

        ve << (vd + kp * (Xarg - temp_pose.ee_pos)), (kphi * errorVector);
        dot_q = (J + Eigen::MatrixXd::Identity(6, 6) * k).inverse() * ve;

        for (int i = 0; i < 6; i++)
        {
            if (dot_q(i) > M_PI)
            {
                std::cout << "joint " << i << " > M_pi" << std::endl;
                dot_q(i) = M_PI;
            }
            if (dot_q(i) < -M_PI)
            {
                std::cout << "joint " << i << " < -M_pi" << std::endl;
                dot_q(i) = -M_PI;
            }
        }

        auto joint_tempo_1 = joint_tempo + dot_q * dt;
        joint_tempo = joint_tempo_1;

        send_joint(joint_tempo);
    }
    currentJoint = joint_tempo;
}

void move_3(eepos end_pose)
{
    ros::Rate rate(1000);
    int j = 0;

    // std::cout << inverse_kinematic(end_pose) << std::endl;
    Eigen::VectorXd joint_end(6);
    joint_end = inverse_kinematic(end_pose).row(j);
    double dt = 0.001, MOVEMENT_VELOCITY = 0.005;

    eepos start_pose = direct_kinematic_eepos(currentJoint);

    Eigen::Vector3d delta_pos = end_pose.ee_pos - start_pose.ee_pos;
    double T = sqrt(pow(delta_pos(0), 2) + pow(delta_pos(1), 2) + pow(delta_pos(2), 2)) * 5.0 + 2.0;

    Eigen::Matrix4d M;
    M << 1, 0, 0, 0,
        0, 1, 0, 0,
        1, pow(T, 1), pow(T, 2), pow(T, 3),
        0, 1, 2 * T, 3 * pow(T, 2);
    Eigen::Vector4d a, b;
    Eigen::MatrixXd delta(6, 4);

    for (int i = 0; i < 6; i++)
    {
        b << currentJoint(i), 0, joint_end(i), 0;
        a = M.inverse() * b;
        delta(i, 0) = a(0);
        delta(i, 1) = a(1);
        delta(i, 2) = a(2);
        delta(i, 3) = a(3);
    }
    // std::cout << Aa.rows() << " - " << Aa.cols() << std::endl;
    // std::cout << Aa << std::endl << std::endl;

    Eigen::VectorXd joint_tempo(6);
    std::ofstream outfile;
    outfile.open("joint.log");
    for (double t = 0; t <= T; t += dt)
    {
        // std::cout << "--------------------"<<t<<"--------------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            joint_tempo(i) = delta(i, 0) + delta(i, 1) * pow(t, 1) + delta(i, 2) * pow(t, 2) + delta(i, 3) * pow(t, 3);
            // std::cout << Aa(i, 0) << std::endl;
            // std::cout << Aa(i, 1) * t << std::endl;
            // std::cout << Aa(i, 2) * pow(t, 2) << std::endl;
            // std::cout << Aa(i, 3) * pow(i, 3) << std::endl;
            // std::cout << "----------------------------------------" << std::endl;
        }

        // std::cout << joint_tempo.transpose() << std::endl;

        outfile << joint_tempo.transpose() << "\n";

        send_joint(joint_tempo);

        rate.sleep();
    }
    currentJoint = joint_tempo;
    outfile.close();
}
