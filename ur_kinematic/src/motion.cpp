#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <string>
#include <vector>

#include "parameters.h"
#include "utils/inverse.cpp"
#include "utils/direct.cpp"
#include "utils/jacobian.cpp"

#define PI 3.14159265

/**
 * if true  -> soft gripper
 * if false -> hard gripper
*/
#define soft_gripper false

#define ROBOT_JOINTS 6
#define GRIPPER_JOINTS 3

// DICHIARAZIONE DEI METODI
void vision_callback(std_msgs::String);
std::vector<std::string> split(std::string, std::string);
void plan(Prediction *, int);
void close_gripper();
void open_gripper();
void move(eepos, double MOVEMENT_VELOCITY = 0.01);
Eigen::Vector3d computeOrientationError(Eigen::Matrix3d, Eigen::Matrix3d);
void send_joint(Eigen::VectorXd);

// DICHIARAZIONE DI VARIABILI
Eigen::VectorXd currentJoint(ROBOT_JOINTS);
Eigen::Vector3d gripper_joint(GRIPPER_JOINTS);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion");
    ros::NodeHandle n;
    // joint_pos << -0.32, 16.5781, 2.49245, -23.4795, -14.4024, -21.1548;
    joint_pos << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    currentJoint << joint_pos;

    pub_joint = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Subscriber sub = n.subscribe("/vision", 100, vision_callback);

    ros::spin();

    return 0;
}


std::vector<std::string> split(std::string s, std::string delimiter)
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
    {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }

    res.push_back(s.substr(pos_start));
    return res;
}

void send_joint(Eigen::VectorXd joints_value)
{
    if (joints_value.rows() == 6)
    {
        std_msgs::Float64MultiArray msg;
        msg.data.resize(ROBOT_JOINTS + GRIPPER_JOINTS);
        // std::cout << "send: ";
        for (int i = 0; i < ROBOT_JOINTS; i++)
        {
            msg.data[i] = (float)joints_value[i];
            // std::cout << msg.data[i] << " ";
        }
        
        msg.data[ROBOT_JOINTS + 0] = gripper_joint(0);
        msg.data[ROBOT_JOINTS + 1] = gripper_joint(1);
        if (!soft_gripper) {
            msg.data[ROBOT_JOINTS + 2] = gripper_joint(2);
        }

        // std::cout << std::endl;
        pub_joint.publish(msg);
    }
}

void vision_callback(std_msgs::String msg)
{
    std::vector<std::string> data = split(msg.data, " || ");

    int n_block = data.size() - 1;
    Prediction blocks[n_block];

    for (int i = 0; i < data.size() - 1; i++)
    {
        /**
         * splitto la stringa in modo da avere 
         */
        std::vector<std::string> data_splitted = split(data[i], " ");
        for (int k = 0; k < 4; k++) std::cout << "data[" << k << "] : " << data_splitted[k] << std::endl;

        /**
         * salvo i dati in array di blocchi:
         */
        if (data_splitted.size() == 4)
        {
            blocks[i].x = std::stod(data_splitted[0]) / 1000;       // pos x
            blocks[i].y = std::stod(data_splitted[1]) / 1000;       // pos y
            blocks[i].z = std::stod(data_splitted[2]) / 1000;       // pos z
            blocks[i].c = std::stoi(data_splitted[3]);              // class

            std::cout << "x: " << std::stod(data_splitted[2]) / 1000 << std::endl;
            std::cout << "y: " << std::stod(data_splitted[1]) / 1000 << std::endl;
            std::cout << "z: " << std::stod(data_splitted[0]) / 1000 << std::endl;
            std::cout << "c: " << data_splitted[3] << std::endl;
        }
        else
        {
            std::cout << "Non abbastanza dati" << std::endl;
        }
    }
    plan(blocks, n_block);
}

void plan(Prediction *data, int n_block)
{
    ros::Rate rate(1000);
    for (int i = 0; i < n_block; i++)
    {
        std::cout << "PLAN num " << (i+1) << std::endl;
        
        eepos block_start;
        block_start.ee_pos << data[i].x, data[i].y, 0.72;
        block_start.ee_eul << Eigen::Vector3d::Zero();
        std::cout << "pos : " << block_start.ee_pos(0) << " " << block_start.ee_pos(1) << " " << block_start.ee_pos(2) << std::endl;

        eepos move1;
        move1 = block_start;
        move1.ee_pos(1) -= (0.031 / 2);
        move1.ee_pos(2) -= 0.2;
        
        std::cout << "pos : " << move1.ee_pos(0) << " " << move1.ee_pos(1) << " " << move1.ee_pos(2) << std::endl;

        move(move1, 0.025);

        eepos move2;
        move2 = block_start;
        move2.ee_pos(1) -= (0.031 / 2);
        move2.ee_pos(2) -= 0.035;
        std::cout << "pos : " << move2.ee_pos(0) << " " << move2.ee_pos(1) << " " << move2.ee_pos(2) << std::endl;

        move(move2);

        close_gripper();
    
        eepos move3;
        move3 = block_start;
        move3.ee_pos(1) -= 0.031;
        move3.ee_pos(2) -= 0.2;
        std::cout << "pos : " << move3.ee_pos(0) << " " << move3.ee_pos(1) << " " << move3.ee_pos(2) << std::endl;

        move(move3);

        eepos target;
        if (data[i].c == 1) {
        target.ee_pos << 0.4, -0.34, 0.720;
        } else {
        target.ee_pos << 0.4, -0.20, 0.720;
        }
        target.ee_eul = Eigen::Vector3d::Zero();
        std::cout << "pos : " << target.ee_pos(0) << " " << target.ee_pos(1) << " " << target.ee_pos(2) << std::endl;

        eepos move4;
        move4 = target;
        move4.ee_pos(2) -= 0.2;
        std::cout << "pos : " << move4.ee_pos(0) << " " << move4.ee_pos(1) << " " << move4.ee_pos(2) << std::endl;

        move(move4, 0.025);

        eepos move5;
        move5 = target;
        move5.ee_pos(2) -= 0.035;
        std::cout << "pos : " << move5.ee_pos(0) << " " << move5.ee_pos(1) << " " << move5.ee_pos(2) << std::endl;

        move(move5);

        open_gripper();

        eepos move6;
        move6 = target;
        move6.ee_pos(2) -= 0.2;
        std::cout << "pos : " << move6.ee_pos(0) << " " << move6.ee_pos(1) << " " << move6.ee_pos(2) << std::endl;

        move(move6);
    }
}

void close_gripper() {
    ros::Rate rate(1000);
    float diameter = 40;
    float alpha = (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    gripper_joint = Vector3d::Ones(3) * alpha;
    for (int i = 0; i < 15; i++)
    {
        send_joint(currentJoint);
        rate.sleep();
    }
    sleep(1);
}
void open_gripper() {
    ros::Rate rate(1000);
    float diameter = 100;
    for (int i = 0; i < 2; i++)
    {
        float alpha = (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
        gripper_joint = Vector3d::Ones(3) * alpha;

        send_joint(currentJoint);

        rate.sleep();
    }
    sleep(0.1);
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

void move(eepos end_pose, double MOVEMENT_VELOCITY)
{
    double dt = 0.001;

    eepos start_pose = direct_kinematic_eepos(currentJoint);

    Eigen::Vector3d delta = end_pose.ee_pos - start_pose.ee_pos;

    double time_velocity = delta.norm() / MOVEMENT_VELOCITY;

    eepos temp_pose;
    Eigen::VectorXd joint_tempo = currentJoint;

    for (double i = dt; i <= time_velocity; i += dt)
    {
        temp_pose = direct_kinematic_eepos(joint_tempo);

        Eigen::Vector3d vd = (((i / time_velocity) * end_pose.ee_pos + (1 - (i / time_velocity)) * start_pose.ee_pos) - (((i - dt) / time_velocity) * end_pose.ee_pos + (1 - ((i - dt) / time_velocity)) * start_pose.ee_pos)) / dt;
        Eigen::Vector3d Xarg = ((i / time_velocity) * end_pose.ee_pos + (1 - (i / time_velocity)) * start_pose.ee_pos);

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
