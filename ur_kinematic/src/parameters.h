#ifndef __PARAMETERS__
#define __PARAMETERS__

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string>

using namespace Eigen; 

double A[6] = {0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.00000};
double D[6] = {0.16250,  0.00000,  0.00000,  0.13330,  0.09970,  0.09960};

bool real_robot = false;
bool debug = false;

VectorXd joint_pos(6);
ros::Publisher pub_joint;

struct eepos{
    Vector3d ee_pos;
    Matrix3d ee_rotm;
    Vector3d ee_eul;
};

struct Prediction {
    double x;
    double y;
    double z;
    int c;
};

// ---------------------------------------- UNO ----------------------------------------
MatrixXd T10d(double th1){
    //--homogeneous trasformation di th1 rispetto all'asse z
    //--la posizione del nuovo frame è solamente spostata verso il basso
    MatrixXd m(4, 4);
    m << cos(th1), -sin(th1),    0,    0,
         sin(th1),  cos(th1),    0,    0,
            0,         0,        1,    D[0],
            0,         0,        0,    1;

    return m;
}
// ---------------------------------------- DUE ----------------------------------------
MatrixXd T21d(double th2){
    //asse y
    MatrixXd m(4, 4);
    m << cos(th2),  -sin(th2),  0,   0,
         0,          0,        -1,   0,
         sin(th2),   cos(th2),  0,   0,
         0,          0,         0,   1;

    return m;
}
// ---------------------------------------- TRE ---------------------------------------- 
MatrixXd T32d(double th3){
    //--asse z
    MatrixXd m(4, 4);
    m << cos(th3), -sin(th3),   0,   A[1],
         sin(th3),  cos(th3),   0,   0,
         0,         0,          1,   D[2],
         0,         0,          0,   1;
    return m;
}
// -------------------------------------- QUATTRO --------------------------------------
MatrixXd T43d(double th4){
    //--asse z
    MatrixXd m(4, 4);
    m << cos(th4), -sin(th4),   0,  A[2],
         sin(th4),  cos(th4),   0,  0,
         0,         0,          1,  D[3],
         0,         0,          0,  1;
    return m;
}
// -------------------------------------- CINQUE ---------------------------------------
MatrixXd T54d(double th5){
    //--asse y
    MatrixXd m(4, 4);
    m <<    cos(th5),  -sin(th5),   0,  0,
            0,          0,         -1,  -D[4],
            sin(th5),   cos(th5),   0,  0,
            0,          0,          0,  1;
    return m;
}
// ---------------------------------------- SEI ----------------------------------------
MatrixXd T65d(double th6){
    //--asse y
    MatrixXd m(4, 4);
    m << cos(th6), -sin(th6),   0,    0,
         0,         0,          1,    D[5],
        -sin(th6), -cos(th6),   0,    0,
         0,         0,          0,    1;
    return m;
}

// direct kinematic
MatrixXd direct_kinematic(VectorXd Th);
eepos direct_kinematic_eepos(VectorXd Th);

// inverse kinematic
MatrixXd inverse_kinematic(Vector3d p, Vector3d phi);
MatrixXd inverse_kinematic(eepos);

// jacobian
MatrixXd jacobian (MatrixXd Th);

#endif