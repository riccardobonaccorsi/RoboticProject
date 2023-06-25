#include "../parameters.h"
#include <eigen3/Eigen/Core>
#include <iostream>

using namespace std;
using namespace Eigen;

MatrixXd direct_kinematic(VectorXd th) {
    MatrixXd rtn = T10d(th(0)) * T21d(th(1))  * T32d(th(2)) * T43d(th(3)) * T54d(th(4)) * T65d(th(5));
    return rtn;
}

eepos direct_kinematic_eepos(VectorXd Th) {
    eepos ee;
    auto rtn = direct_kinematic(Th);
    ee.ee_pos  = rtn.block(0, 3, 3, 1);
    ee.ee_rotm = rtn.block(0, 0, 3, 3);
    ee.ee_eul  << 0, 0, 0;
    return ee;
}