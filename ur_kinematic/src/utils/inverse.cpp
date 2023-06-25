#include "../parameters.h"
#include "eul2rotm.cpp"

#include <math.h>
#include <complex>
#include <eigen3/Eigen/Core>

using namespace Eigen;
using namespace std;

MatrixXd inverse_kinematic(Vector3d p, Matrix3d rotm) {

    Matrix3d R60 = rotm;
    MatrixXd T60(4,4);
    double endEffectorPos[3] = {p(0), p(1), p(2)};

    T60 <<  R60(0, 0), R60(0, 1), R60(0, 2), endEffectorPos[0],
            R60(1, 0), R60(1, 1), R60(1, 2), endEffectorPos[1],
            R60(2, 0), R60(2, 1), R60(2, 2), endEffectorPos[2],
            0,        0,        0,        1;

    //Computation values for th1
    MatrixXd p50(1, 4);
    MatrixXd temp(4, 1);
    temp << 0, 0, -D[5], 1;

    p50 = T60 * temp;

    complex<double> th1_1 = real(atan2(round(p50(1, 0)*100)/100, round(p50(0, 0)*100)/100) + real(acos(D[3] / hypot(p50(1, 0), p50(0, 0)))) + M_PI_2);
    complex<double> th1_2 = real(atan2(round(p50(1, 0)*100)/100, round(p50(0, 0)*100)/100) - real(acos(D[3] / hypot(p50(1, 0), p50(0, 0)))) + M_PI_2);

    //Computation values for th5
    complex<double> th5_1 = real(acos((endEffectorPos[0] * sin(th1_1) - endEffectorPos[1] * cos(th1_1) - D[3]) / D[5]));
    complex<double> th5_2 = real(-acos((endEffectorPos[0] * sin(th1_1) - endEffectorPos[1] * cos(th1_1) - D[3]) / D[5]));
    complex<double> th5_3 = real(acos((endEffectorPos[0] * sin(th1_2) - endEffectorPos[1] * cos(th1_2) - D[3]) / D[5]));
    complex<double> th5_4 = real(-acos((endEffectorPos[0] * sin(th1_2) - endEffectorPos[1] * cos(th1_2) - D[3]) / D[5]));

    //Computation values for th6
    // related to th11 a th51
    MatrixXd T06(4, 4);
    MatrixXd Xhat(3, 1);
    MatrixXd Yhat(3, 1);

    T06 = T60.inverse();
    Xhat = T06.block(0, 0, 3, 1);
    Yhat = T06.block(0, 1, 3, 1);

    complex<double> th6_1 = real(atan2(((-Xhat(1) * real(sin(th1_1)) + Yhat(1) * real(cos(th1_1))) / real(sin(th5_1))), ((Xhat(0) * real(sin(th1_1)) - Yhat(0) * real(cos(th1_1))) / real(sin(th5_1)))));
    complex<double> th6_2 = real(atan2(((-Xhat(1) * real(sin(th1_1)) + Yhat(1) * real(cos(th1_1))) / real(sin(th5_2))), ((Xhat(0) * real(sin(th1_1)) - Yhat(0) * real(cos(th1_1))) / real(sin(th5_2)))));
    complex<double> th6_3 = real(atan2(((-Xhat(1) * real(sin(th1_2)) + Yhat(1) * real(cos(th1_2))) / real(sin(th5_3))), ((Xhat(0) * real(sin(th1_2)) - Yhat(0) * real(cos(th1_2))) / real(sin(th5_3)))));
    complex<double> th6_4 = real(atan2(((-Xhat(1) * real(sin(th1_2)) + Yhat(1) * real(cos(th1_2))) / real(sin(th5_4))), ((Xhat(0) * real(sin(th1_2)) - Yhat(0) * real(cos(th1_2))) / real(sin(th5_4)))));

    MatrixXd T41m(4, 4);
    MatrixXd p41_1(1, 4);
    MatrixXd p41_2(1, 4);
    MatrixXd p41_3(1, 4);
    MatrixXd p41_4(1, 4);

    //------------------------
    //cout << T10d(th1_1) << endl;

    T41m = T10d(real(th1_1)).inverse() * T60 * T65d(real(th6_1)).inverse() * T54d(real(th5_1)).inverse();
    p41_1 = T41m.block(0, 3, 3, 1);
    double p41xz_1 = hypot(p41_1(0), p41_1(2));

    T41m = T10d(real(th1_1)).inverse() * T60 * T65d(real(th6_2)).inverse() * T54d(real(th5_2)).inverse();
    p41_2 = T41m.block(0, 3, 3, 1);
    double p41xz_2 = hypot(p41_2(0), p41_2(2));

    T41m = T10d(real(th1_2)).inverse() * T60 * T65d(real(th6_3)).inverse() * T54d(real(th5_3)).inverse();
    p41_3 = T41m.block(0, 3, 3, 1);
    double p41xz_3 = hypot(p41_3(0), p41_3(2));

    T41m = T10d(real(th1_2)).inverse() * T60 * T65d(real(th6_4)).inverse() * T54d(real(th5_4)).inverse();
    p41_4 = T41m.block(0, 3, 3, 1);
    double p41xz_4 = hypot(p41_4(0), p41_4(2));

    //Computation of the 8 possible values for th3
    complex<double> th3_1 = real(acos((pow(p41xz_1, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2])));
    complex<double> th3_2 = real(acos((pow(p41xz_2, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2])));
    complex<double> th3_3 = real(acos((pow(p41xz_3, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2])));
    complex<double> th3_4 = real(acos((pow(p41xz_4, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2])));
    
    complex<double> th3_5 = -th3_1;
    complex<double> th3_6 = -th3_2;
    complex<double> th3_7 = -th3_3;
    complex<double> th3_8 = -th3_4;

    complex<double> th2_1 = real(atan2(-p41_1(2), -p41_1(0))) - real(asin((-A[2] * real(sin(th3_1))) / p41xz_1));
    complex<double> th2_2 = real(atan2(-p41_2(2), -p41_2(0))) - real(asin((-A[2] * real(sin(th3_2))) / p41xz_2));
    complex<double> th2_3 = real(atan2(-p41_3(2), -p41_3(0))) - real(asin((-A[2] * real(sin(th3_3))) / p41xz_3));
    complex<double> th2_4 = real(atan2(-p41_4(2), -p41_4(0))) - real(asin((-A[2] * real(sin(th3_4))) / p41xz_4));

    complex<double> th2_5 = real(atan2(-p41_1(2), -p41_1(0))) - real(asin((A[2] * real(sin(th3_1))) / p41xz_1));
    complex<double> th2_6 = real(atan2(-p41_2(2), -p41_2(0))) - real(asin((A[2] * real(sin(th3_2))) / p41xz_2));
    complex<double> th2_7 = real(atan2(-p41_3(2), -p41_3(0))) - real(asin((A[2] * real(sin(th3_3))) / p41xz_3));
    complex<double> th2_8 = real(atan2(-p41_4(2), -p41_4(0))) - real(asin((A[2] * real(sin(th3_4))) / p41xz_4));

    //Computation of the 8 possible value for th4
    MatrixXd T43m(4,4);
    MatrixXd Xhat43(1,4);

    T43m = T32d(real(th3_1)).inverse() * T21d(real(th2_1)).inverse() * T10d(real(th1_1)).inverse() * T60 * T65d(real(th6_1)).inverse() * T54d(real(th5_1)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_1 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32d(real(th3_2)).inverse() * T21d(real(th2_2)).inverse() * T10d(real(th1_1)).inverse() * T60 * T65d(real(th6_2)).inverse() * T54d(real(th5_2)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_2 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32d(real(th3_3)).inverse() * T21d(real(th2_3)).inverse() * T10d(real(th1_2)).inverse() * T60 * T65d(real(th6_3)).inverse() * T54d(real(th5_3)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_3 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32d(real(th3_4)).inverse() * T21d(real(th2_4)).inverse() * T10d(real(th1_2)).inverse() * T60 * T65d(real(th6_4)).inverse() * T54d(real(th5_4)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_4 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32d(real(th3_5)).inverse() * T21d(real(th2_5)).inverse() * T10d(real(th1_1)).inverse() * T60 * T65d(real(th6_1)).inverse() * T54d(real(th5_1)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_5 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32d(real(th3_6)).inverse() * T21d(real(th2_6)).inverse() * T10d(real(th1_1)).inverse() * T60 * T65d(real(th6_2)).inverse() * T54d(real(th5_2)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_6 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32d(real(th3_7)).inverse() * T21d(real(th2_7)).inverse() * T10d(real(th1_2)).inverse() * T60 * T65d(real(th6_3)).inverse() * T54d(real(th5_3)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_7 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32d(real(th3_8)).inverse() * T21d(real(th2_8)).inverse() * T10d(real(th1_2)).inverse() * T60 * T65d(real(th6_4)).inverse() * T54d(real(th5_4)).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    double th4_8 = atan2(Xhat43(1), Xhat43(0));

    //Reuslt of the inverse kinematics
    MatrixXd Th(8,6);

    Th << real(th1_1), real(th2_1), real(th3_1), real(th4_1), real(th5_1), real(th6_1),
          real(th1_1), real(th2_5), real(th3_5), real(th4_5), real(th5_1), real(th6_1),
          real(th1_1), real(th2_2), real(th3_2), real(th4_2), real(th5_2), real(th6_2),
          real(th1_1), real(th2_6), real(th3_6), real(th4_6), real(th5_2), real(th6_2),
          real(th1_2), real(th2_3), real(th3_3), real(th4_3), real(th5_3), real(th6_3),
          real(th1_2), real(th2_4), real(th3_4), real(th4_4), real(th5_4), real(th6_4),
          real(th1_2), real(th2_7), real(th3_7), real(th4_7), real(th5_3), real(th6_3),
          real(th1_2), real(th2_8), real(th3_8), real(th4_8), real(th5_4), real(th6_4);

    return Th;
}

MatrixXd inverse_kinematic(eepos pose) {
    Vector3d p = pose.ee_pos;
    Matrix3d rotm = pose.ee_rotm;
    return inverse_kinematic(p, rotm);
}
