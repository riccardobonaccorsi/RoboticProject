#include "../parameters.h"
#include <eigen3/Eigen/Core>
#include <iostream>

using namespace std;
using namespace Eigen;

MatrixXd jacobian(MatrixXd Th)
{
    MatrixXd J1(6, 1);
    J1 << D[4] * (cos(Th(0)) * cos(Th(4)) + cos(Th(1) + Th(2) + Th(3)) * sin(Th(0)) * sin(Th(4))) + D[2] * cos(Th(0)) + D[3] * cos(Th(0)) - A[2] * cos(Th(1) + Th(2)) * sin(Th(0)) - A[1] * cos(Th(1)) * sin(Th(0)) - D[4] * sin(Th(1) + Th(2) + Th(3)) * sin(Th(0)),
        D[4] * (cos(Th(4)) * sin(Th(0)) - cos(Th(1) + Th(2) + Th(3)) * cos(Th(0)) * sin(Th(4))) + D[2] * sin(Th(0)) + D[3] * sin(Th(0)) + A[2] * cos(Th(1) + Th(2)) * cos(Th(0)) + A[1] * cos(Th(0)) * cos(Th(1)) + D[4] * sin(Th(1) + Th(2) + Th(3)) * cos(Th(0)),
        0,
        0,
        0,
        1;

    MatrixXd J2(6, 1);
    J2 << -cos(Th(0)) * (A[2] * sin(Th(1) + Th(2)) + A[1] * sin(Th(1)) + D[4] * (sin(Th(1) + Th(2)) * sin(Th(3)) - cos(Th(1) + Th(2)) * cos(Th(3))) - D[4] * sin(Th(4)) * (cos(Th(1) + Th(2)) * sin(Th(3)) + sin(Th(1) + Th(2)) * cos(Th(3)))),
        -sin(Th(0)) * (A[2] * sin(Th(1) + Th(2)) + A[1] * sin(Th(1)) + D[4] * (sin(Th(1) + Th(2)) * sin(Th(3)) - cos(Th(1) + Th(2)) * cos(Th(3))) - D[4] * sin(Th(4)) * (cos(Th(1) + Th(2)) * sin(Th(3)) + sin(Th(1) + Th(2)) * cos(Th(3)))),
        A[2] * cos(Th(1) + Th(2)) - (D[4] * sin(Th(1) + Th(2) + Th(3) + Th(4))) / 2 + A[1] * cos(Th(1)) + (D[4] * sin(Th(1) + Th(2) + Th(3) - Th(4))) / 2 + D[4] * sin(Th(1) + Th(2) + Th(3)),
        sin(Th(0)),
        -cos(Th(0)),
        0;

    MatrixXd J3(6, 1);
    J3 << cos(Th(0)) * (D[4] * cos(Th(1) + Th(2) + Th(3)) - A[2] * sin(Th(1) + Th(2)) + D[4] * sin(Th(1) + Th(2) + Th(3)) * sin(Th(4))),
        sin(Th(0)) * (D[4] * cos(Th(1) + Th(2) + Th(3)) - A[2] * sin(Th(1) + Th(2)) + D[4] * sin(Th(1) + Th(2) + Th(3)) * sin(Th(4))),
        A[2] * cos(Th(1) + Th(2)) - (D[4] * sin(Th(1) + Th(2) + Th(3) + Th(4))) / 2 + (D[4] * sin(Th(1) + Th(2) + Th(3) - Th(4))) / 2 + D[4] * sin(Th(1) + Th(2) + Th(3)),
        sin(Th(0)),
        -cos(Th(0)),
        0;

    MatrixXd J4(6, 1);
    J4 << D[4] * cos(Th(0)) * (cos(Th(1) + Th(2) + Th(3)) + sin(Th(1) + Th(2) + Th(3)) * sin(Th(4))),
        D[4] * sin(Th(0)) * (cos(Th(1) + Th(2) + Th(3)) + sin(Th(1) + Th(2) + Th(3)) * sin(Th(4))),
        D[4] * (sin(Th(1) + Th(2) + Th(3) - Th(4)) / 2 + sin(Th(1) + Th(2) + Th(3)) - sin(Th(1) + Th(2) + Th(3) + Th(4)) / 2),
        sin(Th(0)),
        -cos(Th(0)),
        0;

    MatrixXd J5(6, 1);
    J5 << -D[4] * sin(Th(0)) * sin(Th(4)) - D[4] * cos(Th(1) + Th(2) + Th(3)) * cos(Th(0)) * cos(Th(4)),
        D[4] * cos(Th(0)) * sin(Th(4)) - D[4] * cos(Th(1) + Th(2) + Th(3)) * cos(Th(4)) * sin(Th(0)),
        -D[4] * (sin(Th(1) + Th(2) + Th(3) - Th(4)) / 2 + sin(Th(1) + Th(2) + Th(3) + Th(4)) / 2),
        sin(Th(1) + Th(2) + Th(3)) * cos(Th(0)),
        sin(Th(1) + Th(2) + Th(3)) * sin(Th(0)),
        -cos(Th(1) + Th(2) + Th(3));

    MatrixXd J6(6, 1);
    J6 << 0,
        0,
        0,
        cos(Th(4)) * sin(Th(0)) - cos(Th(1) + Th(2) + Th(3)) * cos(Th(0)) * sin(Th(4)),
        -cos(Th(0)) * cos(Th(4)) - cos(Th(1) + Th(2) + Th(3)) * sin(Th(0)) * sin(Th(4)),
        -sin(Th(1) + Th(2) + Th(3)) * sin(Th(4));

    MatrixXd J(6, 6);
    J << J1, J2, J3, J4, J5, J6;

    return J;
}