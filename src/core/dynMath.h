#ifndef DYNMATH_H
#define DYNMATH_H

#include <iostream>
#include "eigen-3.4.0/Eigen/Dense"

namespace dynMath
{
    // 将 Eigen::MatrixXd 转换为 Eigen::VectorXd
    Eigen::VectorXd matrixToVector(const Eigen::MatrixXd &matrix);

    Eigen::MatrixXd VecCross(const Eigen::MatrixXd &vec);

    Eigen::MatrixXd p2A(const Eigen::MatrixXd &p);
    Eigen::MatrixXd GCal(const Eigen::MatrixXd &p);
    Eigen::MatrixXd ECal(const Eigen::MatrixXd &p);
    Eigen::MatrixXd epdBCal(const Eigen::MatrixXd &q, const Eigen::MatrixXd &a);
    Eigen::MatrixXd P2DistCal(const Eigen::MatrixXd &qi, const Eigen::MatrixXd &qj, const Eigen::MatrixXd &chii, const Eigen::MatrixXd &chij,
                              const Eigen::MatrixXd &si, const Eigen::MatrixXd &sj, const double d);
}

#endif // DYNMATH_H