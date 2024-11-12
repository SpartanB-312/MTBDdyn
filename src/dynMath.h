#ifndef DYNMATH_H
#define DYNMATH_H

#include "eigen-3.4.0/Eigen/Dense"

namespace dynMath
{
    // 将 Eigen::MatrixXd 转换为 Eigen::VectorXd
    Eigen::VectorXd matrixToVector(const Eigen::MatrixXd &matrix);

    Eigen::MatrixXd VecCross(const Eigen::MatrixXd &vec);

    Eigen::MatrixXd p2A(const Eigen::MatrixXd &p);
    Eigen::MatrixXd GCal(const Eigen::MatrixXd &p);
    Eigen::MatrixXd ECal(const Eigen::MatrixXd &p);
}

#endif // DYNMATH_H