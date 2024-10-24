#ifndef DYNMATH_H
#define DYNMATH_H

#include "eigen-3.4.0/Eigen/Dense"

namespace dynMath
{
    // 将 Eigen::MatrixXd 转换为 Eigen::VectorXd
    Eigen::VectorXd matrixToVector(const Eigen::MatrixXd &matrix);

    Eigen::MatrixXd VecCross(const Eigen::MatrixXd &vec);
}

#endif // DYNMATH_H