#include "dynMath.h"

namespace dynMath
{
    // 将 Eigen::MatrixXd 转换为 Eigen::VectorXd
    Eigen::VectorXd matrixToVector(const Eigen::MatrixXd &matrix)
    {
        // 将矩阵的元素按列顺序存储到向量中
        Eigen::VectorXd vec(matrix.size());
        int index = 0;
        for (int j = 0; j < matrix.cols(); ++j)
        {
            for (int i = 0; i < matrix.rows(); ++i)
            {
                vec(index++) = matrix(i, j);
            }
        }
        return vec;
    }

    Eigen::MatrixXd VecCross(const Eigen::MatrixXd &vec)
    {
        Eigen::Matrix3d mat;
        mat << 0, -vec(2,0), vec(1,0),
            vec(2,0), 0, -vec(0,0),
            -vec(1,0), vec(0,0), 0;
        return mat;
    }

    Eigen::MatrixXd p2A(const Eigen::MatrixXd &p)
    {
        Eigen::MatrixXd A(3, 3);
        double e0 = p(0, 0);
        Eigen::MatrixXd e = p.block(1, 0, 3, 1);
        A = e0 * e0 * Eigen::MatrixXd::Identity(3, 3) - e.transpose() * e * Eigen::MatrixXd::Identity(3, 3) + 2 * e * e.transpose() + 2 * e0 * VecCross(e);
        return A;
    }

    Eigen::MatrixXd GCal(const Eigen::MatrixXd &p)
    {
        double e0 = p(0, 0);
        Eigen::MatrixXd e = p.block(1, 0, 3, 1);
        Eigen::MatrixXd ecross = VecCross(e);
        Eigen::MatrixXd result(3, 4);
        Eigen::MatrixXd temp;
        temp = - ecross + e0 * Eigen::MatrixXd::Identity(3, 3);
        result << -e, temp;
        return result;
    }

    Eigen::MatrixXd ECal(const Eigen::MatrixXd &p)
    {
        double e0 = p(0, 0);
        Eigen::MatrixXd e = p.block(1, 0, 3, 1);
        Eigen::MatrixXd ecross = VecCross(e);
        Eigen::MatrixXd result(3, 4);
        Eigen::MatrixXd temp;
        temp = ecross + e0 * Eigen::MatrixXd::Identity(3, 3);
        result << -e, temp;
        return result;
    }
}