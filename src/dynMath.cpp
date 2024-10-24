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
}