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

    Eigen::MatrixXd epdBCal(const Eigen::MatrixXd &q, const Eigen::MatrixXd &a)
    {
        Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
        double e0 = q(3, 0);
        Eigen::MatrixXd e = q.block(4, 0, 3, 1);
        Eigen::MatrixXd ecross = VecCross(e);
        Eigen::MatrixXd across = VecCross(a);
        Eigen::MatrixXd E(3, 4);
        E << -e, ecross + e0 * I3;
        Eigen::MatrixXd T(4, 4);
        T << 0, -a.transpose(),
            a, -across;
        Eigen::MatrixXd result = 2 * E * T;
        return result;
    }

    Eigen::MatrixXd P2DistCal(const Eigen::MatrixXd &qi, const Eigen::MatrixXd &qj, const Eigen::MatrixXd &chii, const Eigen::MatrixXd &chij,
                              const Eigen::MatrixXd &si, const Eigen::MatrixXd &sj, const double d)
    {
        Eigen::MatrixXd pi = qi.block(3, 0, 4, 1);
        Eigen::MatrixXd pj = qj.block(3, 0, 4, 1);
        Eigen::MatrixXd ri = qi.block(0, 0, 3, 1);
        Eigen::MatrixXd rj = qj.block(0, 0, 3, 1);
        Eigen::MatrixXd Ai = p2A(pi);
        Eigen::MatrixXd Aj = p2A(pj);
        Eigen::MatrixXd Bi = epdBCal(qi, si);
        Eigen::MatrixXd Bj = epdBCal(qj, sj);
        Eigen::MatrixXd Bci = epdBCal(chii, si);
        Eigen::MatrixXd Bcj = epdBCal(chij, sj);
        Eigen::MatrixXd chiri = chii.block(0, 0, 3, 1);
        Eigen::MatrixXd chirj = chij.block(0, 0, 3, 1);
        Eigen::MatrixXd chipi = chii.block(3, 0, 4, 1);
        Eigen::MatrixXd chipj = chij.block(3, 0, 4, 1);

        Eigen::MatrixXd dij = rj + Aj * sj - ri - Ai * si;
        Eigen::MatrixXd bi = chiri + Bi * chipi;
        Eigen::MatrixXd bj = chirj + Bj * chipj;
        Eigen::MatrixXd P2i(1, 7);
        P2i << (bi - bj).transpose(), (bi - bj).transpose() * Bi - dij.transpose() * Bci;
        Eigen::MatrixXd P2j(1, 7);
        P2j << (bj - bi).transpose(), (bj - bi).transpose() * Bj + dij.transpose() * Bcj;
        Eigen::MatrixXd P2(1, 14);
        P2 << P2i, P2j;
        return P2;
    }
}