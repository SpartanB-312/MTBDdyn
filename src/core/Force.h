#ifndef FORCE_H
#define FORCE_H

#include <iostream>
#include <vector>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"

/* 暂时不考虑力的作用点偏离质心的情况
   或许以后需要加入一个函数类用于描述力的时间历程
   暂时不加入力元
*/

class Force
{
public:
    // Constructor
    Force();
    // Destructor
    ~Force();

    void setRPCFObjects(std::vector<RPCF> &objects);
    void setType(int type);

    void setGravity(double &g, Eigen::MatrixXd &dir); // 重力是常值，不需要在每次循环里计算
    void setGravityZ();

    std::vector<Eigen::MatrixXd> getF();
    std::vector<Eigen::MatrixXd> getnp();

    std::vector<Eigen::MatrixXd> getForce();

    std::vector<int> getRPCFid();

private:
    std::vector<RPCF> rpcfObjects;
    std::vector<int> rpcfIds;
    std::vector<Eigen::MatrixXd> F;
    std::vector<Eigen::MatrixXd> np;
    int type;
};

#endif // FORCE_H