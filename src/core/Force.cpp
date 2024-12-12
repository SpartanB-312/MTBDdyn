#include "Force.h"

// Constructor
Force::Force()
{
}

// Destructor
Force::~Force()
{
}

void Force::setRPCFObjects(std::vector<RPCF> &objects)
{
    this->rpcfObjects = objects;
    std::vector<int> rpcfIds;
    for (size_t i = 0; i < rpcfObjects.size(); ++i) {
        rpcfIds.push_back(rpcfObjects[i].getId());
    }
    this->rpcfIds = rpcfIds;
    this->F.resize(objects.size(), Eigen::MatrixXd::Zero(3, 1)); 
    this->np.resize(objects.size(), Eigen::MatrixXd::Zero(3, 1));
}

void Force::setType(int type)
{
    this->type = type;
}

void Force::setGravity(double &g, Eigen::MatrixXd &dir)
{
    for (size_t i = 0; i < rpcfObjects.size(); ++i) {
        const RPCF &rpcf = rpcfObjects[i];
        this->F[i] = rpcf.getMass() * g * dir;
        this->np[i] = Eigen::MatrixXd::Zero(3, 1);
    }
}

void Force::setGravityZ()
{
    double g = -9.81;
    Eigen::MatrixXd dir(3, 1);
    dir << 0, 0, 1;
    for (size_t i = 0; i < rpcfObjects.size(); ++i) {
        const RPCF &rpcf = rpcfObjects[i];
        this->F[i] = rpcf.getMass() * g * dir;
        this->np[i] = (Eigen::MatrixXd::Zero(3, 1));
    }
}

std::vector<Eigen::MatrixXd> Force::getF()
{
    return this->F;
}

std::vector<Eigen::MatrixXd> Force::getnp()
{
    return this->np;
}

std::vector<Eigen::MatrixXd> Force::getForce()
{
    // 考虑到RPCF的np还要转化一次，暂不实现
    return this->F;
}

std::vector<int> Force::getRPCFid()
{
    return this->rpcfIds;
}