#include "MTBDsys.h"

MTBDsys::MTBDsys()
{
}

MTBDsys::~MTBDsys()
{
}

void MTBDsys::preInitialize()
{
    Joints Inner;
    Inner.setRPCFObjects(rpcfObjs);
    Inner.setType(0);
    this->InnerPObjs.push_back(Inner);
    this->nb = rpcfObjs.size();
    this->nc = 7 * nb;
    int nhOj = 0;
    for(auto &Joints : jointsObjs)
    {
        nhOj += Joints.getnhj();
    }
    this->nhI = nb;
    this->nhO = nhOj;
    this->nh = nhO + nhI;
    //
    for (auto &rpcf : rpcfObjs)
    {
        rpcf.update();
    }
}

void MTBDsys::MassCal()
{
    Eigen::MatrixXd Mass;
    for (const auto &rpcf : rpcfObjs)
    {
        double mass = rpcf.getMass();
        Eigen::MatrixXd inertia = rpcf.getInertia();
        Eigen::MatrixXd G = rpcf.getG();

        // m*I3
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d mI3 = mass * I3;

        // 4*G'*J*G
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd GtJG = 4 * G.transpose() * J * G;

        // M
        Eigen::MatrixXd result(7, 7);
        result << mI3, Eigen::MatrixXd::Zero(3, 4),
            Eigen::MatrixXd::Zero(4, 3), GtJG;

        // 将 M 矩阵添加到 Mass 矩阵中
        int currentRows = Mass.rows();
        int currentCols = Mass.cols();
        Mass.conservativeResize(currentRows + 7, currentCols + 7);
        Mass.block(currentRows, currentCols, 7, 7) = result;
    }
    this->M = Mass;
}

void MTBDsys::ForceCal()
{
    this->QeCal();
    this->QvCal();
    Eigen::MatrixXd Q = this->Qe + this->Qv;
    this->Q = Q;
}

void MTBDsys::PhiCal()
{
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(0, 1);
    for (auto &joints : jointsObjs)
    {
        joints.PhiCal();
        Eigen::MatrixXd result = joints.getPhi();
        int currentRows = Phi.rows();
        Phi.conservativeResize(currentRows + result.rows(), Eigen::NoChange);
        Phi.block(currentRows, 0, result.rows(), result.cols()) = result;
    }
    this->Phi = Phi;
}

void MTBDsys::PhiqCal()
{
    Eigen::MatrixXd Phiq;
    for (auto &joints : jointsObjs)
    {
        joints.PhiqCal();
        std::cout << "3" << std::endl;
        std::vector<Eigen::MatrixXd> Phiqs = joints.getPhiq();
        std::vector<int> rpcfids = joints.getRPCFid();
        std::cout << Phiqs.size() << std::endl;
        int currentRows = Phiq.rows();
        Phiq.conservativeResize(currentRows + joints.getnhj(), 7);
        for (int bi = 0; bi < Phiqs.size(); ++bi)
        {
            Eigen::MatrixXd result = Phiqs[bi];
            std::cout << result << std::endl;
            Phiq.block(currentRows, rpcfids[bi] * 7, result.rows(), result.cols()) = result;
        }
    }

    for (auto &inner : InnerPObjs)
    {
        inner.PhiqCal();
        std::vector<Eigen::MatrixXd> Phiqs = inner.getPhiq();
        std::vector<int> rpcfids = inner.getRPCFid();
        std::cout << Phiqs.size() << std::endl;
        int currentRows = Phiq.rows();
        Phiq.conservativeResize(currentRows + inner.getnhj(), 7);
        for (int bi = 0; bi < Phiqs.size(); ++bi)
        {
            Eigen::MatrixXd result = Phiqs[bi];
            std::cout << result << std::endl;
            Phiq.block(currentRows, rpcfids[bi] * 7, result.rows(), result.cols()) = result;
        }
    }
    
    this->Phiq = Phiq;
}

void MTBDsys::gammaCal()
{
    Eigen::MatrixXd gamma;
    for (auto &joints : jointsObjs)
    {
        joints.gammaCal();
        Eigen::MatrixXd result = joints.getgamma();
        int currentRows = gamma.rows();
        gamma.conservativeResize(currentRows + result.rows(), 1);
        gamma.block(currentRows, 0, result.rows(), result.cols()) = result;
    }

    for (auto &inner : InnerPObjs)
    {
        inner.gammaCal();
        Eigen::MatrixXd result = inner.getgamma();
        int currentRows = gamma.rows();
        gamma.conservativeResize(currentRows + result.rows(), 1);
        gamma.block(currentRows, 0, result.rows(), result.cols()) = result;
    }
    this->gamma = gamma;
}

void MTBDsys::QeCal()
{
    Eigen::MatrixXd Qe = Eigen::MatrixXd::Zero(rpcfObjs.size() * 7, 1);
    for (auto &force : forcesObjs)
    {
        std::vector<Eigen::MatrixXd> Fe = force.getF();
        std::vector<Eigen::MatrixXd> np = force.getnp();
        std::vector<int> rpcfids = force.getRPCFid();
        for (size_t i = 0; i < rpcfids.size(); ++i)
        {
            Eigen::MatrixXd G = rpcfObjs[rpcfids[i]].getG();
            Qe.block(rpcfids[i] * 7, 0, Fe[i].rows(), Fe[i].cols()) = Fe[i];
            Qe.block(rpcfids[i] * 7 + 3, 0, 4, 1) = 2 * G.transpose() * np[i];
        }
    }
    this->Qe = Qe;
}

void MTBDsys::QvCal()
{
    Eigen::MatrixXd Qv = Eigen::MatrixXd::Zero(rpcfObjs.size()*7, 1);
    for (size_t bi = 0; bi < rpcfObjs.size(); ++bi)
    {
        Eigen::MatrixXd dG = rpcfObjs[bi].getdG();
        Qv.block(bi*7+3, 0, 4, 1) = 8*dG.transpose()*rpcfObjs[bi].getInertia()*dG;
    }
    this->Qv = Qv;
}

void MTBDsys::setRPCFObjects(std::vector<RPCF> &objects)
{
    this->rpcfObjs = objects;
}

void MTBDsys::setJointsObjs(std::vector<Joints> &objects)
{
    this->jointsObjs = objects;
}

void MTBDsys::setForcesObjs(std::vector<Force> &objects)
{
    this->forcesObjs = objects;
}

Eigen::MatrixXd MTBDsys::getM() const
{
    return this->M;
}

Eigen::MatrixXd MTBDsys::getQ() const
{
    return this->Q;
}

Eigen::MatrixXd MTBDsys::getPhiq() const
{
    return this->Phiq;
}

Eigen::MatrixXd MTBDsys::getgamma() const
{
    return this->gamma;
}

std::vector<RPCF> MTBDsys::getRPCFObjects() const
{
    return this->rpcfObjs;
}

std::vector<Joints> MTBDsys::getJointsObjs() const
{
    return this->jointsObjs;
}

Eigen::MatrixXd MTBDsys::qgetPos() const
{
    Eigen::MatrixXd q(this->nc, 1);
    for (int bi = 0; bi < this->nb; ++bi)
    {
        q.block(7*bi, 0, 3, 1) = rpcfObjs[bi].getPos();
        q.block(3+7*bi, 0, 4, 1) = rpcfObjs[bi].getRot();
    }
    return q;
}

Eigen::MatrixXd MTBDsys::qgetVel() const
{
    Eigen::MatrixXd dq(this->nc, 1);
    for (int bi = 0; bi < this->nb; ++bi)
    {
        dq.block(7*bi, 0, 3, 1) = rpcfObjs[bi].getVel();
        dq.block(3+7*bi, 0, 4, 1) = rpcfObjs[bi].getdRot();
    }
    return dq;
}

std::vector<int> MTBDsys::getsize() const
{
    std::vector<int> size;
    size.push_back(this->nb);
    size.push_back(this->nc);
    size.push_back(this->nh);
    return size;
}

void MTBDsys::update(Eigen::VectorXd q, Eigen::VectorXd dq)
{
    for (int bi = 0; bi < this->nb; ++bi)
    {
        rpcfObjs[bi].qsetPos(q.block(7*bi, 0, 7, 1));
        rpcfObjs[bi].qsetVel(dq.block(7*bi, 0, 7, 1));
        rpcfObjs[bi].update();
    }
    this->MassCal();
    this->ForceCal();
    this->PhiCal();
    this->PhiqCal();
    this->gammaCal();
    this->QeCal();
    this->QvCal();
}

void MTBDsys::update()
{
    for (int bi = 0; bi < this->nb; ++bi)
    {
        rpcfObjs[bi].update();
    }
    this->MassCal();
    this->ForceCal();
    this->PhiCal();
    this->PhiqCal();
    std::cout << "2" << std::endl;
    this->gammaCal();
    std::cout << "1" << std::endl;
    this->QeCal();
    this->QvCal();
}