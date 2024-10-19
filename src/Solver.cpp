#include "Solver.h"

// Constructor
Solver::Solver() {}

// Destructor
Solver::~Solver() {}

void Solver::preInitialize()
{
    //
    this->setnStep();

    //
    std::vector<Joints> InnerJoints;
    for (const auto &rpcf : rpcfObjects)
    {
        Joints Inner;
        Inner.setRPCFObjects(rpcfObjects);
        Inner.setType(0);

        InnerJoints.push_back(Inner);
    }
    this->jointsObjs.push_back(InnerJoints);
}

void Solver::EEuler()
{
}

void Solver::EEulerBaumgarte()
{
    for (const auto &rpcf : rpcfObjects)
    {
        // Perform operations on each RPCF object
        std::cout << "Position Matrix:\n"
                  << rpcf.getPos() << std::endl;
    }
}

// Matrix calculation
void Solver::MassCal()
{
    Eigen::MatrixXd Mass(0, 0);
    for (const auto &rpcf : rpcfObjects)
    {
        double mass = rpcf.getMass();
        Eigen::MatrixXd inertia = rpcf.getInertia();
        Eigen::MatrixXd G = rpcf.getG();

        // 构建 m*I3
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d mI3 = mass * I3;

        // 构建 4*G'*J*G
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(4, 4); // 示例 J 矩阵
        Eigen::MatrixXd GtJG = 4 * G.transpose() * J * G;

        // 构建 M 矩阵
        Eigen::MatrixXd result(7, 7);
        result << mI3, Eigen::MatrixXd::Zero(3, 4),
            Eigen::MatrixXd::Zero(4, 3), GtJG;

        // 将 M 矩阵添加到 Mass 矩阵中
        int currentRows = Mass.rows();
        int currentCols = Mass.cols();
        Mass.conservativeResize(currentRows + 7, currentCols + 7);
        Mass.block(currentRows, currentCols, 7, 7) = result;
    }
}

void Solver::ForceCal()
{
}

void Solver::PhiCal()
{
    Eigen::MatrixXd Phi(0, 0);
    for (const auto &joints : jointsObjs)
    {
        Eigen::MatrixXd result = joints.PhiCal();
        int currentRows = Phi.rows();
        Phi.conservativeResize(currentRows + result.rows(), Eigen::NoChange);
        Phi.block(currentRows, 0, result.rows(), result.cols()) = result;
    }
}

void Solver::PhiqCal()
{
    int nb = rpcfObjects.size();
    Eigen::MatrixXd Phiq(0, nb);
    for (const auto &joints : jointsObjs)
    {
        std::vector<Eigen::MatrixXd> Phiqs = joints.getPhiq();
        int currentRows = Phiq.rows();
        Phiq.conservativeResize(currentRows + Phiqs[0].rows(), Eigen::NoChange);
        for (int ib = 0; ib < Phiqs.size(); ++ib)
        {
            Eigen::MatrixXd result = Phiqs[ib];
            Phiq.block(currentRows, 7*ib+1, result.rows(), result.cols()) = result;
        }
    }
}

void Solver::gammaCal()
{
}

// set
void Solver::setRPCFObjects(const std::vector<RPCF> &objects)
{
    this->rpcfObjects = objects;
}

void Solver::setJointsObjs(const std::vector<Joints> &objects)
{
    this->jointsObjs = objects;
}

void Solver::setTotalTime(const double &TotalTime)
{
    this->Time = TotalTime;
}

void Solver::setTimeStep(const double &TimeStep)
{
    this->h = TimeStep;
}

void Solver::setnStep()
{
    this->nStep = static_cast<int>(std::ceil(this->Time / this->h));
}

// get
std::vector<RPCF> Solver::getRPCFObjects() const
{
    return this->rpcfObjects;
}

std::vector<Joints> Solver::getJointsObjs() const
{
    return this->jointsObjs;
}