#include "Solver.h"

// Constructor
Solver::Solver()
{
    this->Time = 1.0;
    this->h = 0.1;
}

// Destructor
Solver::~Solver() {}

void Solver::preInitialize()
{
    //
    this->setnStep();

    //
    std::vector<Joints> InnerJoints;
    for (auto &rpcf : rpcfObjects)
    {
        Joints Inner;
        Inner.setRPCFObjects(rpcfObjects);
        Inner.setType(0);

        InnerJoints.push_back(Inner);
    }
    for (auto &innerJoint : InnerJoints) {
        this->jointsObjs.push_back(innerJoint);
    }

    for (auto &rpcf : rpcfObjects)
    {
        rpcf.update();
    }
}

void Solver::EEuler()
{
    //
    this->preInitialize();
    //
    std::cout << "Total time steps: " << this->nStep << std::endl; 
    //
    for (int ti = 0; ti < this->nStep; ++ti)
    {
        // Calculate
        std::cout << "Time Step: " << ti << std::endl;
        this->MassCal();
        this->PhiqCal();
        std::cout << "Time Step: " << ti << std::endl;
        this->ForceCal();
        this->gammaCal();
        // LHS
        Eigen::MatrixXd LHS(this->M.rows() + this->Phiq.rows(), this->M.cols() + this->Phiq.rows());
        LHS << this->M, this->Phiq.transpose(),
            this->Phiq, Eigen::MatrixXd::Zero(this->Phiq.rows(), this->Phiq.rows());

        // RHS
        Eigen::MatrixXd RHS(this->Q.rows() + this->Phiq.rows(), 1);
        RHS << this->Q,
            this->gamma;

        // 输出 LHS 和 RHS 的维度以进行调试
        std::cout << "LHS: " << LHS.rows() << "x" << LHS.cols() << std::endl;
        std::cout << "RHS: " << RHS.rows() << "x" << RHS.cols() << std::endl;

        std::cout << LHS << std::endl;

        // std::cout << "RHS Matrix:\n"
        //             << RHS << std::endl;
        // Solve
        Eigen::VectorXd solution = LHS.colPivHouseholderQr().solve(RHS);
        std::cout << "Solution: " << solution << std::endl;
        // Update
        int numObjects = rpcfObjects.size();
        int ddqSize = 7; // 每个对象的加速度大小
        for (int bi = 0; bi < numObjects; ++bi) {
            Eigen::VectorXd ddq = solution.segment(bi * ddqSize, ddqSize); // 提取每个对象的加速度
            Eigen::MatrixXd dq = rpcfObjects[bi].qgetVel(); // 获取当前速度
            Eigen::MatrixXd q = rpcfObjects[bi].qgetPos(); // 获取当前位置
            dq = dq + this->h * ddq; // 更新速度
            q = q + this->h * dq; // 更新位置
            rpcfObjects[bi].qsetVel(dq); // 设置更新后的速度
            rpcfObjects[bi].qsetPos(q); // 设置更新后的位置
            std::cout << "Position Matrix:\n"
                      << rpcfObjects[bi].getPos() << std::endl;
            std::cout << "Rotation Matrix:\n"
                      << rpcfObjects[bi].getRot() << std::endl;

            // Inner update
            rpcfObjects[bi].update();
        }
        // Output
        // 暂无
    }
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
    Eigen::MatrixXd Mass;
    for (const auto &rpcf : rpcfObjects)
    {
        double mass = rpcf.getMass();
        Eigen::MatrixXd inertia = rpcf.getInertia();
        Eigen::MatrixXd G = rpcf.getG();

        // 构建 m*I3
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d mI3 = mass * I3;

        // 构建 4*G'*J*G
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(3, 3);
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
    this->M = Mass;
}

void Solver::ForceCal()
{
    int numObjects = rpcfObjects.size();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7 * numObjects, 1);
    this->Q = Q;
}

void Solver::PhiCal()
{
    Eigen::MatrixXd Phi;
    for (auto &joints : jointsObjs)
    {
        joints.PhiCal();
        Eigen::MatrixXd result = joints.getPhi();
        int currentRows = Phi.rows();
        Phi.conservativeResize(currentRows + result.rows(), Eigen::NoChange);
        Phi.block(currentRows, 0, result.rows(), result.cols()) = result;
    }
}

void Solver::PhiqCal()
{
    int nb = rpcfObjects.size();
    Eigen::MatrixXd phiq;
    for (auto &joints : jointsObjs)
    {
        joints.PhiqCal();
        std::vector<Eigen::MatrixXd> Phiqs = joints.getPhiq();
        int currentRows = phiq.rows();
        phiq.conservativeResize(1, 7);
        for (int bi = 0; bi < Phiqs.size(); ++bi)
        {
            // std::cout << Phiqs.size() << std::endl;
            Eigen::MatrixXd result = Phiqs[bi];
            // std::cout << Phiqs[bi] << std::endl;
            phiq.block(0, 0 , result.rows(), result.cols()) = result;
            // std::cout << "2.5" << std::endl;
        }
        // std::cout << "3" << std::endl;
    }
    // std::cout << "4" << std::endl;
    this->Phiq = phiq;
}

void Solver::gammaCal()
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
    this->gamma = gamma;
}

// set
void Solver::setRPCFObjects(std::vector<RPCF> &objects)
{
    this->rpcfObjects = objects;
}

void Solver::setJointsObjs(std::vector<Joints> &objects)
{
    this->jointsObjs = objects;
}

void Solver::setTotalTime(double &TotalTime)
{
    this->Time = TotalTime;
}

void Solver::setTimeStep(double &TimeStep)
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