#include "Solver.h"

// Constructor
Solver::Solver()
{
    this->Time = 1000.0;
    this->h = 0.1;
}

// Destructor
Solver::~Solver() {}

void Solver::preInitialize()
{
    //
    this->setnStep();
    //
    for (auto &sys : MTBDObjs)
    {
        sys.update();
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
        std::cout << "LHS: " << LHS.rows() << "x" << LHS.cols() << std::endl;
        // RHS
        Eigen::MatrixXd RHS(this->Q.rows() + this->Phiq.rows(), 1);
        RHS << this->Q,
            this->gamma;

        // 输出 LHS 和 RHS 的维度以进行调试
        std::cout << "RHS: " << RHS.rows() << "x" << RHS.cols() << std::endl;
        std::cout << Qe << std::endl;
        std::cout << LHS << std::endl;
        std::cout << RHS << std::endl;
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

// set
void Solver::setMTBDsysObjects(std::vector<MTBDsys> &objects)
{
    this->MTBDObjs = objects;
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
