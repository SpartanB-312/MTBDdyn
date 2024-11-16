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
    for (auto &sys : MTBDObjs)
    {
        sys.preInitialize();
        sys.update();
        std::cout << "Pre-Initialize" << std::endl;
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
        Eigen::MatrixXd M = MTBDObjs[0].getM();
        Eigen::MatrixXd Phiq = MTBDObjs[0].getPhiq();
        Eigen::MatrixXd Q = MTBDObjs[0].getQ();
        Eigen::MatrixXd gamma = MTBDObjs[0].getgamma();
        // LHS
        Eigen::MatrixXd LHS(M.rows() + Phiq.rows(), M.cols() + Phiq.rows());
        LHS.block(0, 0, M.rows(), M.cols()) = M;
        LHS.block(0, M.cols(), Phiq.cols(), Phiq.rows()) = Phiq.transpose();
        LHS.block(M.rows(), 0, Phiq.rows(), Phiq.cols()) = Phiq;
        LHS.block(M.rows(), M.cols(), Phiq.rows(), Phiq.rows()) = Eigen::MatrixXd::Zero(Phiq.rows(), Phiq.rows());
        std::cout << "LHS: " << LHS.rows() << "x" << LHS.cols() << std::endl;
        // RHS
        Eigen::MatrixXd RHS(Q.rows() + Phiq.rows(), 1);
        RHS << Q,
            gamma;
        std::cout << LHS << std::endl;
        std::cout << RHS << std::endl;
        // Solve
        Eigen::VectorXd solution = LHS.colPivHouseholderQr().solve(RHS);
        std::cout << "Solution: " << solution << std::endl;
        // Update
        for (auto &sys : MTBDObjs)
        {
            Eigen::MatrixXd ddq = solution;
            Eigen::MatrixXd dq = sys.qgetVel();
            Eigen::MatrixXd q = sys.qgetPos();
            dq = dq + this->h * ddq;
            sys.update();
        }
        // Output
        // 暂无
    }
}

void Solver::EEulerBaumgarte()
{
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
