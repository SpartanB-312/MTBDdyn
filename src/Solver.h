#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <vector>
#include <cmath>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Joints.h"

class Solver {
public:
    Solver();
    ~Solver();

    void preInitialize();

    // Solvers
    void EEuler();
    void EEulerBaumgarte();

    // Matrix calculation
    void MassCal();
    void ForceCal();

    void PhiCal();
    void PhiqCal();
    void gammaCal();

    // set
    void setRPCFObjects(const std::vector<RPCF>& objects);
    void setJointsObjs(const std::vector<Joints>& objects);

    void setTotalTime(const double& TotalTime);
    void setTimeStep(const double& TimeStep);
    void setnStep();

    // get
    std::vector<RPCF> getRPCFObjects() const;
    std::vector<Joints> getJointsObjs() const;
private:
    std::vector<RPCF> rpcfObjects; // Array of RPCF objects
    std::vector<Joints> jointsObjs; // Array of Joints objects

    double Time; // Total time
    double h; // Time step

    int nStep; // Number of steps

    Eigen::MatrixXd M; // Mass matrix
    Eigen::MatrixXd Q; // Force matrix
    Eigen::MatrixXd Qe; // External force matrix
    Eigen::MatrixXd Qv;

    Eigen::MatrixXd Phi;
    Eigen::MatrixXd Phiq;
    Eigen::MatrixXd gamma;
};

#endif // SOLVER_H