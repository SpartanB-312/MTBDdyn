#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <vector>
#include <cmath>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Joints.h"
#include "Force.h"

class Solver
{
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

    void QeCal();
    void QvCal();

    // set
    void setRPCFObjects(std::vector<RPCF> &objects);
    void setJointsObjs(std::vector<Joints> &objects);
    void setForcesObjs(std::vector<Force> &objects);

    void setTotalTime(double &TotalTime);
    void setTimeStep(double &TimeStep);
    void setnStep();

    // get
    std::vector<RPCF> getRPCFObjects() const;
    std::vector<Joints> getJointsObjs() const;

private:
    std::vector<RPCF> rpcfObjects;  // Array of RPCF objects
    std::vector<Joints> jointsObjs; // Array of Joints objects
    std::vector<Force> forcesObjs; // Array of Forces objects

    double Time; // Total time
    double h;    // Time step

    int nStep; // Number of steps

    Eigen::MatrixXd M;  // Mass matrix
    Eigen::MatrixXd Q;  // Force matrix
    Eigen::MatrixXd Qe; // External force matrix
    Eigen::MatrixXd Qv;

    Eigen::MatrixXd Phi;
    Eigen::MatrixXd Phiq;
    Eigen::MatrixXd gamma;
};

#endif // SOLVER_H