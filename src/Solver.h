#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <vector>
#include <cmath>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Joints.h"
#include "Force.h"
#include "MTBDsys.h"

class Solver
{
public:
    Solver();
    ~Solver();

    void preInitialize();

    // Solvers
    void EEuler();
    void EEulerBaumgarte();

    // set
    void setMTBDsysObjects(std::vector<MTBDsys> &objects);

    void setTotalTime(double &TotalTime);
    void setTimeStep(double &TimeStep);
    void setnStep();


private:
    std::vector<MTBDsys> MTBDObjs;

    double Time; // Total time
    double h;    // Time step

    int nStep; // Number of steps
};

#endif // SOLVER_H