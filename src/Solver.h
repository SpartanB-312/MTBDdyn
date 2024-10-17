#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include <vector>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"

class Solver {
public:
    Solver();
    ~Solver();

    void EEulerBaumgarte();

    // Function to set RPCF objects
    void setRPCFObjects(const std::vector<RPCF>& objects);

    // Function to get RPCF objects
    std::vector<RPCF> getRPCFObjects() const;
private:
    std::vector<RPCF> rpcfObjects; // Array of RPCF objects
};

#endif // SOLVER_H