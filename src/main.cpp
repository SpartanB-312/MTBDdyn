#include <iostream>

#include "RPCF.h"
#include "Solver.h"

int main() {
    // Create an instance of RPCF
    RPCF body1;
    RPCF body2;

    // Set position matrices
    Eigen::MatrixXd pos1(3, 1);
    pos1 << 1, 2, 3;
    body1.setPos(pos1);

    Eigen::MatrixXd pos2(3, 1);
    pos2 << 4, 5, 6;
    body2.setPos(pos2);

    // Create a vector of RPCF objects
    std::vector<RPCF> rpcfObjects = {body1, body2};

    // Create a Solver object
    Solver solver;

    // Set RPCF objects in Solver
    solver.setRPCFObjects(rpcfObjects);

    // Call a function in Solver that uses RPCF objects
    solver.EEulerBaumgarte();

    return 0;
}