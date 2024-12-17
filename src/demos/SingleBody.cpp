#include <iostream>

#include <vector>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Force.h"
#include "MTBDsys.h"
#include "Solver.h"

int main()
{
    std::cout << "Start" << std::endl;
    // Create an instance of RPCF
    RPCF body1;

    // Set position matrices
    Eigen::MatrixXd pos1(3, 1);
    pos1 << 1, 1, 1;
    body1.setPos(pos1);

    Eigen::MatrixXd vel1(3, 1);
    vel1 << 0, 0, 0;
    body1.setVel(vel1);

    Eigen::MatrixXd rot1(4, 1);
    rot1 << 1, 0, 0, 0;
    body1.setRot(rot1);

    Eigen::MatrixXd drot1(4, 1);
    drot1 << 0, 0, 0, 0;
    body1.setdRot(drot1);

    body1.setId(1);
    // Create a vector of RPCF objects
    std::vector<RPCF> rpcfObjects = {body1};

    Force force1;
    force1.setRPCFObjects(rpcfObjects);
    force1.setType(0);
    force1.setGravityZ();
    std::vector<Force> forces = {force1};

    MTBDsys sys1;
    sys1.setRPCFObjects(rpcfObjects);
    sys1.setForcesObjs(forces);
    std::vector<MTBDsys> syss = {sys1};

    // Create a Solver object
    Solver solver1;
    // Set RPCF objects in Solver
    solver1.setMTBDsysObjects(syss);

    // Call a function in Solver that uses RPCF objects
    solver1.EEulerBaumgarte();
    solver1.EEuler();

    std::cout << "Finishd" << std::endl;

    return 0;
}