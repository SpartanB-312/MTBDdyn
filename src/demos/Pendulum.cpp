#include <iostream>

#include <vector>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Marker.h"
#include "Joints.h"
#include "Force.h"
#include "MTBDsys.h"
#include "Solver.h"

int main()
{
    std::cout << "Start" << std::endl;
    RPCF Pendulum1;
    Eigen::MatrixXd pos1(3, 1);
    pos1 << 0, 0.5, -sqrt(3) / 2;
    Pendulum1.setPos(pos1);
    Eigen::MatrixXd vel1(3, 1);
    vel1 << 0, 0, 0;
    Pendulum1.setVel(vel1);
    Eigen::MatrixXd rot1(4, 1);
    rot1 << 1, 0, 0, 0;
    Pendulum1.setRot(rot1);
    Eigen::MatrixXd drot1(4, 1);
    drot1 << 0, 0, 0, 0;
    Pendulum1.setdRot(drot1);
    Pendulum1.setId(0);
    std::vector<RPCF> rpcfObjects = {Pendulum1};

    Marker Ground;
    Ground.setGround();
    Ground.setId(0); // Ground id must be 0 and 0 must be the Ground id
    Marker Mk1;
    Eigen::MatrixXd Mk1pos(3, 1);
    Mk1pos << 0, 0, 0;
    Mk1.setMkpos(Mk1pos);
    Mk1.setId(1);
    Mk1.setBody(Pendulum1);
    std::vector<Marker> mkObjects = {Ground, Mk1};

    Joints joint1;
    joint1.setMkObjs(mkObjects);
    joint1.setDist(1);
    std::vector<Joints> joints = {joint1};

    Force force1;
    force1.setRPCFObjects(rpcfObjects);
    force1.setType(0);
    force1.setGravityZ();
    std::vector<Force> forces = {force1};

    MTBDsys sys1;
    sys1.setRPCFObjects(rpcfObjects);
    sys1.setForcesObjs(forces);
    sys1.setJointsObjs(joints);
    std::vector<MTBDsys> syss = {sys1};

    Solver solver1;
    solver1.setMTBDsysObjects(syss);
    solver1.EEuler();

    std::cout << "Finishd" << std::endl;

    return 0;
}