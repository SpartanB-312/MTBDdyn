#ifndef _MTBDSYS_H
#define _MTBDSYS_H

#include <iostream>
#include <vector>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Marker.h"
#include "Joints.h"
#include "Force.h"

class MTBDsys
{
public:
    MTBDsys();
    ~MTBDsys();

    void preInitialize();

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

    // get
    Eigen::MatrixXd getM() const;
    Eigen::MatrixXd getQ() const;
    Eigen::MatrixXd getPhiq() const;
    Eigen::MatrixXd getgamma() const;

    std::vector<RPCF> getRPCFObjects() const;
    std::vector<Joints> getJointsObjs() const;

    Eigen::MatrixXd qgetPos() const;
    Eigen::MatrixXd qgetVel() const;

    std::vector<int> getsize() const;
    // update
    void update(Eigen::VectorXd q, Eigen::VectorXd dq);
    void update();
private:
    std::vector<RPCF> rpcfObjs;
    std::vector<Joints> jointsObjs;
    std::vector<Joints> InnerPObjs;
    std::vector<Force> forcesObjs;

    Eigen::MatrixXd M;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd Qe;
    Eigen::MatrixXd Qv;

    Eigen::MatrixXd Phi;
    Eigen::MatrixXd Phiq;
    Eigen::MatrixXd gamma;

    Eigen::MatrixXd dPhi;

    int nb;
    int nc;
    int nhI;
    int nhO;
    int nh;
};
#endif // _MTBDSYS_H