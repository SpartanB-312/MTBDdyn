#ifndef Joints_H
#define Joints_H

#include <vector>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Marker.h"

class Joints
{
public:
    Joints();
    ~Joints();

    void PhiCal();
    void PhiqCal();
    void dPhiCal();
    void gammaCal();

    Eigen::MatrixXd PhiPCal();
    Eigen::MatrixXd PhiPqCal();
    Eigen::MatrixXd dPhiPCal();
    Eigen::MatrixXd gammaPCal();

    // Phi
    Eigen::MatrixXd PhiDistCal();
    Eigen::MatrixXd PhiD1Cal();
    Eigen::MatrixXd PhiD2Cal();

    // Phiq
    Eigen::MatrixXd PhiDistqCal();


    // dPhi
    Eigen::MatrixXd dPhiDistCal();


    // gammaP
    Eigen::MatrixXd gammaDistCal();


    // set
    void setMkObjs(const std::vector<Marker> &objects);
    void setRPCFObjects(const std::vector<RPCF> &objects);

    void setType(int type);

    // get
    std::vector<Marker> getMkObjs();
    std::vector<int> getRPCFid();

    Eigen::MatrixXd getBodyData();

    Eigen::MatrixXd getPhi() const;
    std::vector<Eigen::MatrixXd> getPhiq() const;
    Eigen::MatrixXd getdPhi() const;
    Eigen::MatrixXd getgamma() const;

    int getnhj() const;
private:
    Eigen::MatrixXd Phi;
    std::vector<Eigen::MatrixXd> Phiq;
    Eigen::MatrixXd dPhi;
    Eigen::MatrixXd gamma;
    std::vector<Marker> MkObjs;
    std::vector<RPCF> rpcfObjects;
    int type;
    int nhj;
};

#endif // Joints_H