#ifndef Joints_H
#define Joints_H

#include <vector>
#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "Marker.h"

class Joints {
public:
    Joints();
    ~Joints();

    void PhiCal();
    void PhiqCal();
    void dPhiCal();
    void gammaCal();

    Eigen::MatrixXd PhiPCal(); // 暂时用物体的，以后换成CM
    Eigen::MatrixXd PhiPqCal();
    Eigen::MatrixXd dPhiPCal();
    Eigen::MatrixXd gammaPCal();

    // set
    void setMkObjs(const std::vector<Marker>& objects);
    void setRPCFObjects(const std::vector<RPCF>& objects);

    void setType(int type);

    // get
    std::vector<Marker> getMkObjs();

    Eigen::MatrixXd getBodyData();

    Eigen::MatrixXd getPhi() const;
    std::vector<Eigen::MatrixXd> getPhiq() const;
    Eigen::MatrixXd getdPhi() const;
    Eigen::MatrixXd getgamma() const;
private:
    Eigen::MatrixXd Phi;
    std::vector<Eigen::MatrixXd> Phiq;
    Eigen::MatrixXd dPhi;
    Eigen::MatrixXd gamma;
    std::vector<Marker> MkObjs;
    std::vector<RPCF> rpcfObjects;
    int type;
};

#endif // Joints_H