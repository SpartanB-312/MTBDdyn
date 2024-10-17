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

    Eigen::MatrixXd PhiCal();
    Eigen::MatrixXd PhiqCal();
    Eigen::MatrixXd dPhiCal();
    Eigen::MatrixXd gammaCal();

    // set
    void setMkObjs(const std::vector<Marker>& objects);

    // get
    std::vector<Marker> getMkObjs();

    Eigen::MatrixXd getBodyData();
private:
    Eigen::MatrixXd Phi;
    Eigen::MatrixXd Phiq;
    Eigen::MatrixXd dPhi;
    Eigen::MatrixXd gamma;
    std::vector<Marker> MkObjs;
};

#endif // Joints_H