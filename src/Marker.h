#ifndef MARKER_H
#define MARKER_H

#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"

class Marker {
public:
    Marker();
    ~Marker();

private:
    Eigen::MatrixXd Mkpos;
    Eigen::MatrixXd MkA; // u,v,w
    RPCF Body;
};

#endif // MARKER_H