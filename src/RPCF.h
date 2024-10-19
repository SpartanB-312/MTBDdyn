#ifndef RPCF_H
#define RPCF_H

#include "eigen-3.4.0/Eigen/Dense"
#include "dynMath.h"

class RPCF
{
public:
    // Constructor
    RPCF();

    // Destructor
    ~RPCF();

    //
    void Update();

    // Matrix calculation
    void GCal();

    // Member function to set matrix values
    void setMatrix(const Eigen::MatrixXd &mat);

    void setPos(const Eigen::MatrixXd &pos);

    void setVel(const Eigen::MatrixXd &vel);

    void setRot(const Eigen::MatrixXd &rot);

    void setAngVel(const Eigen::MatrixXd &angVel);

    void setdRot(const Eigen::MatrixXd &drot);

    void setMass(const double &mass);

    void setInertia(const Eigen::MatrixXd &inertia);

    // Member function to get matrix
    Eigen::MatrixXd getMatrix() const;

    Eigen::MatrixXd getPos() const;

    Eigen::MatrixXd getVel() const;

    Eigen::MatrixXd getRot() const;

    Eigen::MatrixXd getAngVel() const;

    Eigen::MatrixXd getdRot() const;

    double getMass() const;

    Eigen::MatrixXd getInertia() const;

    Eigen::MatrixXd getG() const;

    // Update related matrixs
    void update();
    // // Struct to hold matrix data
    // struct MatrixData {
    //     Eigen::MatrixXd matrix;
    // };

    // struct PosData {

    // };

    // struct RotData {

    // };

    // struct MassData {

    // };

    // struct InertiaData {

    // };
private:
    // Variables
    Eigen::MatrixXd matrix;
    Eigen::MatrixXd pos;
    Eigen::MatrixXd vel;
    Eigen::MatrixXd rot;
    Eigen::MatrixXd angVel;
    Eigen::MatrixXd drot;
    Eigen::MatrixXd inertia;

    Eigen::MatrixXd G;

    double mass;
    int id;
};

#endif // RPCF_H