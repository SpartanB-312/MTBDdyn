#include "RPCF.h"

// Constructor
RPCF::RPCF() {
    // Initialize matrix with zeros
    this->matrix = Eigen::MatrixXd::Zero(2, 2);
    this->pos = Eigen::MatrixXd::Zero(3, 1);
    this->rot = Eigen::MatrixXd::Zero(4, 1);
    this->mass = 1.0;
    this->inertia = Eigen::MatrixXd::Identity(3, 3);
}

// Destructor
RPCF::~RPCF() {}

// Member function to set matrix values
void RPCF::setMatrix(const Eigen::MatrixXd& mat) {
    this->matrix = mat;
}

void RPCF::setPos(const Eigen::MatrixXd& pos) {
    this->pos = pos;
}

void RPCF::setRot(const Eigen::MatrixXd& rot) {
    this->rot = rot;
}

void RPCF::setMass(const double& mass) {
    this->mass = mass;
}

void RPCF::setInertia(const Eigen::MatrixXd& inertia) {
    this->inertia = inertia;
}

// Member function to get matrix
Eigen::MatrixXd RPCF::getMatrix() const {
    return matrix;
}

Eigen::MatrixXd RPCF::getPos() const {
    return pos;
}

Eigen::MatrixXd RPCF::getRot() const {
    return rot;
}