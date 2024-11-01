#include "RPCF.h"
#include "dynMath.h"

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

// Update related matrixs
void RPCF::update()
{
    // Inner update
    this->GCal();
    this->dGCal();
}

// Matrix calculation
void RPCF::GCal() {
    double e0 = this->rot.coeff(0, 0);
    Eigen::MatrixXd e = this->rot.block(1, 0, 3, 1);
    Eigen::MatrixXd ecross = dynMath::VecCross(e);
    // std::cout << ecross << std::endl;
    Eigen::MatrixXd result(3, 4);
    Eigen::MatrixXd temp;
    temp = - ecross + e0 * Eigen::MatrixXd::Identity(3, 3);
    // std::cout << (e0 * Eigen::MatrixXd::Identity(3, 3)) << std::endl;
    result << -e, temp;
    // std::cout << result << std::endl;
    this->G = result;
}

void RPCF::dGCal()
{
    double de0 = this->drot.coeff(0, 0);
    Eigen::MatrixXd de = this->drot.block(1, 0, 3, 1);
    Eigen::MatrixXd decross = dynMath::VecCross(de);
    Eigen::MatrixXd result(3, 4);
    Eigen::MatrixXd temp;
    temp = - decross + de0 * Eigen::MatrixXd::Identity(3, 3);
    result << -de, temp;
    this->dG = result;
}

void RPCF::setId(const int &id)
{
    this->id = id;
}

// Member function to set matrix values
void RPCF::setMatrix(const Eigen::MatrixXd& mat) {
    this->matrix = mat;
}

void RPCF::setPos(const Eigen::MatrixXd& pos) {
    this->pos = pos;
}

void RPCF::setVel(const Eigen::MatrixXd &vel)
{
    this->vel = vel;
}

void RPCF::setRot(const Eigen::MatrixXd& rot) {
    this->rot = rot;
}

void RPCF::setdRot(const Eigen::MatrixXd& drot)
{
    this->drot = drot;
}

void RPCF::setMass(const double& mass) {
    this->mass = mass;
}

void RPCF::setInertia(const Eigen::MatrixXd& inertia) {
    this->inertia = inertia;
}

void RPCF::qsetPos(const Eigen::MatrixXd &q)
{
    this->pos = q.block(0, 0, 3, 1);
    this->rot = q.block(3, 0, 4, 1);
}

void RPCF::qsetVel(const Eigen::MatrixXd &dq)
{
    this->vel = dq.block(0, 0, 3, 1);
    this->drot = dq.block(3, 0, 4, 1);
}

// Member function to get matrix
Eigen::MatrixXd RPCF::getMatrix() const {
    return matrix;
}

Eigen::MatrixXd RPCF::getPos() const {
    return pos;
}

Eigen::MatrixXd RPCF::getVel() const
{
    return this->vel;
}

Eigen::MatrixXd RPCF::getRot() const {
    return rot;
}

Eigen::MatrixXd RPCF::getdRot() const
{
    return drot;
}

double RPCF::getMass() const {
    return mass;
}

Eigen::MatrixXd RPCF::getInertia() const {
    return inertia;
}

Eigen::MatrixXd RPCF::getG() const {
    return G;
}

Eigen::MatrixXd RPCF::getdG() const
{
    return this->dG;
}

Eigen::MatrixXd RPCF::qgetPos() const
{
    Eigen::MatrixXd q(7, 1);
    q.block(0, 0, 3, 1) = this->pos;
    q.block(3, 0, 4, 1) = this->rot;
    return q;
}

Eigen::MatrixXd RPCF::qgetVel() const
{
    Eigen::MatrixXd dq(7, 1);
    dq.block(0, 0, 3, 1) = this->vel;
    dq.block(3, 0, 4, 1) = this->drot;
    return dq;
}

int RPCF::getId() const
{
    return id;
}
