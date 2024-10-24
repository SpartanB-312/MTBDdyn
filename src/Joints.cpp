# include "Joints.h"

Joints::Joints() {
    this->Phi = Eigen::MatrixXd::Zero(1, 1);
    this->Phiq = std::vector<Eigen::MatrixXd>();
    this->dPhi = Eigen::MatrixXd::Zero(1, 1);
    this->gamma = Eigen::MatrixXd::Zero(1, 1);
}

Joints::~Joints() {}

void Joints::PhiCal() {
    switch (this->type) {
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 0:
        this->Phi = PhiPCal();
        break;
    }
}

void Joints::PhiqCal() {
    switch (this->type) {
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 0:
        this->Phiq.push_back(PhiPqCal());
        break;
    }
}

void Joints::dPhiCal() {
    switch (this->type) {
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 0:
        this->dPhi = dPhiPCal();
        break;
    }
}

void Joints::gammaCal() {
    switch (this->type) {
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 0:
        this->gamma = gammaPCal();
        break;
    }
}

Eigen::MatrixXd Joints::PhiPCal() {
    Eigen::MatrixXd PhiP(0, 1);
    for (const auto& rpcf : rpcfObjects) {
        Eigen::MatrixXd p = rpcf.getRot();
        Eigen::MatrixXd result = p.transpose() * p - Eigen::MatrixXd::Identity(1, 1);
        int currentRows = PhiP.rows();
        PhiP.conservativeResize(currentRows + 1, Eigen::NoChange);
        PhiP.block(currentRows, 0, 1, 1) = result;
    }
    return PhiP;
}

Eigen::MatrixXd Joints::PhiPqCal() {
    Eigen::MatrixXd PhiPq(0, 0);
    for (const auto& rpcf : rpcfObjects) {
        Eigen::MatrixXd p = rpcf.getRot();
        Eigen::MatrixXd result(1, 7);
        result << 0, 0, 0, 2 * p.transpose();

        int currentRows = PhiPq.rows();
        int currentCols = PhiPq.cols();
        PhiPq.conservativeResize(currentRows + 1, currentCols + 7);
        PhiPq.block(currentRows, currentCols, 1, 7) = result;
    }
    return PhiPq;
}

Eigen::MatrixXd Joints::dPhiPCal()
{
    return Eigen::MatrixXd();// 目前用不到
}

Eigen::MatrixXd Joints::gammaPCal()
{
    Eigen::MatrixXd gammaP(0, 1);
    for (const auto& rpcf : rpcfObjects) {
        Eigen::MatrixXd dp = rpcf.getdRot();
        Eigen::MatrixXd result = 2 * dp.transpose() * dp;
        int currentRows = gammaP.rows();
        gammaP.conservativeResize(currentRows + 1, Eigen::NoChange);
        gammaP.block(currentRows, 0, 1, 1) = result;
    }
    return gammaP;
}

void Joints::setMkObjs(const std::vector<Marker>& objects) {
    this->MkObjs = objects;
}

void Joints::setRPCFObjects(const std::vector<RPCF>& objects) {
    rpcfObjects = objects;
}

void Joints::setType(int JointType) {
    this->type = JointType;
}

std::vector<Marker> Joints::getMkObjs() {
    return this->MkObjs;
}

Eigen::MatrixXd Joints::getBodyData() {
    return Eigen::MatrixXd(); // 暂时用不到
}

Eigen::MatrixXd Joints::getPhi() const {
    return this->Phi;
}

std::vector<Eigen::MatrixXd> Joints::getPhiq() const {
    return this->Phiq;
}

Eigen::MatrixXd Joints::getdPhi() const {
    return this->dPhi;
}

Eigen::MatrixXd Joints::getgamma() const {
    return this->gamma;
}