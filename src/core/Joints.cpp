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
    case 1: // distance
        this->Phi = PhiDistCal();
        break;
    case 2: // spherical
        break;
    case 3: // dot 1
        break;
    case 4: // dot 2
        break;
    case 5: // cylindrical
        break;
    case 6: // revolute
        break;
    case 7: // translational
        break;
    case 8: // universal
        break;
    case 0:
        this->Phi = PhiPCal();
        break;
    }
}

void Joints::PhiqCal() {
    switch (this->type) {
    case 1:
        this->Phiq = PhiDistqCal();
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
        this->Phiq={PhiPqCal()};
        break;
    }
}

void Joints::dPhiCal() {
    switch (this->type) {
    case 1:
        this->dPhi = dPhiDistCal();
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
        this->gamma = gammaDistCal();
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
    for (const auto& rpcf : rpcfObjects) { // 内部约束仍然由RPCF计算，而不是CM
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

// Phi
Eigen::MatrixXd Joints::PhiDistCal()
{
    Eigen::MatrixXd Ai = MkObjs[0].getMkA(); // 以后把计算写在update中
    Eigen::MatrixXd Aj = MkObjs[1].getMkA();
    Eigen::MatrixXd ri = MkObjs[0].getMkR();
    Eigen::MatrixXd rj = MkObjs[1].getMkR();
    Eigen::MatrixXd si = MkObjs[0].getMkpos();
    Eigen::MatrixXd sj = MkObjs[1].getMkpos();

    Eigen::MatrixXd dij = rj + Aj * sj - ri - Ai * si;
    return (dij.transpose() * dij - Eigen::MatrixXd::Identity(1, 1) * dist * dist) / 2;
}

std::vector<Eigen::MatrixXd> Joints::PhiDistqCal()
{
    Eigen::MatrixXd Ai = MkObjs[0].getMkA(); // 以后把计算写在update中
    Eigen::MatrixXd Aj = MkObjs[1].getMkA();
    Eigen::MatrixXd ri = MkObjs[0].getMkR();
    Eigen::MatrixXd rj = MkObjs[1].getMkR();
    Eigen::MatrixXd si = MkObjs[0].getMkpos();
    Eigen::MatrixXd sj = MkObjs[1].getMkpos();
    Eigen::MatrixXd qi = MkObjs[0].getMkq();
    Eigen::MatrixXd qj = MkObjs[1].getMkq();

    Eigen::MatrixXd dij = rj + Aj * sj - ri - Ai * si;
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd epdBi = dynMath::epdBCal(qi, si);
    Eigen::MatrixXd epdBj = dynMath::epdBCal(qj, sj);
    Eigen::MatrixXd Phiqi;
    Eigen::MatrixXd Phiqj;
    Phiqi.conservativeResize(1, 7);
    Phiqi << -dij.transpose() * I3, -dij.transpose() * epdBi;
    Phiqj.conservativeResize(1, 7);
    Phiqj << dij.transpose() * I3, dij.transpose() * epdBj;

    return {Phiqi, Phiqj};
}

Eigen::MatrixXd Joints::gammaDistCal()
{
    Eigen::MatrixXd qi = MkObjs[0].getMkq();
    Eigen::MatrixXd qj = MkObjs[1].getMkq();
    Eigen::MatrixXd dqi = MkObjs[0].getMkdq();
    Eigen::MatrixXd dqj = MkObjs[1].getMkdq();
    Eigen::MatrixXd si = MkObjs[0].getMkpos();
    Eigen::MatrixXd sj = MkObjs[1].getMkpos();

    Eigen::MatrixXd chii = dqi;
    Eigen::MatrixXd chij = dqj;
    Eigen::MatrixXd dqij(14, 1);
    dqij << dqi,
            dqj;

    Eigen::MatrixXd P2 = dynMath::P2DistCal(qi, qj, chii, chij, si, sj, dist);
    Eigen::MatrixXd gamma = P2 * dqij;

    return gamma;
}

// set
void Joints::setMkObjs(const std::vector<Marker>& objects) {
    this->MkObjs = objects;
}

void Joints::setRPCFObjects(const std::vector<RPCF>& objects) {
    this->rpcfObjects = objects;
}

void Joints::setType(int JointType) {
    this->type = JointType;
    switch (this->type) {
    case 1: // distance
        this->nhj = 1;
        break;
    case 2: // spherical
        break;
    case 3: // dot 1
        break;
    case 4: // dot 2
        break;
    case 5: // cylindrical
        break;
    case 6: // revolute
        break;
    case 7: // translational
        break;
    case 8: // universal
        break;
    case 0:
        this->nhj = this->rpcfObjects.size();
        this->Phi = PhiPCal();
        break;
    }
}

void Joints::setDist(int dist)
{
    this->dist = dist;
    setType(1);
}

std::vector<Marker> Joints::getMkObjs() {
    return this->MkObjs;
}

std::vector<int> Joints::getRPCFid() {
    std::vector<int> rpcfIds;
    for (const auto& rpcf : rpcfObjects) {
        rpcfIds.push_back(rpcf.getId());
    }
    return rpcfIds;
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

int Joints::getnhj() const {
    return this->nhj;
}