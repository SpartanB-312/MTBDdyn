#include "Marker.h"

Marker::Marker()
{
    Mkpos = Eigen::MatrixXd::Zero(3, 1);
    MkAu = Eigen::MatrixXd::Zero(3, 1);
    MkAv = Eigen::MatrixXd::Zero(3, 1);
    MkAw = Eigen::MatrixXd::Zero(3, 1);
}

Marker::~Marker()
{
}

void Marker::setGround()
{
    this->Mkpos = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd Auvw = Eigen::MatrixXd::Identity(3, 3);
    setMkAuvw(Auvw);

    this->isGroundSet = true;
    this->BodyId = 0;
}

void Marker::setMkpos(const Eigen::MatrixXd &pos)
{
    this->Mkpos = pos;
}

void Marker::setMkAuvw(const Eigen::MatrixXd &Auvw)
{
    this->MkAu = Auvw.block(0, 0, 3, 1);
    this->MkAv = Auvw.block(0, 1, 3, 1);
    this->MkAw = Auvw.block(0, 2, 3, 1);
}

void Marker::setMkAuvw(const Eigen::MatrixXd &Au, const Eigen::MatrixXd &Av, const Eigen::MatrixXd &Aw)
{
    this->MkAu = Au;
    this->MkAv = Av;
    this->MkAw = Aw;
}

void Marker::setMkAp(const Eigen::MatrixXd &p)
{
    Eigen::MatrixXd Auvw = dynMath::p2A(p);
    this->Mkpos = p;
}

void Marker::setId(const int &id)
{
    this->id = id;
}

void Marker::setBody(const RPCF &body)
{
    this->Body = body;
    this->BodyId = body.getId();
}

Eigen::MatrixXd Marker::getMkpos() const
{
    return this->Mkpos;
}

Eigen::MatrixXd Marker::getMkAuvw() const
{
    Eigen::MatrixXd Auvw(3, 3);
    Auvw.block(0, 0, 3, 1) = this->MkAu;
    Auvw.block(0, 1, 3, 1) = this->MkAv;
    Auvw.block(0, 2, 3, 1) = this->MkAw;
    return Auvw;
}

Eigen::MatrixXd Marker::getMku() const
{
    return this->MkAu;
}

Eigen::MatrixXd Marker::getMkv() const
{
    return this->MkAv;
}

Eigen::MatrixXd Marker::getMkw() const
{
    return this->MkAw;
}

Eigen::MatrixXd Marker::getMkA() const
{
    if (isGroundSet) {
        return Eigen::MatrixXd::Identity(3, 3);
    } else {
        return dynMath::p2A(this->Body.getRot());
    }
}

Eigen::MatrixXd Marker::getMkR() const
{
    if (isGroundSet) {
        return Eigen::MatrixXd::Zero(3, 1);
    } else {
        return this->Body.getPos();
    }
}

Eigen::MatrixXd Marker::getMkRot() const
{
    if (isGroundSet) {
        return Eigen::MatrixXd::Zero(4, 1);
    } else {
        return this->Body.getRot();
    }
}

Eigen::MatrixXd Marker::getMkq() const
{
    if (isGroundSet) {
        Eigen::MatrixXd q = Eigen::MatrixXd::Zero(7, 1);
        q(3, 0) = 1.0;
        return q;
    } else {
        return this->Body.qgetPos();
    }
}

Eigen::MatrixXd Marker::getMkdq() const
{
    if (isGroundSet) {
        return Eigen::MatrixXd::Zero(7, 1);
    } else {
        return this->Body.qgetVel();
    }
}

int Marker::getid() const
{
    return this->id;
}

int Marker::getBodyId() const
{
    return this->BodyId;
}

void Marker::update()
{
    // nothing
}