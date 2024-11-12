#include "Marker.h"

// 构造函数
Marker::Marker()
{
    // 初始化成员变量
    Mkpos = Eigen::MatrixXd::Zero(3, 1);
    MkAu = Eigen::MatrixXd::Zero(3, 1);
    MkAv = Eigen::MatrixXd::Zero(3, 1);
    MkAw = Eigen::MatrixXd::Zero(3, 1);
}

// 析构函数
Marker::~Marker()
{
    // 这里可以添加析构逻辑，如果有需要的话
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

void Marker::setid(const int &id)
{
    this->id = id;
}

void Marker::setBody(const RPCF &body)
{
    this->Body = body;
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

int Marker::getid() const
{
    return this->id;
}

void Marker::update()
{
    // nothing
}