#ifndef MARKER_H
#define MARKER_H

#include "eigen-3.4.0/Eigen/Dense"

#include "RPCF.h"
#include "dynMath.h"

class Marker {
public:
    Marker();
    ~Marker();

    // set
    void setGround();

    void setMkpos(const Eigen::MatrixXd &pos);
    void setMkAuvw(const Eigen::MatrixXd &Auvw);
    void setMkAuvw(const Eigen::MatrixXd &Au, const Eigen::MatrixXd &Av, const Eigen::MatrixXd &Aw);
    void setMkAp(const Eigen::MatrixXd &p);
    void setId(const int &id);

    void setBody(const RPCF &body);
    // get
    Eigen::MatrixXd getMkpos() const;
    Eigen::MatrixXd getMkAuvw() const;
    Eigen::MatrixXd getMku() const;
    Eigen::MatrixXd getMkv() const;
    Eigen::MatrixXd getMkw() const;

    Eigen::MatrixXd getMkA() const; // 名字或许可以改成body
    Eigen::MatrixXd getMkR() const;
    Eigen::MatrixXd getMkRot() const;
    Eigen::MatrixXd getMkq() const;
    Eigen::MatrixXd getMkdq() const;
    int getid() const;
    int getBodyId() const;

    // update
    void update();
private:
    Eigen::MatrixXd Mkpos;
    Eigen::MatrixXd MkAu; // u,v,w
    Eigen::MatrixXd MkAv; // u,v,w
    Eigen::MatrixXd MkAw; // u,v,w

    Eigen::MatrixXd MkA;
    RPCF Body;
    int id;
    int BodyId;

    bool isGroundSet;
};

#endif // MARKER_H