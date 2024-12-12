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
    void setMkpos(const Eigen::MatrixXd &pos);
    void setMkAuvw(const Eigen::MatrixXd &Auvw);
    void setMkAuvw(const Eigen::MatrixXd &Au, const Eigen::MatrixXd &Av, const Eigen::MatrixXd &Aw);
    void setMkAp(const Eigen::MatrixXd &p);
    void setid(const int &id);

    void setBody(const RPCF &body);
    // get
    Eigen::MatrixXd getMkpos() const;
    Eigen::MatrixXd getMkAuvw() const;
    Eigen::MatrixXd getMku() const;
    Eigen::MatrixXd getMkv() const;
    Eigen::MatrixXd getMkw() const;
    int getid() const;

    // update
    void update();
private:
    Eigen::MatrixXd Mkpos;
    Eigen::MatrixXd MkAu; // u,v,w
    Eigen::MatrixXd MkAv; // u,v,w
    Eigen::MatrixXd MkAw; // u,v,w
    RPCF Body;
    int id;
};

#endif // MARKER_H