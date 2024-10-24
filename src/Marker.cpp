#include "Marker.h"

// 构造函数
Marker::Marker() {
    // 初始化成员变量
    Mkpos = Eigen::MatrixXd::Zero(3, 1);
    MkA = Eigen::MatrixXd::Zero(3, 1);
}

// 析构函数
Marker::~Marker() {
    // 这里可以添加析构逻辑，如果有需要的话
}