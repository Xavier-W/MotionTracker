#include "motion_tracker.h"
#include <Eigen/Dense>
#include <iostream>

// 内部实现类，用于隐藏具体的滤波器实现细节
class MotionTracker::Impl {
public:
    Eigen::VectorXd x;  // 状态向量
    Eigen::MatrixXd P;  // 状态协方差矩阵
    Eigen::MatrixXd Q;  // 过程噪声协方差矩阵
    Eigen::MatrixXd R;  // 测量噪声协方差矩阵
    Eigen::MatrixXd H;  // 测量矩阵
    Eigen::MatrixXd F;  // 状态转移矩阵

    bool isInitialized;

    Impl(int stateNum, int measureNum) : isInitialized(false) {
        // 初始化状态向量和协方差矩阵
        x = Eigen::VectorXd::Zero(stateNum);
        P = Eigen::MatrixXd::Identity(stateNum, stateNum) * 1000;
        Q = Eigen::MatrixXd::Identity(stateNum, stateNum) * 1e-1;
        R = Eigen::MatrixXd::Identity(measureNum, measureNum) * 8;
        H = Eigen::MatrixXd::Zero(measureNum, stateNum);
        F = Eigen::MatrixXd::Identity(stateNum, stateNum);

        // 设置 H 矩阵为测量矩阵
        H(0, 0) = 1;
        H(1, 1) = 1;

        // 状态转移矩阵 F 的初始化
        F(0, 2) = 1;
        F(1, 3) = 1;
    }

    void predict() {
        // 扩展卡尔曼滤波器的预测步骤
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::VectorXd& z) {
        // 扩展卡尔曼滤波器的更新步骤
        Eigen::VectorXd y = z - H * x;
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = P - K * H * P;
    }
};

MotionTracker::MotionTracker(int stateNum, int measureNum)
    : pImpl(new Impl(stateNum, measureNum)) {}

void MotionTracker::init(const Point2D& firstMeasuredPoint) {
    pImpl->x[0] = firstMeasuredPoint.x;
    pImpl->x[1] = firstMeasuredPoint.y;
    pImpl->isInitialized = true;
}

Point2D MotionTracker::track(const Point2D& measuredPoint) {
    if (!pImpl->isInitialized) {
        throw std::runtime_error("MotionTracker is not initialized.");
    }

    Eigen::VectorXd measurement(2);
    measurement << measuredPoint.x, measuredPoint.y;

    pImpl->predict();
    pImpl->update(measurement);

    return Point2D(pImpl->x[0], pImpl->x[1]);
}
