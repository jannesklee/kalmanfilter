#include <iostream>
#include "KalmanFilter.hpp"

using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
    x_ = x_in; // state vector
    P_ = P_in; // state covariance matrix
    F_ = F_in; // state transition matrix
    H_ = H_in; // measurement matrix
    R_ = R_in; // measurement covariance matrix
    Q_ = Q_in; // process covariance matrix
    is_initialized_ = true;
}

void KalmanFilter::predict() {
    x_ = F_ * x_;  // + B*u;?
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::getState() {
    return x_;
}

