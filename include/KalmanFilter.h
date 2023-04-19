#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter();
    ~KalmanFilter();
    void init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
              Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);
    void predict();
    void update(const Eigen::VectorXd &z);
    Eigen::VectorXd getState();
private:
    bool is_initialized_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd Q_;
};

#endif /* KALMANFILTER_H */

