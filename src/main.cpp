#include <iostream>
#include "Eigen/Dense"
#include "KalmanFilter.hpp"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

int main() {
    KalmanFilter kf;

    // initial state vector
    VectorXd x(4);
    x << 1, 1, 1, 1;

    // initial covariance matrix
    MatrixXd P(4, 4);
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;

    // state transition matrix
    MatrixXd F(4, 4);
    F << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // measurement matrix
    MatrixXd H(2, 4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    // measurement covariance matrix
    MatrixXd R(2, 2);
    R << 0.0225, 0,
         0, 0.0225;

    // process covariance matrix
    MatrixXd Q(4, 4);
    Q << 1, 0, 1, 0,
         0, 1, 0, 1,
         1, 0, 1, 0,
         0, 1, 0, 1;

    // initialize Kalman filter
    kf.init(x, P, F, H, R, Q);

    // create a vector of measurements
    vector<VectorXd> measurements;
    VectorXd meas(2);
    meas << 1.1, 0.9;
    measurements.push_back(meas);
    meas << 2.2, 1.9;
    measurements.push_back(meas);
    meas << 3.3, 2.8;
    measurements.push_back(meas);
    meas << 4.4, 3.7;
    measurements.push_back(meas);
    meas << 5.5, 4.6;
    measurements.push_back(meas);
    meas << 6.6, 5.5;
    measurements.push_back(meas);
    meas << 7.7, 6.4;
    measurements.push_back(meas);
    meas << 8.8, 7.3;
    measurements.push_back(meas);
    meas << 9.9, 8.2;
    measurements.push_back(meas);
    meas << 10.1, 9.1;
    measurements.push_back(meas);

    // perform Kalman filtering on the measurements
    for (size_t i = 0; i < measurements.size(); ++i) {
        kf.predict();
        kf.update(measurements[i]);
        VectorXd state = kf.getState();
        cout << "State [" << state(0) << ", " << state(1) << ", " << state(2) << ", " << state(3) << "]" << endl;
    }

    return 0;
}

