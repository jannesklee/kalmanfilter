#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include "KalmanFilter.hpp"
#include <nlohmann/json.hpp>
#include "gps_data.hpp"


using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using json = nlohmann::json;

int main() {
    // Load configuration file
//    ifstream ifs("../examples/gps.json");
//    json config;
//    ifs >> config;
    // Load configuration file
    ifstream ifs("../examples/ball.json");
    json config;
    ifs >> config;

    // Read configuration values
    VectorXd x(config["state"].size());
    for (int i = 0; i < config["state"].size(); ++i) {
        x[i] = config["state"][i];
    }

    MatrixXd P(4, 4);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P(i, j) = config["covariance"][i][j];
        }
    }

    MatrixXd F(4, 4);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            F(i, j) = config["F"][i][j];
        }
    }

    MatrixXd H(2, 4);
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            H(i, j) = config["H"][i][j];
        }
    }

    MatrixXd R(2, 2);
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            R(i, j) = config["R"][i][j];
        }
    }

    MatrixXd Q(4, 4);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Q(i, j) = config["Q"][i][j];
        }
    }


    cout << config["Q"].size() << endl;

    string data_type = config["positioning_data_type"];

    if (data_type == "gps") {
//        GPSData gps_data = config["gps_data"];
        KalmanFilter kf;
//        kf.init(x, P, F, H, R, Q);
//        z << gps_data.lat, gps_data.lon;
//        kf.update(z);
        // ...
    }
    else if (data_type == "lidar") {
        // handle lidar data
    }
    else if (data_type == "radar") {
        // handle radar data
    }
    else {
        cerr << "Unknown positioning data type: " << data_type << endl;
        return 1;
    }

    return 0;
}
