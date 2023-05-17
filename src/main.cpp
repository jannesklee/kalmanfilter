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


// Function to load a matrix from JSON
MatrixXd loadMatrixFromJson(const json& matrixJson) {
    const size_t rows = matrixJson.size();
    const size_t cols = matrixJson[0].size();

    MatrixXd matrix(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            matrix(i, j) = matrixJson[i][j];
        }
    }

    return matrix;
}

// Function to load a vector from JSON
VectorXd loadVectorFromJson(const json& vectorJson) {
    const size_t size = vectorJson.size();
    VectorXd vector(size);
    for (size_t i = 0; i < size; ++i) {
        vector[i] = vectorJson[i];
    }
    return vector;
}

int main() {
    int x_size, P_size, F_size, H_size_dim1, H_size_dim2, R_size, Q_size;

    // Load configuration file
    ifstream ifs("../examples/gps.json");
    json config;
    ifs >> config;
    // Load configuration file
//    ifstream ifs("../examples/ball.json");
//    json config;
//    ifs >> config;

    // Read configuration values
    VectorXd x = loadVectorFromJson(config["state"]);
    MatrixXd P = loadMatrixFromJson(config["covariance"]);
    MatrixXd F = loadMatrixFromJson(config["F"]);
    MatrixXd H = loadMatrixFromJson(config["H"]);
    MatrixXd R = loadMatrixFromJson(config["R"]);
    MatrixXd Q = loadMatrixFromJson(config["Q"]);

    string data_type = config["positioning_data_type"];

    if (data_type == "gps") {
        GPSData gps_data = config["gps_data"];
        KalmanFilter kf;
        kf.init(x, P, F, H, R, Q);
        x << gps_data.lat, gps_data.lon;
    //    kf.update(z);
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
