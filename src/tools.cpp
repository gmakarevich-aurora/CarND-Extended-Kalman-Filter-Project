#include <iostream>
#include <math.h>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

static constexpr float kMinimumNonZero = 0.0000001;

Tools::Tools() {
}

Tools::~Tools() {
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
        const vector<VectorXd> &ground_truth) const {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
            || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) const {
    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);


    float c1 = px*px+py*py;
    // Check division by zero
    if(fabs(c1) < kMinimumNonZero){
        c1 = kMinimumNonZero;
    }

    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    //compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
           -(py / c1), (px / c1), 0, 0,
           py * (vx * py - vy * px) / c3,
           px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}

VectorXd Tools::ConvertRadarMeasurementToGeneral(
        const VectorXd& measurement) const {
    float rho = measurement(0);
    float phi = measurement(1);
    float rho_dot = measurement(2);

    float x = rho * cos(phi);
    float y = rho * sin(phi);
    float vx = rho_dot * cos(phi);
    float vy = rho_dot * sin(phi);

    VectorXd r(4);
    r << x, y, vx, vy;
    return r;
}

VectorXd Tools::ConvertGeneralMeasurementToRadar(
        const VectorXd& measurement) const {
    float rho = sqrt(measurement(0) * measurement(0) +
                        measurement(1) * measurement(1));
    float theta = atan2(measurement(1), measurement(0));
    float div = fabs(rho) < kMinimumNonZero ? kMinimumNonZero : rho;
    float rho_dot = (measurement(0) * measurement(2) +
                     measurement(1) * measurement(3)) / rho;

    VectorXd r(3);
    r << rho, theta, rho_dot;
    return r;
}
