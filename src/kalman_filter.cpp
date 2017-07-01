#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

static constexpr float kPi = 3.141592654;

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Init(
    VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
    MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    CommonKalmanUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    VectorXd z_pred = tools_.ConvertGeneralMeasurementToRadar(x_);
    VectorXd y = z - z_pred;
    if (y(1) < -1 * kPi) {
        y(1) = y(1) + 2 * kPi;
    } else if (y(1) > kPi) {
        y(1) = y(1) - 2 * kPi;
    }

    CommonKalmanUpdate(y);
}

void KalmanFilter::CommonKalmanUpdate(const Eigen::VectorXd& y) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    //new estimate
    x_ = x_ + (K * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
