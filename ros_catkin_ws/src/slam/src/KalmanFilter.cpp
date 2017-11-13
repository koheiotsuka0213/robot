/*
 * KalmanFilter.cpp
 *
 *  Created on: Nov 12, 2017
 *      Author: kohei
 */


#include <iostream>
#include <stdexcept>

#include "slam/KalmanFilter.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::predictionUpdate(const Eigen::MatrixXd A, double dt)
{
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  x_hat = x_hat_new;
  P = A*P*A.transpose() + Q;

  t += dt;
}

void KalmanFilter::measurementUpdate(const Eigen::VectorXd& y, double dt)
{
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;
}
