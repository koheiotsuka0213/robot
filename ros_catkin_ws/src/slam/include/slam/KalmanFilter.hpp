/*
 * KalmanFilter.hpp
 *
 *  Created on: Nov 12, 2017
 *      Author: kohei
 */

#ifndef KALMANFILTER_HPP_
#define KALMANFILTER_HPP_

#include "slam/IPoseFilter.hpp"

#pragma once

class KalmanFilter : public IPoseFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  /**
  * Create a blank estimator.
  */
  KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0);

  void predictionUpdate(const Eigen::MatrixXd A, double dt);

  void measurementUpdate(const Eigen::VectorXd& y, double dt);

  EFilterMethod getPoseFilterMethod() {return KALMAN_FILTER; };
  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };
  Eigen::MatrixXd getA() {return A;};


private:

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};

#endif /* KALMANFILTER_HPP_ */
