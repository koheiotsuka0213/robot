/*
 * PoseEstimation.hpp
 *
 *  Created on: Nov 13, 2017
 *      Author: kohei
 */

#ifndef POSEESTIMATION_HPP_
#define POSEESTIMATION_HPP_
#pragma once

#include <geodesy/utm.h>
#include <pthread.h>
#include "slam/KalmanFilter.hpp"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

class PoseEstimation {

public:
  PoseEstimation();
 ~PoseEstimation();

 void init();

 void setPoseFilter(IPoseFilter * poseFilter);
 void onGPSFixReceived(const sensor_msgs::NavSatFix::ConstPtr& msg);
 void onGPSTwistReceived(const geometry_msgs::TwistStamped::ConstPtr& msg);

 void measurementUpdateThreadEntry();
 static void * measurementUpdateThreadFunc(void * This) {((PoseEstimation *)This)->measurementUpdateThreadEntry(); return NULL;}
 void predictionUpdateThreadEntry();
 static void * predictionUpdateThreadFunc(void * This) {((PoseEstimation *)This)->predictionUpdateThreadEntry(); return NULL;}

 Eigen::VectorXd getState();

private:
 void updateSystemDynamicsMxWIthdt(Eigen::MatrixXd& A, double dt);
 void setDefaultKalmanModel();
IPoseFilter * mPoseFilter;
pthread_t mSensorMeasurementUpdateThrd;
pthread_t mPredictionUpdateThrd;

pthread_cond_t mMeasurementUpdate_cond;
pthread_mutex_t  mMeasurementUpdate_mtx;

geodesy::UTMPoint mOffsetUTMPoint;
pthread_mutex_t  mGPSUTMData_mtx;
std::deque<geometry_msgs::PoseStamped> mGPSUTMData;

pthread_mutex_t  mGPSTwistData_mtx;
pthread_cond_t mGPSTwistData_cond;
std::deque<geometry_msgs::TwistStamped> mGPSTwistData;

pthread_mutex_t  mPoseFilter_mtx;


};
#endif /* POSEESTIMATION_HPP_ */
