/*
 * PoseEstimation.cpp
 *
 *  Created on: Nov 13, 2017
 *      Author: kohei
 */
#include "ros/ros.h"
#include "slam/PoseEstimation.hpp"
#include "geographic_msgs/GeoPoint.h"

  PoseEstimation::PoseEstimation()
  : mPoseFilter(NULL)
  {
  }

  PoseEstimation::~PoseEstimation()
  {
/*
    pthread_join(mSensorMeasurementUpdateThrd, NULL);
    pthread_mutex_destroy(&mGPSUTMData_mtx);
    pthread_cond_destroy(&mGPSUTMData_cond);

    pthread_join(mPredictionUpdateThrd, NULL);
    pthread_mutex_destroy(&mPoseFilter_mtx);
*/

    delete mPoseFilter;

  }

  void PoseEstimation::init()
  {
    pthread_mutex_init(&mPoseFilter_mtx, 0);
    pthread_create(&mPredictionUpdateThrd, 0, predictionUpdateThreadFunc, this);
    setDefaultKalmanModel();

    pthread_mutex_init(&mGPSUTMData_mtx, 0);
    pthread_cond_init(&mGPSUTMData_cond, 0);
    pthread_create(&mSensorMeasurementUpdateThrd, 0, measurementUpdateThreadFunc, this);
  }

 void PoseEstimation::setPoseFilter(IPoseFilter * poseFilter)
 {
   pthread_mutex_lock(&mPoseFilter_mtx);
   mPoseFilter = poseFilter;
   pthread_mutex_unlock(&mPoseFilter_mtx);
 }

 void PoseEstimation::onGPSFixReceived(const sensor_msgs::NavSatFix::ConstPtr& msg)
 {
   ROS_INFO("Received GPS data lat:%f, long:%f, alt:%f", msg->latitude, msg->longitude, msg->altitude);
     geographic_msgs::GeoPoint geo_pt;
     geo_pt.latitude = msg->latitude;
     geo_pt.longitude = msg->longitude;
     geo_pt.altitude = msg->altitude;
     geodesy::UTMPoint UTMPt(geo_pt);

     if (mOffsetUTMPoint.zone == 0)
     {
       mOffsetUTMPoint = UTMPt;
     }

     geometry_msgs::PoseStamped poseStamped;
     poseStamped.header.frame_id = "world";
     poseStamped.pose.position.x = UTMPt.easting - mOffsetUTMPoint.easting;
     poseStamped.pose.position.y = UTMPt.northing - mOffsetUTMPoint.northing;
     poseStamped.pose.position.z = UTMPt.altitude - mOffsetUTMPoint.altitude;
     ROS_INFO("Raw Pose x:%f, y:%f, z:%f", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);

     pthread_mutex_lock(&mGPSUTMData_mtx);
     mGPSUTMData.push_back(poseStamped);
     ROS_INFO("Notifying measurementUpdateThread of new data..");
     pthread_cond_signal(&mGPSUTMData_cond);
     pthread_mutex_unlock(&mGPSUTMData_mtx);
 }

 void PoseEstimation::measurementUpdateThreadEntry()
 {
   double dt = 0.01; //temporary

   while (ros::ok())
   {
       pthread_mutex_lock(&mGPSUTMData_mtx);
       ROS_INFO("measurementUpdateThread Waiting for new data....");
       pthread_cond_wait(&mGPSUTMData_cond, &mGPSUTMData_mtx);
       size_t queueSize = mGPSUTMData.size();
       for (size_t i = 0; i < queueSize; i++)
       {
         ROS_INFO("Using data #%d of mGPSUTMDataSize:%d", static_cast<int>(i), static_cast<int>(queueSize));
         geometry_msgs::PoseStamped poseStamped = mGPSUTMData[0];
         mGPSUTMData.pop_front();
         Eigen::VectorXd Y(9);
         Y <<  poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z,
             mPoseFilter->state()(3), mPoseFilter->state()(4), mPoseFilter->state()(5), mPoseFilter->state()(6), mPoseFilter->state()(7), mPoseFilter->state()(8);
           ROS_INFO("dt:%f, x:%f, y:%f, z:%f, vx:%f, vy:%f, vz:%f, ax:%f, ay:%f, az:%f", dt,
               mPoseFilter->state()(0), mPoseFilter->state()(1), mPoseFilter->state()(2), mPoseFilter->state()(3), mPoseFilter->state()(4), mPoseFilter->state()(5), mPoseFilter->state()(6)
               , mPoseFilter->state()(7), mPoseFilter->state()(8));
          mPoseFilter->measurementUpdate(Y, dt);
       }
       pthread_mutex_unlock(&mGPSUTMData_mtx);
   }
 }

 void PoseEstimation::predictionUpdateThreadEntry()
 {
   ROS_INFO("predictionUpdate..");

   ros::Time lastUpdateTime = ros::Time::now();
   ros::Rate r(200); // 200 hz
   while(ros::ok())
   {
     double dt = ros::Time::now().toSec() - lastUpdateTime.toSec();

     pthread_mutex_lock(&mPoseFilter_mtx);
     Eigen::MatrixXd A = mPoseFilter->getA();
     updateSystemDynamicsMxWIthdt(A, dt);
     mPoseFilter->predictionUpdate(A, dt);
     pthread_mutex_unlock(&mPoseFilter_mtx);
     lastUpdateTime = ros::Time::now();
     r.sleep();
   }
 }

 Eigen::VectorXd PoseEstimation::getState()
 {
   pthread_mutex_lock(&mPoseFilter_mtx);
   Eigen::VectorXd state = mPoseFilter->state();
   pthread_mutex_unlock(&mPoseFilter_mtx);
   return state;
 }

 void PoseEstimation::updateSystemDynamicsMxWIthdt(Eigen::MatrixXd& A, double dt)
 {
   A(0, 3) = dt;
   A(0, 6) = 1/2.0*dt*dt;
   A(1, 4) = dt;
   A(1, 7) = 1/2.0*dt*dt;
   A(2, 5) = dt;
   A(2, 8) = 1/2.0*dt*dt;

   A(3, 6) = dt;
   A(4, 7) = dt;
   A(5, 8) = dt;
  }

 void PoseEstimation::setDefaultKalmanModel()
 {
   double dt = 0.005;

     /* Create a Kalman filter with the specified matrices.
      *   A - System dynamics matrix
      *   C - Output matrix
      *   Q - Process noise covariance
      *   R - Measurement noise covariance
      *   P - Estimate error covariance
      */

     Eigen::MatrixXd A(9, 9);
    A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt*dt, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt*dt, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt*dt,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

     Eigen::MatrixXd C(9, 9);
     C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

     Eigen::MatrixXd Q(9, 9);
    Q << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;


     Eigen::MatrixXd R(9, 9);
     R << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

     Eigen::MatrixXd P(9, 9);
     Eigen::MatrixXd G(9, 1);
     G << 1/2.0*dt*dt, 1/2.0*dt*dt, 1/2.0*dt*dt, dt, dt, dt, 1.0, 1.0, 1.0;
     double accProcessNoise = 0.1;
     P = G * G.transpose() * accProcessNoise * accProcessNoise;
     pthread_mutex_lock(&mPoseFilter_mtx);
     mPoseFilter = new KalmanFilter(dt,
                      A,
                      C,
                      Q,
                      R,
                      P);
//     ROS_INFO("P size row:%d col:%d", P.rows(), P.cols());
     Eigen::VectorXd x0(9);
     x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
     mPoseFilter->init(0, x0);
     pthread_mutex_unlock(&mPoseFilter_mtx);
 }

