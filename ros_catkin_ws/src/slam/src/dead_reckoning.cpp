/*
 * dead_reckoning.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: kohei
 */


#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "geographic_msgs/GeoPoint.h"
#include <tf/transform_broadcaster.h>
#include <geodesy/utm.h>
#include "slam/KalmanFilter.hpp"

ros::Publisher rawPositioningResultPub;
ros::Publisher deadReckoingResultPub;
bool isFirstPoint = true;
geodesy::UTMPoint utmFrameOffset;
KalmanFilter kf;
void onGPSFixReceived(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_INFO("Received GPS data lat:%f, long:%f, alt:%f", msg->latitude, msg->longitude, msg->altitude);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  geographic_msgs::GeoPoint geo_pt;
  geo_pt.latitude = msg->latitude;
  geo_pt.longitude = msg->longitude;
  geo_pt.altitude = msg->altitude;
  geodesy::UTMPoint utm_pt(geo_pt);

  if (isFirstPoint)
  {
    utmFrameOffset = utm_pt;
    isFirstPoint = false;
  }
  transform.setOrigin( tf::Vector3(utmFrameOffset.easting, utmFrameOffset.northing, utm_pt.altitude) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/DR_frame"));

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = "world";

  poseStamped.pose.position.x = utm_pt.easting - utmFrameOffset.easting;
  poseStamped.pose.position.y = utm_pt.northing - utmFrameOffset.northing;
  poseStamped.pose.position.z = utm_pt.altitude - utmFrameOffset.altitude;

  ROS_INFO("Raw Pose x:%f, y:%f, z:%f", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);
  rawPositioningResultPub.publish(poseStamped);

  Eigen::VectorXd Y(9);
  double dt = 0.01;
  Y <<  poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z,
      kf.state()(3), kf.state()(4), kf.state()(5), kf.state()(6), kf.state()(7), kf.state()(8);
  ROS_INFO("dt:%f, x:%f, y:%f, z:%f, vx:%f, vy:%f, vz:%f, ax:%f, ay:%f, az:%f", dt,
      kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3), kf.state()(4), kf.state()(5), kf.state()(6)
      , kf.state()(7), kf.state()(8));
  kf.measurementUpdate(Y, dt);
}

KalmanFilter getKalmanFilterInstance()
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
  KalmanFilter kf(dt,
                   A,
                   C,
                   Q,
                   R,
                   P);
  ROS_INFO("P size row:%d col:%d", P.rows(), P.cols());

  Eigen::VectorXd x0(9);
  x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  kf.init(0, x0);

  return kf;
}

void updateSystemDynamicsMxWIthdt(Eigen::MatrixXd& A, double dt)
{
  A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt*dt, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt*dt, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt*dt,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dead_reckoning");

  ros::NodeHandle n;

  ros::Subscriber GPSNavSatFix_sub = n.subscribe("/kitti/oxts/gps/fix", 1000, onGPSFixReceived);

  rawPositioningResultPub = n.advertise<geometry_msgs::PoseStamped>("rawPositioningResult", 1000);
  deadReckoingResultPub = n.advertise<geometry_msgs::PoseStamped>("deadReckoingResult", 1000);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  kf = getKalmanFilterInstance();
  ros::Time lastUpdateTime = ros::Time::now();
  ros::Rate r(200); // 200 hz

  while(ros::ok())
  {
      double dt = ros::Time::now().toSec() - lastUpdateTime.toSec();
      Eigen::MatrixXd A = kf.getA();
      updateSystemDynamicsMxWIthdt(A, dt);
      kf.predictionUpdate(A, dt);
      lastUpdateTime = ros::Time::now();
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.frame_id = "world";
      poseStamped.pose.position.x = kf.state()(0);
      poseStamped.pose.position.y = kf.state()(1);
      poseStamped.pose.position.z = kf.state()(2);
      ROS_INFO("DR Result x:%f, y:%f, z:%f, dt:%f", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z, dt);
      deadReckoingResultPub.publish(poseStamped);
      r.sleep();
  }

  return 0;
}

