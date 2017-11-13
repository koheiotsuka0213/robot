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
#include "slam/PoseEstimation.hpp"

ros::Publisher rawPositioningResultPub;
ros::Publisher deadReckoingResultPub;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "dead_reckoning");

  ros::NodeHandle n;

  PoseEstimation poseEstimation;

  poseEstimation.init();

  ros::Subscriber GPSNavSatFix_sub = n.subscribe("/kitti/oxts/gps/fix", 1000, &PoseEstimation::onGPSFixReceived, &poseEstimation);

  rawPositioningResultPub = n.advertise<geometry_msgs::PoseStamped>("rawPositioningResult", 1000);
  deadReckoingResultPub = n.advertise<geometry_msgs::PoseStamped>("deadReckoingResult", 1000);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate r(200); // 200 hz

  while(ros::ok())
  {
      ROS_INFO("Before get state..");
      Eigen::VectorXd state = poseEstimation.getState();
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.frame_id = "world";
      poseStamped.pose.position.x = state(0);
      poseStamped.pose.position.y = state(1);
      poseStamped.pose.position.z = state(2);
      ROS_INFO("DR Result x:%f, y:%f, z:%f", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);
      deadReckoingResultPub.publish(poseStamped);
      r.sleep();
  }

  return 0;
}
