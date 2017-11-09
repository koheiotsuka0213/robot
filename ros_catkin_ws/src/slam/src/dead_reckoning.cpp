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

ros::Publisher deadReckoingResultPublisher;
bool isFirstPoint = true;
geodesy::UTMPoint utmFrameOffset;
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

  ROS_INFO("DR Result x:%f, y:%f, z:%f", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);

  deadReckoingResultPublisher.publish(poseStamped);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dead_reckoning");

  ros::NodeHandle n;

  ros::Subscriber GPSNavSatFix_sub = n.subscribe("/kitti/oxts/gps/fix", 1000, onGPSFixReceived);

  deadReckoingResultPublisher = n.advertise<geometry_msgs::PoseStamped>("deadReckoingResult", 1000);

  ros::spin();

  return 0;
}

