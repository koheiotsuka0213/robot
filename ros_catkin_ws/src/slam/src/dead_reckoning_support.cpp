/*
 * dead_reckoning_support.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: kohei
 */

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
#include <visualization_msgs/Marker.h>
#include <geodesy/utm.h>

ros::Publisher DRTrajRawPub;
ros::Publisher DRTrajPub;
geodesy::UTMPoint OffsetUTMPoint;
void ondeadReckoingResultReceived(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "world";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = msg->pose.position.x;
  p.y = msg->pose.position.y;
  p.z = msg->pose.position.z;

  points.points.push_back(p);
  line_strip.points.push_back(p);

  // The line list needs two points for each line
  line_list.points.push_back(p);
  p.z += 1.0;
  line_list.points.push_back(p);

  DRTrajPub.publish(points);
  // DRTrajPub.publish(line_strip);
  // DRTrajPub.publish(line_list);
}

void onrawPositioningResultReceived(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  geographic_msgs::GeoPoint geo_pt;
  geo_pt.latitude = msg->latitude;
  geo_pt.longitude = msg->longitude;
  geo_pt.altitude = msg->altitude;
  geodesy::UTMPoint UTMPt(geo_pt);

  if (OffsetUTMPoint.zone == 0)
  {
    OffsetUTMPoint = UTMPt;
  }

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = "world";
  poseStamped.pose.position.x = UTMPt.easting - OffsetUTMPoint.easting;
  poseStamped.pose.position.y = UTMPt.northing - OffsetUTMPoint.northing;
  poseStamped.pose.position.z = UTMPt.altitude - OffsetUTMPoint.altitude;



  static visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "world";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;

  // Points are green
  points.color.b = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = poseStamped.pose.position.x;
  p.y = poseStamped.pose.position.y;
  p.z = poseStamped.pose.position.z;

  points.points.push_back(p);
  line_strip.points.push_back(p);

  // The line list needs two points for each line
  line_list.points.push_back(p);
  p.z += 1.0;
  line_list.points.push_back(p);

  DRTrajRawPub.publish(points);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dead_reckoning_support");

  ros::NodeHandle n;

  ros::Subscriber deadReckoningResultSub = n.subscribe("deadReckoingResult", 1000, ondeadReckoingResultReceived);
  ros::Subscriber GPSNavSatFix_sub = n.subscribe("/kitti/oxts/gps/fix", 1000, onrawPositioningResultReceived);
  DRTrajRawPub = n.advertise<visualization_msgs::Marker>("DRTrajRaw", 1000);
  DRTrajPub = n.advertise<visualization_msgs::Marker>("DRTraj", 1000);

  ros::spin();

  return 0;
}

