#include <string>
#include <time.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <dirent.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

std_msgs::Header parseTime(std::string timestamp)
{

    std_msgs::Header header;

    // example: 2011-09-26 13:21:35.134391552
    //          01234567891111111111222222222
    //                    0123456789012345678
    struct tm t = {0};  // Initalize to all 0's
    t.tm_year = boost::lexical_cast<int>(timestamp.substr(0, 4)) - 1900;
    t.tm_mon  = boost::lexical_cast<int>(timestamp.substr(5, 2)) - 1;
    t.tm_mday = boost::lexical_cast<int>(timestamp.substr(8, 2));
    t.tm_hour = boost::lexical_cast<int>(timestamp.substr(11, 2));
    t.tm_min  = boost::lexical_cast<int>(timestamp.substr(14, 2));
    t.tm_sec  = boost::lexical_cast<int>(timestamp.substr(17, 2));
    t.tm_isdst = -1;
    time_t timeSinceEpoch = mktime(&t);

    header.stamp.sec  = timeSinceEpoch;
    header.stamp.nsec = boost::lexical_cast<int>(timestamp.substr(20, 8));

    return header;
}

int getIMU(std::string filename, sensor_msgs::Imu *ros_msgImu, std_msgs::Header *header)
{
    std::ifstream file_oxts(filename.c_str());
    if (!file_oxts.is_open())
    {
        ROS_ERROR_STREAM("Fail to open " << filename);
        return 0;
    }

    ROS_DEBUG_STREAM("Reading IMU data from oxts file: " << filename );

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    std::string line = "";

    getline(file_oxts, line);
    tokenizer tok(line, sep);
    std::vector<std::string> s(tok.begin(), tok.end());

    ros_msgImu->header.frame_id = ros::this_node::getName();
    ros_msgImu->header.stamp = header->stamp;

    //    - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
    //    - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
    //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
    ros_msgImu->linear_acceleration.x = boost::lexical_cast<double>(s[11]);
    ros_msgImu->linear_acceleration.y = boost::lexical_cast<double>(s[12]);
    ros_msgImu->linear_acceleration.z = boost::lexical_cast<double>(s[13]);

    //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
    //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
    //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
    ros_msgImu->angular_velocity.x = boost::lexical_cast<double>(s[8]);
    ros_msgImu->angular_velocity.y = boost::lexical_cast<double>(s[9]);
    ros_msgImu->angular_velocity.z = boost::lexical_cast<double>(s[10]);

    //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
    //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
    //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
    tf::Quaternion q = tf::createQuaternionFromRPY(   boost::lexical_cast<double>(s[3]),
                                                      boost::lexical_cast<double>(s[4]),
                                                      boost::lexical_cast<double>(s[5])
                                                  );
    ros_msgImu->orientation.x = q.getX();
    ros_msgImu->orientation.y = q.getY();
    ros_msgImu->orientation.z = q.getZ();
    ros_msgImu->orientation.w = q.getW();

    return 1;
}

int main(int argc, char **argv)
{
  static const int imuDataCharNumsPerLine = 30;

  ros::init(argc, argv, "imu_player");

  ros::NodeHandle n("imu_player");

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("sensor/imu", 1, true);

  std::string imuDataPath;
  std::string imuDataFile;
  std::string imuTimeStampFile;
  std::ifstream timeStamps(imuTimeStampFile.c_str());
  std_msgs::Header timeHeader;
  std::string readTimeStampLine;
  sensor_msgs::Imu imuRosMsg;
  
  unsigned int totalEntries = 0;
  unsigned int currentEntry = 0;

  if (!timeStamps.is_open())
  {
    ROS_ERROR_STREAM("Fail to open " << imuTimeStampFile);
    n.shutdown();
    return -1;
  }
  
  DIR * dir = opendir(imuDataPath.c_str());
  struct dirent *ent;
  unsigned int len = 0; 

  while ((ent = readdir(dir)))
  {
    len = strlen (ent->d_name);
    if (len > 2)
      totalEntries++;
  }
  closedir (dir);

  while(currentEntry < totalEntries)
  {
    timeStamps.seekg(imuDataCharNumsPerLine * currentEntry);
    getline(timeStamps, readTimeStampLine);
    timeHeader.stamp = parseTime(readTimeStampLine).stamp;
    std::string currentFile = imuDataPath + boost::str(boost::format("%010d") % currentEntry ) + ".txt";
    if (!getIMU(currentFile, &imuRosMsg, &timeHeader))
    {
      ROS_ERROR_STREAM("Fail to open " << currentFile);
      n.shutdown();
      return -1;
    }

    imu_pub.publish(imuRosMsg);
    currentEntry++;
  }  

  return 0;
}

