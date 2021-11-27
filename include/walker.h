/**
 * @file walker.h
 * @author Sameer Pusegaonkar (sameer@umd.edu)
 * @brief A walker header file for implementing the walker methods
 * @version 0.1
 * @date 2021-11-25
 * @copyright Copyright (c) 2021
 */

#ifndef INCLUDE_WALKER_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Walker {
 public:
  /**
    * @brief Constructor for the Walker class
  */
  Walker();

  /**
   * @brief Moves the robot
   * @return void
   */
  void Travel();

  /**
   * @brief A callback method for the laser scanner. This method will set the IsObstacleNearby flag.
   * @param msg 
   */
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

 private:
  ros::Subscriber laser_subscriber;  // Subscribes to get laser data
  ros::Publisher velocity_publisher;  // Publishes velocity
  geometry_msgs::Twist velocity;
  ros::NodeHandle n;
  bool IsObstacleNearby;  // flag to see if obstacle is nearby
};

#endif  // INCLUDE_WALKER_H_
