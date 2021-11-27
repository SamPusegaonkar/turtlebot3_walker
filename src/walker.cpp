/**
 * @file walker.cpp
 * @author Sameer Pusegaonkar (sameer@umd.edu)
 * @brief A walker cpp file which implmenets a walker algorithm
 * @version 0.1
 * @date 2021-11-25
 * @copyright Copyright (c) 2021
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "walker.h"

/**
 * @brief Construct a new Walker:: Walker object
 * Also initilizes the laser publisher and subscriber.
 */
Walker::Walker() {
  // Initilize the laser objects
  this->IsObstacleNearby = false;
  this->laser_subscriber = n.subscribe <sensor_msgs::LaserScan> ("/scan", 1000,
  &Walker::checkObstacle, this);

  // Initizlize the velocity objects
  this->velocity_publisher = n.advertise <geometry_msgs::Twist> ("/cmd_vel", 1000);
}

/**
 * @brief A laser callback method to see if a obstacle is nearby
 * @param msg 
 */
void Walker::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (auto range : msg->ranges) {
    if ( range < 1 ) {
      this->IsObstacleNearby = true;
      break;
    }
  }
}


