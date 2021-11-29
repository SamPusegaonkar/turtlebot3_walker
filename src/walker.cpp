/**
 * MIT License
 *
 * Copyright (c) 2021 Sameer Pusegaonkar
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file walker.cpp
 * @author Sameer Pusegaonkar (sameer@umd.edu)
 * @brief A walker cpp file which implmenets a walker algorithm
 * @version 0.1
 * @date 2021-11-25
 * @copyright MIT License (c) 2021 Samer Pusegaonkar
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
  &Walker::LaserCallback, this);

  // Initizlize the velocity objects
  this->velocity_publisher = n.advertise <geometry_msgs::Twist> ("/cmd_vel", 1000);
}


/**
 * @brief A laser callback method to see if a obstacle is nearby
 * @param msg 
 */
void Walker::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  this->IsObstacleNearby = false;

  // Get the first 50 values
  for (auto i = 0; i < 50; i++) {
    auto depth = msg->ranges[i];
    if ( depth < 0.5 ) {
      this->IsObstacleNearby = true;
      break;
    }
  }

  // Get the las 50 values
  for (auto i = 290; i < 359; i++) {
    auto depth = msg->ranges[i];
    if ( depth < 0.5 ) {
      this->IsObstacleNearby = true;
      break;
    }
  }
}


/**
 * @brief A method to move the turtlebot
 * This bot will rotate when it sees a obstacle
 */
void Walker::Travel() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (this->IsObstacleNearby) {
      velocity.linear.x = 0.0;
      velocity.angular.z = 0.1;
    } else {
      velocity.linear.x = 0.1;
      velocity.angular.z = 0.0;
    }
    this->velocity_publisher.publish(velocity);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
