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
 * @file walker.h
 * @author Sameer Pusegaonkar (sameer@umd.edu)
 * @brief A walker header file for implementing the walker methods
 * @version 0.1
 * @date 2021-11-25
 * @copyright MIT License (c) 2021 Samer Pusegaonkar
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
