
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <walker.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "walker");
  Walker turtlebot_walker;
  turtlebot_walker.Travel();
  return 0;
}
