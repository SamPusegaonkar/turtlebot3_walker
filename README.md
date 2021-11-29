# turtlebot3_walker

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

A walker algorithm for the turtlebot3 in a ROS package.

# Dependencies

  * I assume that you are using Ubuntu 18.04 with ROS Melodic installed. To install ROS Melodic, please visit this [link](
  http://wiki.ros.org/melodic/Installation/Ubuntu). Do the full-desktop-installation.

  * If you have ROS, cmake (catkin) will already be installed. If you dont, install cmake.

  * Make sure you have a caktin workspace setup. Follow [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for reference.


# How to build this repo
```
  * cd ~/catkin_ws/src
  * git clone https://github.com/SamPusegaonkar/turtlebot3_walker
  * cd ~/catkin_ws/
  * catkin_make
```


# How to run CppLint, CppCheck
```
  * cd ~/catkin_ws/src/turtlebot3_walker

  * cppcheck --enable=all --std=c++11 --language=c++ -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/")  $( find . -name *.h | grep -vE -e "^./build/" -e "^./vendor/")  --output-file=results/cppcheck_process.txt > results/cppcheck_result.txt
  
  * cpplint --verbose 5 $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/") $( find . -name *.h | grep -vE -e "^./build/" -e "^./vendor/") > results/cpplint_result.txt

```

# How to run doxygen
```
  * cd ~/catkin_ws/src/turtlebot3_walker

  * doxygen Doxyfile #Open the index file to read the documentation!

```

# How to run the repo and play the rosbag file
```
  * cd ~/catkin_ws/
  * source ./devel/setup.bash
  * roslaunch turtlebot3_walker main.launch record:=true
  * rosbag play ~/catkin_ws/src/turtlebot3_walker/results/recording.bag # To play the bag file
  * rosbag info ~/catkin_ws/src/turtlebot3_walker/results/recording.bag # To play the bag file

```