#ifndef NodeHandle_H
#define NodeHandle_H
/*******************************
 * Include system header files
 ******************************/
#include <string.h>

#include <cstdio>
#include <iostream>
/*******************************
 * Include ROS header files
 ******************************/
#include "ros/ros.h"
/*******************************
 ** Include msg header files
 ******************************/
#include "std_msgs/Float32.h"
/*******************************
 * Define
 ******************************/
// #define DEBUG

class MotorNodeHandle {
 public:
  MotorNodeHandle(int, char **, std::string);
  ~MotorNodeHandle();

 public:
  // variable
  float CmdPos = 0;
  // function
  void pubMotorFB(float);


 private:
  // variable
  ros::NodeHandle *n;
  ros::Publisher MotorFB_pub;
  ros::Subscriber MotorPos_sub;
  std::string node_name;
  // function
  void init(std::string);
  void CmdPosBack(const std_msgs::Float32::ConstPtr &);
};

#endif  // NodeHandle_H
