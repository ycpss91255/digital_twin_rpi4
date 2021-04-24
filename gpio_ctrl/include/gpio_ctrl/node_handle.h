#ifndef NodeHandle_H
#define NodeHandle_H
/*******************************
 * Include system header files
 ******************************/
#include <stdint.h>
#include <string.h>

#include <cstdio>
#include <iostream>
#include <vector>
/*******************************
 * Include ROS header files
 ******************************/
#include "ros/ros.h"
/*******************************
 ** Include msg header files
 ******************************/
#include "std_msgs/Float64.h"
/*******************************
 * Define
 ******************************/
// #define DEBUG
using namespace std;
class MotorNodeHandle {
 public:
  MotorNodeHandle(int, char **, string, uint64_t);
  ~MotorNodeHandle();

 public:
  // variable
  float CmdPos = 0;
  // function
  vector<int> getPin();
  void pubMotorFB(float);

 private:
  // variable
  ros::NodeHandle *n;
  string WheelNS;

  ros::Publisher MotorFB_pub;
  ros::Subscriber MotorPos_sub;
  // function
  void init();
  void CmdPosBack(const std_msgs::Float64::ConstPtr &);
};

#endif  // NodeHandle_H
