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
#include "gpio_ctrl/MotorCmdFB.h"
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

  // function
  vector<int> getPin();
  void pubMotorFB(float);
  void pubMotorCmdFB(gpio_ctrl::MotorCmdFB);
  float getCmdPos();
  float getDigitalCmdPos();

 private:
  // variable
  ros::NodeHandle *n;
  string WheelNS;
  string RobotName;
  int WheelNum;

  ros::Publisher MotorFB_pub;
  ros::Publisher MotorCmdFB_pub;
  ros::Subscriber MotorPos_sub;
  ros::Subscriber DigitalMotorPos_sub;

  float CmdPos = 0;
  float DigitalCmdPos = 0;

  // function
  void init();
  void CmdPosBack(const std_msgs::Float64::ConstPtr &);
  void DigitalMotorPosBack(const std_msgs::Float64::ConstPtr &);
};

#endif  // NodeHandle_H
