#ifndef MotorControl_H
#define MotorControl_H

/*******************************
 ** Include system header files
 ******************************/
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
/*******************************
 * Include ROS header files
 ******************************/
#include "ros/ros.h"
/*******************************
 ** Include header files
 ******************************/
#include <pigpiod_if2.h>
/*******************************
 * Define
 ******************************/
#define DEBUG

typedef struct {
  int PWM;
  int DIR;
  int CS;
}MotorData;

typedef struct {
  MotorData A;
  MotorData B;
  int SLP;
} MD02Data;

class MotorControl {
 public:
  MotorControl(MD02Data);
  ~MotorControl();

 public:
  // variable
  // gpio_ctrl::MotorData MotorA;
  // gpio_ctrl::MotorData MotorB;

  // function
  void setSpeed(float, float);
  MD02Data DevCmd;

 private:
  // variable
  int PI;
  MD02Data DevPin;

  // function
  void init();
  void setPWM(MotorData, MotorData, float);

};

#endif // MotorControl_H
