#ifndef MotorControl_H
#define MotorControl_H

/*******************************
 ** Include system header files
 ******************************/
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <string.h>
/*******************************
 * Include ROS header files
 ******************************/
#include "ros/ros.h"
/*******************************
 ** Include msg header files
 ******************************/
#include "gpio_ctrl/MotorFB.h"
/*******************************
 ** Include header files
 ******************************/
#include "gpio_ctrl/motor_typedef.h"
#include <pigpiod_if2.h>
/*******************************
 * Define
 ******************************/
// #define DEBUG

class MotorControl {
 public:
  MotorControl(MotorPin);
  MotorControl(MotorPin, int);
  ~MotorControl();

 public:
  // variable
  MotorPin Cmd;

  // function
  void setDevSLP(int);
  void setGlitch(int);

  //FIXME : pigpio Lib not found gpio read analog value
  void getCSValue();

  void setSpeed(float);
  // TODO : add get enc value function

  // TODO : add enc read function
  void readEnc();


 private:
  // variable
  int pi;
  int SLP_pin;
  MotorPin Pin;
  // EncoderData 
  int glitch = 1000;
  int CS;
  int cd_id_a;
  int cd_id_b;
  // function

  void init();
  void clear();

#ifdef DEBUG
  void Pin_printf(const char* , int, int);
#endif

};

#endif // MotorControl_H
