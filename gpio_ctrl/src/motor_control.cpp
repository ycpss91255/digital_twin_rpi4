/*******************************
 ** Include system header files
 ******************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
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
/*******************************
 ** Include header files
 ******************************/
#include <pigpiod_if2.h>

#include "gpio_ctrl/node_handle.h"
/*******************************
 * Define
 ******************************/
// #define ADJUST
// #define DEBUG
/*******************************
 * Base param
 ******************************/
int pi;
std::string RobotName;
uint64_t WheelNum;
int glitch = 500;
typedef struct {
  int PWM;
  int DIR;
  int CS;
  int enA;
  int enB;
} MotorPin;
MotorPin Pin;
int SLP_Pin;
bool Mode;
/*******************************
 * Gpio feed back param
 ******************************/
typedef struct {
  int LevA;
  int LevB;
  float Step;
  float Current;
} EncoderData;
EncoderData *FBData = (EncoderData *)malloc(sizeof(*FBData));
int cb_id_a = -1;
int cb_id_b = -1;
/*******************************
 * Pos control param
 ******************************/
// TODO : ros topic function
gpio_ctrl::MotorCmdFB Cmd;
float WheelPerimeter = 0.038 * M_PI;  // m
float Kp = 20;
float Kd = 0;
float EncoderSum = 102.08;
float previous_error = 0;

/*******************************
 * Declare function
 ******************************/
void init(vector<int>);
void clear();
void usage();
void ParamHandle(int, char **);
void EncCallBack(int, unsigned, unsigned, uint32_t, void *);
void setENS(bool, int);  // enc/s
void PosControl(float, float, float);

// BUG : pigpio Lib not found gpio read analog value
// void getCSValue();

#ifdef ADJUST
#include <ctime>
#include <numeric>

#include "matplotlibcpp.h"
#define plt matplotlibcpp

std::vector<float> current, target;

void setGlitch(int);
void getLine(float, float);
void AdjustPIDParam();
#endif

#ifdef DEBUG
void Pin_printf(const char *, int, int);
#endif

int main(int argc, char **argv) {
  ParamHandle(argc, argv);
  MotorNodeHandle Node(argc, argv, RobotName, WheelNum);
  init(Node.getPin());
#ifndef ADJUST
  while (ros::ok()) {
    if (!Mode) {
      PosControl(Node.CmdPos, Kp, Kd);
    }
    Node.pubMotorFB(FBData->Step);
    ros::spinOnce();
  }
#else
  int i = 0;
  clock_t start = clock(), end;
  int loop_num = 600;
  float postarget = 102;
  while (ros::ok() && i <= loop_num) {
    PosControl(postarget, Kp, Kd);

    end = clock();  // 計算結束時間
    double time = ((double)(end - start)) / CLOCKS_PER_SEC;  // 計算實際花費時間
    if (time >= 0.001) {
      getLine(FBData->Step, postarget);
      i++;
      start = clock();
    }
  }
  AdjustPIDParam();
#endif
  clear();
  return 0;
}

// function
void init(vector<int> Pin_v) {
  pi = pigpio_start(0, 0);

  Pin.DIR = Pin_v.at(1);
  Pin.PWM = Pin_v.at(2);
  Pin.CS = Pin_v.at(3);
  Pin.enA = Pin_v.at(4);
  Pin.enB = Pin_v.at(5);
  SLP_Pin = Pin_v.at(6);

  if (pi < 0) {
    printf("Can't connect to pigpio daemon\n");
    clear();
  } else {
    MotorPin PinStatus;

    PinStatus.PWM = set_mode(pi, Pin.PWM, PI_OUTPUT);
    PinStatus.DIR = set_mode(pi, Pin.DIR, PI_OUTPUT);
    PinStatus.CS = set_mode(pi, Pin.CS, PI_INPUT);

    PinStatus.enA = set_mode(pi, Pin.enA, PI_INPUT);
    PinStatus.enB = set_mode(pi, Pin.enB, PI_INPUT);
    PinStatus.enA |= set_pull_up_down(pi, Pin.enA, PI_PUD_UP);
    PinStatus.enB |= set_pull_up_down(pi, Pin.enB, PI_PUD_UP);
    PinStatus.enA |= set_glitch_filter(pi, Pin.enA, glitch);
    PinStatus.enB |= set_glitch_filter(pi, Pin.enB, glitch);
    int SLPStatus;

    // slp_sw_value
    if (Pin_v.at(0)) {
      SLPStatus = set_mode(pi, SLP_Pin, PI_OUTPUT);
      SLPStatus |= gpio_write(pi, SLP_Pin, 1);
    }
    cb_id_a = callback_ex(pi, Pin.enA, EITHER_EDGE, EncCallBack, FBData);
    cb_id_b = callback_ex(pi, Pin.enB, EITHER_EDGE, EncCallBack, FBData);

#ifdef DEBUG
    Pin_printf("PWM", Pin.PWM, PinStatus.PWM);
    Pin_printf("DIR", Pin.DIR, PinStatus.DIR);
    Pin_printf("CS", Pin.CS, PinStatus.CS);
    Pin_printf("GPIO_ENA", Pin.enA, PinStatus.enA);
    Pin_printf("GPIO_ENB", Pin.enB, PinStatus.enB);
    if (Pin_v.at(0)) Pin_printf("SLP", SLP_Pin, SLPStatus);
#endif

    if ((PinStatus.PWM || PinStatus.DIR || PinStatus.CS || PinStatus.enA ||
         PinStatus.enB) != 0) {
      printf("Set up pin error\n");
      clear();
    }
  }
}

void clear() {
  gpio_write(pi, Pin.DIR, 0);
  set_PWM_dutycycle(pi, Pin.PWM, 0);
  if (cb_id_a >= 0) {
    callback_cancel(cb_id_a);
    cb_id_a = -1;
  }
  if (cb_id_b) {
    callback_cancel(cb_id_b);
    cb_id_b = -1;
  }
  set_glitch_filter(pi, Pin.enB, 0);
  set_glitch_filter(pi, Pin.enB, 0);
  if (SLP_Pin > 0) gpio_write(pi, SLP_Pin, 0);
  pigpio_stop(pi);
}

void usage() {
  fprintf(stderr,
          "\n"
          "Usage : motor_control [options] ...\n"
          "   -r string, robot name\n"
          "   -n value, wheel num\n"
          "   -m boolean, 0 : PosControl Mode, 1 : FeedBack Mode\n");
}

void ParamHandle(int argc, char **argv) {
  if (argc != 9)
    usage();
  else {
    int opt;
    while ((opt = getopt(argc, argv, "r:n:m:")) != -1) switch (opt) {
        case 'r':
          RobotName.assign(optarg);
          break;

        case 'n':
          WheelNum = strtol(optarg, NULL, 0);
          break;

        case 'm':
          Mode = strtol(optarg, NULL, 0);
          break;

        default:
          usage();
          clear();
          exit(EXIT_FAILURE);
      }
  }
}

void EncCallBack(int pi, unsigned user_gpio, unsigned level, uint32_t tick,
                 void *userdata) {
  EncoderData *FBData = (EncoderData *)userdata;
  float cw, ccw;
  bool Ap = false, An = false, Bp = false,
       Bn = false;  // n = negative p = positive
  if (level != PI_TIMEOUT) {
    if (user_gpio == Pin.enA) {
      FBData->LevA = level;
      if (level == 1)
        Ap = true;
      else
        An = true;
    } else {
      FBData->LevB = level;
      if (level == 1)
        Bp = true;
      else
        Bn = true;
    }
    cw = ((Ap & !FBData->LevB) | (An & FBData->LevB) | (Bp & FBData->LevA) |
          (Bn & !FBData->LevA));

    ccw = ((Ap & FBData->LevB) | (An & !FBData->LevB) | (Bp & !FBData->LevA) |
           (Bn & FBData->LevA));
    float pos = (cw - ccw) / 4;
    // float pos = (cw - ccw);
    FBData->Step += pos;

    FBData->Current = FBData->Step / EncoderSum * WheelPerimeter;
  }
}

void setENS(bool dir, int ens) {
  // get up to 1250 pulses in 1 second
  // will not move if the pulse wave is lower than 125
  Cmd.dir = dir;
  ens = abs(ens);
  Cmd.pwm = (ens < 125)     ? 0
            : (ens >= 1250) ? 255
                            : round(ens / 4.902);  // 1250(ens) to 255(gpio max)

  gpio_write(pi, Pin.DIR, Cmd.dir);
  set_PWM_dutycycle(pi, Pin.PWM, Cmd.pwm);
}

void PosControl(float target, float Kp, float Kd) {
  float error = target - FBData->Step;
  float derivative = error - previous_error;
  float output = Kp * error + Kd * derivative;
  previous_error = error;
  bool dir = (output <= 0) ? 0 : 1;
  setENS(dir, output);

#ifdef DEBUG
#ifdef ADJUST
  printf("node_name = wheel%ld, error = %f, derivative = %f, output = %f\n",
         WheelNum, error, derivative, output);
#endif
#endif
}

#ifdef ADJUST
void setGlitch(int glitch_v) {
  if (glitch_v >= 0 && (glitch_v != glitch)) {
    glitch = glitch_v;
    set_glitch_filter(pi, Pin.gpio_enA, glitch);
    set_glitch_filter(pi, Pin.gpio_enB, glitch);
  }
}

void getLine(float curr_pos, float target_pos) {
  current.push_back(curr_pos);
  target.push_back(target_pos);
}

void AdjustPIDParam() {
  int c_size = current.size();
  int t_size = target.size();
  if ((c_size != t_size) || c_size >= 0 || t_size >= 0) {
    plt::plot(current);
    plt::plot(target);
    plt::show();
  }
}
#endif

// DEBUG pin function
#ifdef DEBUG
void Pin_printf(const char *text, int pin, int status) {
  printf("%s Pin %d Status = %s\n", text, pin, (status == 0) ? "OK" : "Error");
}
#endif
