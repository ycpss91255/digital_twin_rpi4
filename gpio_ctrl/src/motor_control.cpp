/*******************************
 ** Include system header files
 ******************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
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
#include <pigpiod_if2.h>

#include "gpio_ctrl/node_handle.h"
/*******************************
 * Define
 ******************************/
// #define ADJUST
// #define DEBUG

#ifdef ADJUST
#include <ctime>
#include <numeric>

#include "matplotlibcpp.h"
#define plt matplotlibcpp

std::vector<float> current, target;

void getLine(float, float);
void AdjustPIDParam();
#endif

typedef struct {
  int PWM;
  int DIR;
  int CS;
  int gpio_enA;
  int gpio_enB;
} MotorPin;
typedef struct {
  int LevA;
  int LevB;
  float Step;
  float Current;
} EncoderData;

int pi;
std::string robot_ns;
std::string node_name;
MotorPin Pin;
int SLP_Pin;
int glitch = 500;
MotorPin Cmd;
int cb_id_a = -1;
int cb_id_b = -1;
EncoderData *FBData = (EncoderData *)malloc(sizeof(*FBData));
float WheelPerimeter = 0.038 * M_PI;  // m
float Kp = 20;
float Kd = 0;
float EncoderSum = 102.08;
float previous_error = 0;

void init();
void clear();
uint64_t getNum(char *, int *);
void usage();
void ParamHandle(int, char **);
void setGlitch(int);
void EncCallBack(int, unsigned, unsigned, uint32_t, void *);
void setENS(bool, int);  // enc/s
void PosControl(float, float, float);
// FIXME : wirte a comment
// BUG : pigpio Lib not found gpio read analog value
// void getCSValue();

#ifdef DEBUG
void Pin_printf(const char *, int, int);
#endif

int main(int argc, char **argv) {
  ParamHandle(argc, argv);
  MotorNodeHandle Node(argc, argv, robot_ns, node_name);
  init();
  bool feedback = true;
#ifndef ADJUST
  while (ros::ok()) {
    // TODO : FB mode
    if (feedback != true) PosControl(Node.CmdPos, Kp, Kd);
    printf("int = %f\n", FBData->Step);
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
  // Node.~MotorNodeHandle();
  // exit(EXIT_SUCCESS);
  return 0;
}

// Library
void init() {
  pi = pigpio_start(0, 0);

  if (pi < 0) {
    printf("Can't connect to pigpio daemon\n");
    clear();
  } else {
    MotorPin PinStatus;

    PinStatus.PWM = set_mode(pi, Pin.PWM, PI_OUTPUT);
    PinStatus.DIR = set_mode(pi, Pin.DIR, PI_OUTPUT);
    PinStatus.CS = set_mode(pi, Pin.CS, PI_INPUT);

    PinStatus.gpio_enA = set_mode(pi, Pin.gpio_enA, PI_INPUT);
    PinStatus.gpio_enB = set_mode(pi, Pin.gpio_enB, PI_INPUT);
    PinStatus.gpio_enA |= set_pull_up_down(pi, Pin.gpio_enA, PI_PUD_UP);
    PinStatus.gpio_enB |= set_pull_up_down(pi, Pin.gpio_enB, PI_PUD_UP);
    PinStatus.gpio_enA |= set_glitch_filter(pi, Pin.gpio_enA, glitch);
    PinStatus.gpio_enB |= set_glitch_filter(pi, Pin.gpio_enB, glitch);
    int SLPStatus = -1;
    if (SLP_Pin > 0) {
      SLPStatus = set_mode(pi, SLP_Pin, PI_OUTPUT);
      SLPStatus |= gpio_write(pi, SLP_Pin, 1);
    }
    cb_id_a = callback_ex(pi, Pin.gpio_enA, EITHER_EDGE, EncCallBack, FBData);
    cb_id_b = callback_ex(pi, Pin.gpio_enB, EITHER_EDGE, EncCallBack, FBData);

#ifdef DEBUG
    Pin_printf("PWM", Pin.PWM, PinStatus.PWM);
    Pin_printf("DIR", Pin.DIR, PinStatus.DIR);
    Pin_printf("CS", Pin.CS, PinStatus.CS);
    Pin_printf("GPIO_ENA", Pin.gpio_enA, PinStatus.gpio_enA);
    Pin_printf("GPIO_ENB", Pin.gpio_enB, PinStatus.gpio_enB);
    if (SLP_Pin > 0) Pin_printf("SLP", SLP_Pin, SLPStatus);
#endif

    if ((PinStatus.PWM || PinStatus.DIR || PinStatus.CS || PinStatus.gpio_enA ||
         PinStatus.gpio_enB) != 0) {
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
  set_glitch_filter(pi, Pin.gpio_enB, 0);
  set_glitch_filter(pi, Pin.gpio_enB, 0);
  if (SLP_Pin > 0) gpio_write(pi, SLP_Pin, 0);
  pigpio_stop(pi);
}

uint64_t getNum(char *str, int *err) {
  uint64_t val;
  char *endptr;

  *err = 0;
  val = strtoll(str, &endptr, 0);
  if (*endptr) {
    *err = 1;
    val = -1;
  }
  return val;
}

void usage() {
  fprintf(stderr,
          "\n"
          "Usage : motor_control [options] ...\n"
          "   -N string, ROS node name\n"
          "   -d value, DIR Pin, 0-31\n"
          "   -p value, PWM Pin, 0-31\n"
          "   -c value, CS  Pin, 0-31\n"
          "   -a value, EnA Pin, 0-31\n"
          "   -b value, EnB Pin, 0-31\n"
          "   -s value, SLP Pin, 0-31\n"
          "ROSRUN INPUT STYLE\n"
          "rosrun this_pkg this_node -N node_name -d (int){gpio DIR} ... \n\n");
}

void ParamHandle(int argc, char **argv) {
  // node name, Pin , SLP Pin
  int opt, err, i;
  // rosrun and roslaunch
  if ((argc != 17) && (argc != 19))
    usage();
  else
    while ((opt = getopt(argc, argv, "N:n:d:p:c:a:b:s:")) != -1) switch (opt) {
        case 'N':
          robot_ns.assign(optarg);
          break;

        case 'n':
          node_name.assign(optarg);
          break;

        case 'd':
          i = getNum(optarg, &err);
          if ((i >= 0) && (i <= 31))
            Pin.DIR = i;
          else
            printf("invalid -d option %s", optarg);
          break;

        case 'p':
          i = getNum(optarg, &err);
          if ((i >= 0) && (i <= 31))
            Pin.PWM = i;
          else
            printf("invalid -p option %s", optarg);
          break;

        case 'c':
          i = getNum(optarg, &err);
          if ((i >= 0) && (i <= 31))
            Pin.CS = i;
          else
            printf("invalid -c option %s", optarg);
          break;

        case 'a':
          i = getNum(optarg, &err);
          if ((i >= 0) && (i <= 31))
            Pin.gpio_enA = i;
          else
            printf("invalid -a option %s", optarg);

          break;
        case 'b':
          i = getNum(optarg, &err);
          if ((i >= 0) && (i <= 31))
            Pin.gpio_enB = i;
          else
            printf("invalid -b option %s", optarg);
          break;

        case 's':
          i = getNum(optarg, &err);
          if ((i >= 0) && (i <= 31))
            SLP_Pin = i;
          else
            printf("invalid -s option %s", optarg);
          break;

        default:
          usage();
          clear();
          exit(EXIT_FAILURE);
      }
}

void setGlitch(int glitch_v) {
  if (glitch_v >= 0 && (glitch_v != glitch)) {
    glitch = glitch_v;
    set_glitch_filter(pi, Pin.gpio_enA, glitch);
    set_glitch_filter(pi, Pin.gpio_enB, glitch);
  }
}

void EncCallBack(int pi, unsigned user_gpio, unsigned level, uint32_t tick,
                 void *userdata) {
  EncoderData *FBData = (EncoderData *)userdata;
  float cw, ccw;
  bool Ap = false, An = false, Bp = false,
       Bn = false;  // n = negative p = positive
  if (level != PI_TIMEOUT) {
    if (user_gpio == Pin.gpio_enA) {
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
  // get up to 1260 pulses in 1 second
  // will not move if the pulse wave is lower than 125
  Cmd.DIR = dir;
  ens = abs(ens);
  Cmd.PWM = (ens < 125)     ? 0
            : (ens >= 1250) ? 255
                            : round(ens / 4.902);  // 1250(ens) to 255(gpio max)

  gpio_write(pi, Pin.DIR, Cmd.DIR);
  set_PWM_dutycycle(pi, Pin.PWM, Cmd.PWM);
}

void PosControl(float target, float Kp, float Kd) {
  float error = target - FBData->Step;
  float derivative = error - previous_error;
  float output = Kp * error + Kd * derivative;
  previous_error = error;
  bool dir = (output <= 0) ? 0 : 1;
  setENS(dir, output);
#ifdef DEBUG
  printf("node_name = %s, error = %f, derivative = %f, output = %f\n",
         node_name.c_str(), error, derivative, output);
#endif
}

#ifdef ADJUST
void getLine(float curr_pos, float target_pos) {
  current.push_back(curr_pos);
  target.push_back(target_pos);
}

void AdjustPIDParam() {
  int c_size = current.size();
  int t_size = target.size();
  if ((c_size != t_size) || c_size >= 0 || t_size >= 0) {
    // std::vector<float> c_len, t_len;
    // c_len.resize(c_size);
    // t_len.resize(t_size);
    // std::iota(c_len.begin(), c_len.end(), 1);
    // std::iota(t_len.begin(), t_len.end(), 1);
    // plt::clf();
    // plt::plot(current, c_len);
    // plt::plot(target, t_len);
    // plt::title("Sample figure");
    // plt::legend();

    plt::plot(current);
    plt::plot(target);
    plt::show();
  }

  // // Clear previous plot
  // plt::clf();
  // // Plot line from given x and y data. Color is selected automatically.
  // plt::plot(x, y);
  // // Plot a line whose name will show up as "log(x)" in the legend.
  // plt::named_plot("log(x)", x, z);

  // // Set x-axis to interval [0,1000000]
  // plt::xlim(0, n * n);

  // // Add graph title
  // plt::title("Sample figure");
  // // Enable legend.
  // plt::legend();
  // // Display plot continuously
  // plt::pause(0.01);
}
#endif

// DEBUG pin function
#ifdef DEBUG
void Pin_printf(const char *text, int pin, int status) {
  printf("%s Pin %d Status = %s\n", text, pin, (status == 0) ? "OK" : "Error");
}
#endif
