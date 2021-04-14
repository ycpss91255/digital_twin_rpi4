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

#include "gpio_ctrl/motor_typedef.h"
#include "gpio_ctrl/node_handle.h"
/*******************************
 * Define
 ******************************/
// #define DEBUG
int pi;
std::string node_name;
MotorPin Pin;
int SLP_Pin;
int glitch = 500;
MotorPin Cmd;
int cb_id_a = -1;
int cb_id_b = -1;
EncoderData *FBData = (EncoderData *)malloc(sizeof(*FBData));

float WheelDiameter = 0.038 * M_PI;  // m
float EncoderSum = 102.08;
// float target = 0;

int previous_error = 0;
// float integral = 0;

void init();
void clear();
uint64_t getNum(char *str, int *err);
void usage();
void ParamHandle(int, char **);
void setGlitch(int);
void setSpeed(float);
void EncCallBack(int, unsigned, unsigned, uint32_t, void *);
void setPos(float, float, float);

// FIXME : wirte a comment
// BUG : pigpio Lib not found gpio read analog value
// void getCSValue();

float value_map(float);
#ifdef DEBUG
void Pin_printf(const char *, int, int);
#endif

int main(int argc, char **argv) {
  // TODO : add rosparam get and cmd get function
  // Pin.DIR = 14;
  // Pin.PWM = 15;
  // Pin.CS = 18;
  // Pin.gpio_enA = 23;
  // Pin.gpio_enB = 24;
  // SLP_Pin = 12;
  ParamHandle(argc, argv);

  init();
  MotorNodeHandle Node(argc, argv, node_name);

  // printf()

  printf("init\n");

  // // TODO : target -> ros topic
  // float target = (float)strtod(argv[1], NULL);
  // // TODO : test Kp,Kd param value
  // float Kp = (float)strtod(argv[2], NULL);
  // float Kd = (float)strtod(argv[3], NULL);
  //   while (1) {
  //     // setPos(target, Kp, Kd);
  //     usleep(1000);

  // }

  clear();
  Node.~MotorNodeHandle();
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

    cb_id_a = callback_ex(pi, Pin.gpio_enA, EITHER_EDGE, EncCallBack, FBData);
    cb_id_b = callback_ex(pi, Pin.gpio_enB, EITHER_EDGE, EncCallBack, FBData);

#ifdef DEBUG
    Pin_printf("PWM", Pin.PWM, PinStatus.PWM);
    Pin_printf("DIR", Pin.DIR, PinStatus.DIR);
    Pin_printf("CS", Pin.CS, PinStatus.CS);
    Pin_printf("GPIO_ENA", Pin.gpio_enA, PinStatus.gpio_enA);
    Pin_printf("GPIO_ENB", Pin.gpio_enB, PinStatus.gpio_enB);
#endif

    if ((PinStatus.PWM || PinStatus.DIR || PinStatus.CS || PinStatus.gpio_enA ||
         PinStatus.gpio_enB) != 0) {
      printf("Set up pin error\n");
      clear();
    }
  }
}

void init(int pin) {
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
    SLP_Pin = pin;
    int SLPStatus = set_mode(pi, SLP_Pin, PI_OUTPUT);
    SLPStatus |= gpio_write(pi, SLP_Pin, 1);

    cb_id_a = callback_ex(pi, Pin.gpio_enA, EITHER_EDGE, EncCallBack, FBData);
    cb_id_b = callback_ex(pi, Pin.gpio_enB, EITHER_EDGE, EncCallBack, FBData);

#ifdef DEBUG
    Pin_printf("PWM", Pin.PWM, PinStatus.PWM);
    Pin_printf("DIR", Pin.DIR, PinStatus.DIR);
    Pin_printf("CS", Pin.CS, PinStatus.CS);
    Pin_printf("GPIO_ENA", Pin.gpio_enA, PinStatus.gpio_enA);
    Pin_printf("GPIO_ENB", Pin.gpio_enB, PinStatus.gpio_enB);
    Pin_printf("SLP", SLP_Pin, SLPStatus);
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

void setGlitch(int glitch_v) {
  if (glitch_v >= 0 && (glitch_v != glitch)) {
    glitch = glitch_v;
    set_glitch_filter(pi, Pin.gpio_enA, glitch);
    set_glitch_filter(pi, Pin.gpio_enB, glitch);
  }
}

void setSpeed(float percent) {
  Cmd.DIR = (percent >= 0) ? 1 : 0;
  percent = (percent == 0)                      ? 0
            : (percent > 100 || percent < -100) ? 255
                                                : round(abs(percent) * 2.55);
  Cmd.PWM = (int)percent;

  gpio_write(pi, Pin.DIR, Cmd.DIR);
  set_PWM_dutycycle(pi, Pin.PWM, Cmd.PWM);
}

void EncCallBack(int pi, unsigned user_gpio, unsigned level, uint32_t tick,
                 void *userdata) {
  EncoderData *FBData = (EncoderData *)userdata;
  int cw, ccw;
  bool Ap = false, An = false, Bp = false, Bn = false;
  // n = negative
  // p = positive

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
    FBData->Step += (cw - ccw);
  }
}

void setPos(float target, float Kp, float Kd) {
  // target is m
  // TODO : step Conversion to m
  int EncTarget = (int)round(target / WheelDiameter * EncoderSum);
  int error = EncTarget - FBData->Step;
  int derivative = error - previous_error;
  float output = Kp * (float)error + Kd * (float)derivative;
  previous_error = error;

  setSpeed(output);
}

// DEBUG pin function
#ifdef DEBUG
void Pin_printf(const char *text, int pin, int status) {
  printf("%s Pin %d Status = %s\n", text, pin, (status == 0) ? "OK" : "Error");
}
#endif

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
  if ((argc != 15) && (argc != 17))
    usage();
  else
    while ((opt = getopt(argc, argv, "N:d:p:c:a:b:s:")) != -1) switch (opt) {
        case 'N':
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
