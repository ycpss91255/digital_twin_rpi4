/*******************************
 ** Include system header files
 ******************************/
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <unistd.h>
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

MotorPin Pin;
MotorPin Cmd;
EncoderData *FBData = (EncoderData *)malloc(sizeof(*FBData));
int pi;
// enc callback functuin param
int SLP_Pin;
int glitch = 1000;
int CS;
int cb_id_a = -1;
int cb_id_b = -1;
bool SLP_sw;

void init();
void init(int);
void clear();
void clear(bool );

void setGlitch(int);
void setSpeed(float);
void readEnc();
void EncCallBack(int, unsigned, unsigned, uint32_t,void *);

//FIXME : pigpio Lib not found gpio read analog value
void getCSValue();

#ifdef DEBUG
void Pin_printf(const char* , int, int);
#endif

int main(int argc, char **argv) {
  // TODO : add rosparam get and cmd get function
  Pin.DIR = 14;
  Pin.PWM = 15;
  Pin.CS = 18;
  Pin.gpio_enA = 23;
  Pin.gpio_enB = 24;
  SLP_Pin = 12;
  SLP_sw ? init(SLP_Pin) : init();
  setSpeed(100);
  while(1);
  SLP_sw ? clear(SLP_Pin) : clear();
  return 0;
}

// Library
void init() {
  pi = pigpio_start(0, 0);

  if(pi < 0){
    printf("Can't connect to pigpio daemon\n");
    clear();
  }
  else {
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

    if((PinStatus.PWM || PinStatus.DIR || PinStatus.CS || PinStatus.gpio_enA || PinStatus.gpio_enB) != 0) {
      printf("Set up pin error\n");
      clear();
    }
  }
}

void init(int pin) {
  pi = pigpio_start(0, 0);

  if(pi < 0) {
    printf("Can't connect to pigpio daemon\n");
    clear();
  }
  else {
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

    if((PinStatus.PWM || PinStatus.DIR || PinStatus.CS || PinStatus.gpio_enA || PinStatus.gpio_enB) != 0) {
      printf("Set up pin error\n");
      clear();
    }
  }
}

void clear() {
  gpio_write(pi, Pin.DIR, 0);
  set_PWM_dutycycle(pi, Pin.PWM, 0);
  if(cb_id_a >= 0) {
    callback_cancel(cb_id_a);
    cb_id_a = -1;
  }
  if(cb_id_b) {
    callback_cancel(cb_id_b);
   cb_id_b = -1;
  }
  set_glitch_filter(pi, Pin.gpio_enB, 0);
  set_glitch_filter(pi, Pin.gpio_enB, 0);
  pigpio_stop(pi);

}

void clear(bool SLP_sw) {
  gpio_write(pi, Pin.DIR, 0);
  set_PWM_dutycycle(pi, Pin.PWM, 0);
  if(cb_id_a >= 0) {
    callback_cancel(cb_id_a);
    cb_id_a = -1;
  }
  if(cb_id_b) {
    callback_cancel(cb_id_b);
   cb_id_b = -1;
  }
  set_glitch_filter(pi, Pin.gpio_enB, 0);
  set_glitch_filter(pi, Pin.gpio_enB, 0);
  if (SLP_sw)
    gpio_write(pi, SLP_Pin, 0);
  pigpio_stop(pi);
}

void setGlitch(int glitch_v) {
  if(glitch_v >= 0 && (glitch_v != glitch)) {
    glitch = glitch_v;
    set_glitch_filter(pi, Pin.gpio_enA, glitch);
    set_glitch_filter(pi, Pin.gpio_enB, glitch);
  }
}

void setSpeed(float percent) {
  Cmd.DIR = (percent >= 0) ? 1 : 0;
  percent = (percent == 0) ? 0 : \
    (percent > 100 || percent < -100) ? 255 : round(abs(percent) * 2.55);
  Cmd.PWM = (int)percent;

  gpio_write(pi, Pin.DIR, Cmd.DIR);
  set_PWM_dutycycle(pi, Pin.PWM, Cmd.PWM);
}

void EncCallBack(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void * userdata) {
  printf("123\n");
}

// DEBUG pin function
#ifdef DEBUG
  void Pin_printf(const char*text, int pin, int status){
    printf("%s Pin %d Status = %s\n", text, pin, (status == 0) ? "OK" : "Error");
  }
#endif
