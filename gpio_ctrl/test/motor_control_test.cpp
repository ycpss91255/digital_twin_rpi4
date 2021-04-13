#include <iostream>
#include <unistd.h>
#include "gpio_ctrl/motor_typedef.h"
#include "gpio_ctrl/motor_control.h"

int main(int argc, char **argv) {
  MotorPin Pin1;

  Pin1.DIR = 14;
  Pin1.PWM = 15;
  Pin1.CS = 18;
  Pin1.gpio_enA = 23;
  Pin1.gpio_enB = 24;
  int SLP_pin = 12;
  MotorControl Dev1(Pin1, SLP_pin);
  if(argc != 2)
    printf("please input param!\n");
  else {
    float speed = (float)strtod(argv[1],NULL);
    printf("%f\n", speed);
    // min 7
    Dev1.setSpeed(speed);
    while(1);
}

  return 0;
}
