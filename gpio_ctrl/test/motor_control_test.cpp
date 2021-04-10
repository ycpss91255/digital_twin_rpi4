#include <iostream>
#include <unistd.h>
#include "gpio_ctrl/motor_control.h"

int main(int argc, char **argv) {

  MD02Data Pin1;

  Pin1.A.PWM = 14;
  Pin1.A.DIR = 15;
  Pin1.A.CS = 18;
  Pin1.B.PWM = 25;
  Pin1.B.DIR = 8;
  Pin1.B.CS = 7;
  Pin1.SLP = 12;
  // HACK : En_Y 23 En_G 24 11enc.

  MotorControl Dev1(Pin1);



  // for(int i = 0 ; i < 10 ; i++){
  //   Dev1.setSpeed(-100, 100);
  //   printf("100\n");
  //   sleep(1);
  //   Dev1.setSpeed(-50, 50);
  //   printf("50\n");
  //   sleep(1);
  //   Dev1.setSpeed(-25, 25);
  //   printf("25\n");
  //   sleep(1);
  //   Dev1.setSpeed(0, 0);
  //   printf("0\n");
  //   sleep(1);
  //   Dev1.setSpeed(25, -25);
  //   printf("25\n");
  //   sleep(1);
  //   Dev1.setSpeed(50, -50);
  //   printf("50\n");
  //   sleep(1);
  //   Dev1.setSpeed(100, -100);
  //   printf("100\n");
  //   sleep(1);
  // }


  Dev1.setSpeed(0, 0);

  return 0;
}
