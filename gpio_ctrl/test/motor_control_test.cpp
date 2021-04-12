#include <iostream>
#include <unistd.h>
#include "gpio_ctrl/motor_control.h"

int main(int argc, char **argv) {

  MD02Data Pin1;

  Pin1.A.PWM = 14;
  Pin1.A.DIR = 15;
  Pin1.A.CS = 18;
  Pin1.A.gpio_enA = 23;
  Pin1.A.gpio_enB = 24;
  Pin1.B.PWM = 25;
  Pin1.B.DIR = 8;
  Pin1.B.CS = 7;
  Pin1.B.gpio_enA = 5;
  Pin1.B.gpio_enB = 6;
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
  if(argc != 3){
    printf("please input param!\n");
    Dev1.setSpeed(0, 0);
  }
  else{
    printf("%f\n", (float)strtod(argv[1],NULL));
    printf("%f\n", (float)strtod(argv[2],NULL));
    Dev1.setSpeed((float)strtod(argv[1],NULL), (float)strtod(argv[2],NULL));
  }
  return 0;
}
