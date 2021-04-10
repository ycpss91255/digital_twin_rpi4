#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pigpiod_if2.h>

void EncCallBack(int, unsigned, unsigned, uint32_t);

int main(int argc, char **argv){
  int pi = pigpio_start(0, 0); /* Connect to local Pi. */
  // uint16_t = 4;

  if (pi < 0){
      printf("Can't connect to pigpio daemon\n");
      return 1;
  }
  set_mode(pi, 23, PI_INPUT);
  while(1){
    callback(pi, 23, RISING_EDGE, EncCallBack);
  }

  pigpio_stop(pi); /* Disconnect from local Pi. */
  return 0;
}

void EncCallBack(int pi, unsigned user_gpio, unsigned level, uint32_t tick){
  printf("123\n");
}
