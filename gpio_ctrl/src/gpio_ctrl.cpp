#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pigpiod_if2.h>

int main(int argc, char *argv[]){
  int pi = pigpio_start(0, 0); /* Connect to local Pi. */
  // uint16_t = 4;

  if (pi < 0){
      printf("Can't connect to pigpio daemon\n");
      return 1;
  }

  int set_num = set_mode(pi, 4, PI_OUTPUT);
  printf("%d\n", set_num);
  for(int x = 0; x < 5; x++){
    gpio_write(pi, 4, 1);
    sleep(1);
    gpio_write(pi, 4, 0);
    sleep(1);
  }
  pigpio_stop(pi); /* Disconnect from local Pi. */
  return 0;
}
