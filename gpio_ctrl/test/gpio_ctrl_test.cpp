#include <stdio.h>
#include <stdint.h>
#include <cstdlib>
#include <unistd.h>
#include <pigpiod_if2.h>

void EncCallBack(int, unsigned, unsigned, uint32_t,void *);

// typedef void()

typedef struct{
  int x;
  int y;
} FU_T;


int main(int argc, char **argv){
  int pi = pigpio_start(0, 0); /* Connect to local Pi. */
  if (pi < 0){
      printf("Can't connect to pigpio daemon\n");
      return 1;
  }
  FU_T *rr;
  rr = (FU_T *)malloc(sizeof(*rr));
  rr->x =10;
  // rr = malloc(sizeof(FU_T));

  set_mode(pi, 23, PI_INPUT);
  set_glitch_filter(pi, 23, 1000);
  set_mode(pi, 24, PI_INPUT);
  set_glitch_filter(pi, 24, 1000);
  set_mode(pi, 10, PI_OUTPUT);
  set_pull_up_down(pi, 10, PI_PUD_UP);
  while(1){
    int cb = callback_ex(pi, 23, EITHER_EDGE, EncCallBack,rr);
  }

  pigpio_stop(pi); /* Disconnect from local Pi. */
  return 0;
}

void EncCallBack(int pi, unsigned user_gpio, unsigned level, uint32_t tick,void *user){
  FU_T *rr = (FU_T *)user;
  printf("rr.x =%d\n", rr->x);
  printf("pi = %d\n", pi);
  printf("user_gpio = %d\n", user_gpio);
  printf("level = %d\n", level);
}
