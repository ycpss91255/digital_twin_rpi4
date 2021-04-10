#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include <pigpiod_if2.h>

/*
# servo_demo_D.c
# 2016-10-08
# Public Domain

gcc -Wall -pthread -o servo_demo_D servo_demo_D.c -lpigpiod_if2

This software requires the pigpio daemon to be running.

sudo pigpiod

./servo_demo_D          # Send servo pulses to GPIO 4.
./servo_demo_D 23 24 25 # Send servo pulses to GPIO 23, 24, 25.
*/

#define NUM_GPIO 32

#define MIN_WIDTH 1000
#define MAX_WIDTH 2000

int run=1;

int step[NUM_GPIO];
int width[NUM_GPIO];
int used[NUM_GPIO];

int randint(int from, int to)
{
   return (random() % (to - from + 1)) + from;
}

void stop(int signum)
{
   run = 0;
}

typedef void (*signalFunc_t) (int signum);

static void setSignalHandler(int signum, signalFunc_t sigHandler)
{
   struct sigaction new;

   memset(&new, 0, sizeof(new));
   new.sa_handler = sigHandler;

   sigaction(signum, &new, NULL);
}

int main(int argc, char *argv[])
{
   int i, g, pi;

   pi = pigpio_start(NULL, NULL);

   if (pi < 0) return -1;

   setSignalHandler(SIGINT, stop);

   if (argc == 1) used[4] = 1;
   else
   {
      for (i=1; i<argc; i++)
      {
         g = atoi(argv[i]);
         if ((g>=0) && (g<NUM_GPIO)) used[g] = 1;
      }
   }

   printf("Sending servos pulses to GPIO");

   for (g=0; g<NUM_GPIO; g++)
   {
      if (used[g])
      {
         printf(" %d", g);
         step[g] = randint(5, 25);
         if ((step[g] % 2) == 0) step[g] = -step[g];
         width[g] = randint(MIN_WIDTH, MAX_WIDTH);
      }
   }

   printf(", control C to stop.\n");

   while(run)
   {
      for (g=0; g<NUM_GPIO; g++)
      {
         if (used[g])
         {
            set_servo_pulsewidth(pi, g, width[g]);

            // printf("%d %d\n", g, width[g]);

            width[g] += step[g];

            if ((width[g]<MIN_WIDTH) || (width[g]>MAX_WIDTH))
            {
               step[g] = -step[g];
               width[g] += step[g];
            }
         }
      }

      time_sleep(0.1);
   }

   printf("\ntidying up\n");

   for (g=0; g<NUM_GPIO; g++)
   {
      if (used[g]) set_servo_pulsewidth(pi, g, 0);
   }

   pigpio_stop(pi);

   return 0;
}

