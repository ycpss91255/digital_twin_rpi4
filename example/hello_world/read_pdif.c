#include <stdio.h>
#include <pigpiod_if2.h>

int main(int argc, char *argv[])
{
   int pi;
   int GPIO=4;
   int level;

   pi = pigpio_start(0, 0); /* Connect to local Pi. */

   if (pi < 0)
   {
      printf("Can't connect to pigpio daemon\n");
      return 1;
   }

   level = gpio_read(pi, GPIO);

   printf("GPIO %d is %d\n", GPIO, level);

   pigpio_stop(pi); /* Disconnect from local Pi. */
   
   return 0;
}
