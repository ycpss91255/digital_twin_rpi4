#include <stdio.h>
#include <pigpio.h>

int main(int argc, char *argv[])
{
   int GPIO=4;
   int level;

   if (gpioInitialise() < 0) return 1;

   level = gpioRead(GPIO);

   printf("GPIO %d is %d\n", GPIO, level);

   gpioTerminate();
}
