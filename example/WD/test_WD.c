/*
test_WD.c
2015-11-25
Public Domain
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include <pigpiod_if2.h>

#include "WD.h"

/*

REQUIRES

Wiegand contacts 0 and 1 connected to separate GPIO.

TO BUILD

gcc -Wall -pthread -o WD test_WD.c WD.c -lpigpiod_if2

TO RUN

sudo pigpiod # If the daemon is not already running

then

./WD -gGPIO -wGPIO

*/

void fatal(char *fmt, ...)
{
   char buf[128];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);

   fprintf(stderr, "%s\n", buf);

   fflush(stderr);

   exit(EXIT_FAILURE);
}

void usage()
{
   fprintf(stderr, "\n" \
      "Usage: WD [OPTION] ...\n" \
      "   -g value, gpio green (0), 0-31,           default None\n" \
      "   -w value, gpio white (1), 0-31,           default None\n" \
      "   -m value, message gap, 1000-65000 micros, default 2000\n" \
      "   -s value, run seconds, >=0 (0=forever),   default 0\n" \
      "   -h string, host name,                     default NULL\n" \
      "   -p value, socket port, 1024-32000,        default 8888\n" \
      "EXAMPLE\n" \
      "WD -g10 -w12\n" \
      "   Read a Wiegand device connected to GPIO 10/12.\n\n");
}

int optGpioGreen = -1;
int optGpioWhite = -1;
int optSeconds = 0;
int optGap = 2000;
char *optHost   = NULL;
char *optPort   = NULL;

static uint64_t getNum(char *str, int *err)
{
   uint64_t val;
   char *endptr;

   *err = 0;
   val = strtoll(str, &endptr, 0);
   if (*endptr) {*err = 1; val = -1;}
   return val;
}

static void initOpts(int argc, char *argv[])
{
   int opt, err, i;

   while ((opt = getopt(argc, argv, "g:w:m:s:h:p:")) != -1)
   {
      switch (opt)
      {
         case 'g':
            i = getNum(optarg, &err);
            if ((i >= 0) && (i <= 31)) optGpioGreen = i;
            else fatal("invalid -g option (%s)", optarg);
            break;

         case 'w':
            i = getNum(optarg, &err);
            if ((i >= 0) && (i <= 31)) optGpioWhite = i;
            else fatal("invalid -w option (%s)", optarg);
            break;

         case 'm':
            i = getNum(optarg, &err);
            if ((i >= 1000) && (i <= 65000)) optGap = i;
            else fatal("invalid -m option (%s)", optarg);
            break;

         case 's':
            i = getNum(optarg, &err);
            if (i >= 0) optSeconds = i;
            else fatal("invalid -s option (%s)", optarg);
            break;

         case 'h':
            optHost = malloc(sizeof(optarg)+1);
            if (optHost) strcpy(optHost, optarg);
            break;

         case 'p':
            optPort = malloc(sizeof(optarg)+1);
            if (optPort) strcpy(optPort, optarg);
            break;

        default: /* '?' */
           usage();
           exit(-1);
        }
    }
}

void cbf(WD_data_t w)
{
   printf("%llu %d\n", (long long)w.code, w.bits);
}

int main(int argc, char *argv[])
{
   int pi;
   WD_t * w;

   initOpts(argc, argv);

   if ((optGpioGreen < 0) || (optGpioWhite < 0) || (optGpioGreen == optGpioWhite))
   {
      fprintf(stderr, "You must specify the green and white GPIO.\n");
      exit(0);
   }

   pi = pigpio_start(optHost, optPort); /* Connect to Pi. */

   if (pi >= 0)
   {
      w = WD(pi, optGpioGreen, optGpioWhite, optGap, cbf);

      if (optSeconds) sleep(optSeconds);
      else while(1) sleep(60);

      WD_cancel(w);

      pigpio_stop(pi);
   }
   return 0;
}

