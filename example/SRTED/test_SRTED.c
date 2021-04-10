/*
test_SRTED.c
2015-11-16
Public Domain
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include <pigpiod_if2.h>

#include "SRTED.h"

/*

REQUIRES

One of more sonar rangers which use the trigger/echo method
of operation.

TO BUILD

gcc -Wall -pthread -o SRTED test_SRTED.c SRTED.c -lpigpiod_if2

TO RUN

./SRTED

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
      "Usage: DHTXXD [OPTION] ...\n" \
      "   -t value, trigger gpio, 0-31,               default 4\n" \
      "   -e value, echo gpio, 0-31,                  default 5\n" \
      "   -i value, reading interval in seconds\n" \
      "             0=single reading,                 default 0\n" \
      "   -h string, host name,                       default NULL\n" \
      "   -p value, socket port, 1024-32000,          default 8888\n" \
      "EXAMPLE\n" \
      "SRTED -t11 -e15 -i5\n" \
      "   Read a sonar ranger connected to GPIO 11/15 every 5 seconds.\n\n");
}

int optTrig     = 4;
int optEcho     = 5;
char *optHost   = NULL;
char *optPort   = NULL;
float optInterval = 0.0;

static uint64_t getNum(char *str, int *err)
{
   uint64_t val;
   char *endptr;

   *err = 0;
   val = strtoll(str, &endptr, 0);
   if (*endptr) {*err = 1; val = -1;}
   return val;
}

static float getFloat(char *str, int *err)
{
   float val;
   char *endptr;

   *err = 0;
   val = strtof(str, &endptr);
   if (*endptr) {*err = 1; val = 0.0;}
   return val;
}

static void initOpts(int argc, char *argv[])
{
   int opt, err, i;
   float f;

   while ((opt = getopt(argc, argv, "t:e:h:i:p:")) != -1)
   {
      switch (opt)
      {
         case 't':
            i = getNum(optarg, &err);
            if ((i >= 0) && (i <= 31)) optTrig = i;
            else fatal("invalid -t option (%d)", i);
            break;

         case 'e':
            i = getNum(optarg, &err);
            if ((i >= 0) && (i <= 31)) optEcho = i;
            else fatal("invalid -e option (%d)", i);
            break;

         case 'h':
            optHost = malloc(sizeof(optarg)+1);
            if (optHost) strcpy(optHost, optarg);
            break;

         case 'i':
            f = getFloat(optarg, &err);
            if ((f>=0.0) && (f<=86400.0) && (!err)) optInterval = f;
            else fatal("invalid -i option (%s)", optarg);
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

void cbf(SRTED_data_t r)
{
   printf("%d %6.1f %6d\n",
      r.status, r.range_cms, r.round_trip_micros);
}

int main(int argc, char *argv[])
{
   int pi;
   SRTED_t *sr;

   initOpts(argc, argv);

   pi = pigpio_start(optHost, optPort); /* Connect to Pi. */

   if (pi >= 0)
   {
      sr = SRTED(pi, optTrig, optEcho, cbf); /* Create sonar ranger. */

      if (optInterval)
      {
         SRTED_auto_read(sr, optInterval);

         while (1) time_sleep(60);
      }
      else
      {
         SRTED_manual_read(sr);
      }

      SRTED_cancel(sr); /* Cancel sonar ranger. */

      pigpio_stop(pi); /* Disconnect from Pi. */
   }
   return 0;
}

