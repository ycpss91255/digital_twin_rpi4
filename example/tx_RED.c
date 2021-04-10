/*
tx_RED.c
2015-11-25
Public Domain
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include <pigpiod_if2.h>

/*

This programs transmits a quadrature signal on a pair of GPIO.  It is
intended to be used to test rotary encoder software.

REQUIRES

Nothing

TO BUILD

gcc -Wall -pthread -o tx_RED tx_RED.c -lpigpiod_if2

TO RUN

sudo pigpiod # If the daemon is not already running

then

./tx_RED -aGPIO -bGPIO [options]

./tx_RED -? for options

E.g.

./tx_RED -a5 -b6 -s20 -r-100

NOTE

This code sets the used GPIO to be outputs.  If you intend to feed the
signals into a program under test it is probably best to use different
GPIO in the two programs and connect them by wires.

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
      "Usage: tx_RED [OPTION] ...\n" \
      "   -a value, gpio A (0), 0-31,             default None\n" \
      "   -b value, gpio B (1), 0-31,             default None\n" \
      "   -s value, detents per second, 1-30000,  default 8\n" \
      "   -r value, repeats, +/- 1-65535,         default 1\n" \
      "   -h string, host name,                   default NULL\n" \
      "   -p value, socket port, 1024-32000,      default 8888\n" \
      "EXAMPLE\n" \
      "tx_RED -a10 -b12\n" \
      "   Transmit quadrature waveform on GPIO 10/12.\n\n");
}

int optGpioA = -1;
int optGpioB = -1;
int optDetentsPerSec = 8;
int optMicros = 1000000 / (4 * 8);
int optRepeats = 1;
char *optHost   = NULL;
char *optPort   = NULL;

int wid;

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

   while ((opt = getopt(argc, argv, "a:b:s:r:h:p:")) != -1)
   {
      switch (opt)
      {
         case 'a':
            i = getNum(optarg, &err);
            if ((i >= 0) && (i <= 31)) optGpioA = i;
            else fatal("invalid -a option (%s)", optarg);
            break;

         case 'b':
            i = getNum(optarg, &err);
            if ((i >= 0) && (i <= 31)) optGpioB = i;
            else fatal("invalid -b option (%s)", optarg);
            break;

         case 's':
            i = getNum(optarg, &err);
            if ((i >= 1) && (i <= 30000))
            {
               optDetentsPerSec = i;
               optMicros = 1000000 / (4 * i);
            }
            else fatal("invalid -s option (%s)", optarg);
            break;

         case 'r':
            i = getNum(optarg, &err);
            if ((i >= -65535) && (i <= 65535) && i) optRepeats = i;
            else fatal("invalid -r option (%s)", optarg);
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

void send_quadrature(int pi)
{
   char chain[8];
   int repeats;

   if (optRepeats < 0) repeats = -optRepeats; else repeats = optRepeats;

   chain[0] = 255;
   chain[1] = 0; /*Start of wave block. */
   chain[2] = wid;
   chain[3] = 255;
   chain[4] = 1; /* Repeat optRepeats. */
   chain[5] = repeats & 255;
   chain[6] = (repeats >> 8) & 255;

   wave_chain(pi, chain, 7);

   while (wave_tx_busy(pi)) time_sleep(0.05);
}

int main(int argc, char *argv[])
{
   int pi;
   uint32_t A, B;

   initOpts(argc, argv);

   if ((optGpioA < 0) || (optGpioB < 0) || (optGpioA == optGpioB))
   {
      fprintf(stderr, "You must specify GPIO A and GPIO B.\n");
      exit(0);
   }

   A = 1<<optGpioA;
   B = 1<<optGpioB;

   pi = pigpio_start(optHost, optPort); /* Connect to Pi. */

   if (pi >= 0)
   {
      set_mode(pi, optGpioA, PI_OUTPUT);
      set_mode(pi, optGpioB, PI_OUTPUT);

      wave_add_new(pi);

      if (optRepeats < 0)
      {
         /* Clockwise. */

         wave_add_generic(pi, 4, (gpioPulse_t[])
         {
            {0, A, optMicros},
            {0, B, optMicros},
            {A, 0, optMicros},
            {B, 0, optMicros},
         });

         wid = wave_create(pi);
      }
      else
      {
         /* Counter clockwise. */

         wave_add_generic(pi, 4, (gpioPulse_t[])
         {
            {0, B, optMicros},
            {0, A, optMicros},
            {B, 0, optMicros},
            {A, 0, optMicros},
         });

         wid = wave_create(pi);
      }

      send_quadrature(pi);

      wave_delete(pi, wid);

      pigpio_stop(pi);
   }
   return 0;
}

