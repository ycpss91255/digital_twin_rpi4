/*
WD.c
2015-11-25
Public Domain
*/

#include <stdio.h>
#include <stdlib.h>

#include <pigpiod_if2.h>

#include "WD.h"

struct _WD_s
{
   int pi;
   int gpioGreen;
   int gpioWhite;
   int timeout;
   int gap;
   uint32_t timeout_tick;
   int cbGreenId;
   int cbWhiteId;
   int in_code;
   int ready;
   int bits;
   WD_CB_t cb_func;
   uint64_t code;
   uint32_t code_timeout;
   WD_data_t data;
};

static void _cb(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
   /*
      Accumulate bits until both gpios 0 and 1 timeout.
   */

   WD_t *self=user;
   int ticks;

   if (level == 0) /* a falling edge indicates a new bit */
   {
      if (self->in_code)
      {
         ticks = tick - self->timeout_tick;

         if (ticks > 0)
         {
            /* Old message has timed out. */

            self->data.bits = self->bits;
            self->data.code = self->code;
            self->ready = 1;

            (self->cb_func)(self->data);

            self->bits = 0;
            self->code = 0;
            self->code_timeout = 0;
         }
      }
      else
      {
         self->in_code = 1;

         self->bits = 0;
         self->code = 0;
         self->code_timeout = 0;

         set_watchdog(self->pi, self->gpioGreen, self->timeout);
         set_watchdog(self->pi, self->gpioWhite, self->timeout);
      }

      self->timeout_tick = tick + self->gap;

      self->bits++;
      self->code <<= 1;

      if (gpio == self->gpioGreen)
      {
         self->code_timeout &= 2; /* clear gpio 0 timeout */
      }
      else
      {
         self->code_timeout &= 1; /* clear gpio 1 timeout */
         self->code |= 1;
      }
   }
   else if (level == PI_TIMEOUT)
   {
      ticks = tick - self->timeout_tick;

      if (ticks > 0)
      {
         if (self->in_code)
         {
            if (gpio == self->gpioGreen)
            {
               self->code_timeout |= 1; /* timeout 0 */
            }
            else
            {
               self->code_timeout |= 2; /* timeout 1 */
            }

            if (self->code_timeout == 3) /* both GPIO timed out */
            {
               set_watchdog(self->pi, self->gpioGreen, 0);
               set_watchdog(self->pi, self->gpioWhite, 0);

               self->in_code = 0;

               self->data.bits = self->bits;
               self->data.code = self->code;
               self->ready = 1;
               (self->cb_func)(self->data);
            }
         }
         else
         {
            set_watchdog(self->pi, self->gpioGreen, 0);
            set_watchdog(self->pi, self->gpioWhite, 0);
         }
      }
   }
}

WD_t *WD(
   int pi,
   int gpioGreen,
   int gpioWhite,
   int gap,
   WD_CB_t cb_func)
{
   /*
      Instantiate with the gpio for 0 (green wire), the gpio for 1
      (white wire), the callback function, and the timeout in
      milliseconds which indicates the end of a code.

      The callback is passed the code length in bits and the value.
   */

   WD_t *self;

   self = malloc(sizeof(WD_t));

   self->pi = pi;

   self->gpioGreen = gpioGreen;
   self->gpioWhite = gpioWhite;

   self->cb_func = cb_func;

   self->gap = gap;
   self->timeout = (gap+999)/1000;

   self->in_code = 0;
   self->ready = 0;

   self->data.pi = pi;
   self->data.gpioGreen = gpioGreen;
   self->data.gpioWhite = gpioWhite;

   set_mode(pi, gpioGreen, PI_INPUT);
   set_mode(pi, gpioWhite, PI_INPUT);

   set_pull_up_down(pi, gpioGreen, PI_PUD_UP);
   set_pull_up_down(pi, gpioWhite, PI_PUD_UP);

   self->cbGreenId = callback_ex(pi, gpioGreen, EITHER_EDGE, _cb, self);
   self->cbWhiteId = callback_ex(pi, gpioWhite, EITHER_EDGE, _cb, self);

   return self;
}

int WD_ready(WD_t *self)
{
   return self->ready;
}

WD_data_t WD_data(WD_t *self)
{
   self->ready = 0;
   return self->data;
}

void WD_cancel(WD_t *self)
{
   /*
      Cancel the Wiegand decoder.
   */

   if (self)
   {
      if (self->cbGreenId >= 0)
      {
         callback_cancel(self->cbGreenId);
         self->cbGreenId = -1;
      }

      if (self->cbWhiteId >= 0)
      {
         callback_cancel(self->cbWhiteId);
         self->cbWhiteId = -1;
      }

      free(self);
   }
}

