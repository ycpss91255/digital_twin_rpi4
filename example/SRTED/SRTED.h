/*
SRTED.h
2015-11-16
Public Domain
*/

#ifndef SRTED_H
#define SRTED_H

struct SRTED_s;

typedef struct SRTED_s SRTED_t;

#define SRTE_GOOD      0
#define SRTE_BAD_RANGE 1
#define SRTE_TIMEOUT   2

typedef struct
{
   int pi;
   int trig;
   int echo;
   int status; /* Good, bad range, or timeout */
   int round_trip_micros; /* micros taken to travel to the object and back */
   float range_cms; /* range in centimetres to the object */
   double timestamp; /* time of last reading (good or not) */
   double tov; /* time of validity of last good range */
} SRTED_data_t;

typedef void (*SRTED_CB_t)(SRTED_data_t);

/*
SRTED starts a sonar ranger sensor on Pi pi with GPIO trig
for the trigger and GPIO echo for the echo.

SRTED_config may be used to exclude readings less than min_cms
centimetres and more than max_cms centimetres.  If such a reading
is made then a status of SRTED_BAD_RANGE will be returned along
with the previous reading.

If cb_func is not null it will be called at each new reading
whether the received data is valid or not.  The callback
receives a SRTED_data_t object.

If cb_func is null then the SRTED_ready function should be
called to check for new data which may then be retrieved by
a call to SRTED_data.

A single reading may be triggered with SRTED_manual_read.

A reading may be triggered at regular intervals using
SRTED_auto_read.  The mimimum permitted read interval
is 0.1 seconds.

At program end the DHTXX sensor should be cancelled using
SRTED_cancel.  This releases system resources.
*/

SRTED_t     *SRTED             (int pi,
                                int trig,
                                int echo,
                                SRTED_CB_t cb_func);

void         SRTED_config      (SRTED_t *self, float min_cms, float max_cms);

void         SRTED_cancel      (SRTED_t *self);

int          SRTED_ready       (SRTED_t *self);

SRTED_data_t SRTED_data        (SRTED_t *self);

void         SRTED_manual_read (SRTED_t *self);

void         SRTED_auto_read   (SRTED_t *self, float seconds);

#endif

