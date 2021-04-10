/*
WD.h
2015-11-25
Public Domain
*/

#ifndef WD_H
#define WD_H

#include <stdint.h>

struct _WD_s;

typedef struct _WD_s WD_t;

typedef struct
{
   int pi;
   int gpioGreen;
   int gpioWhite;
   int bits;
   uint64_t code;
} WD_data_t;

typedef void (*WD_CB_t)(WD_data_t);

/*

WD starts a Wiegand decoder on Pi pi with GPIO gpioGreen (0),
GPIO gpioWhite (1), timeout gap, and callback cb_func.

gap microseconds without a new bit indicates the end of a code.

If cb_func is not null it will be called for each newly read code.

If cb_func is null WD_ready may be called to check for a new code which
can then be read with WD_data.

At program end the Wiegand decoder should be cancelled using
WDD_cancel.  This releases system resources.

*/

WD_t     *WD       (int pi,
                    int gpioGreen, /* 0 */
                    int gpioWhite, /* 1 */
                    int gap,
                    WD_CB_t cb_func);

int       WD_ready (WD_t *wieg);

WD_data_t WD_data  (WD_t *wieg);

void      WD_cancel(WD_t *wieg);

#endif

