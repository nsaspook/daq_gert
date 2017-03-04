/*
 * @(#)$Id: xy.c,v 2.0 2008/12/17 17:35:46 baccala Exp $
 *
 * Copyright (C) 1996 - 1999 Tim Witham <twitham@quiknet.com>
 *
 * (see the files README and COPYING for more details)
 *
 * xoscope external math filter command example (xy mode) in C.
 *
 * See operl for an example that can do arbitrary math at run-time.
 *
 * An external oscope command must continuously read two shorts from
 * stdin and write one short to stdout.  It must exit when stdin
 * closes.  The two shorts read in are the current input samples from
 * channel 1 & 2.  The short written out should be the desired math
 * function result.
 *
 * This example provides an X-Y display window as an external command.
 *
 * usage: xy [samples]
 *
 * The command-line argument is the number of samples per screenful.
 * You may need to use this if you resize the window.  (default = 640)
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include "config.h"
#include "display.h"

int mode = 2, h_points = 640;

/* handle single key commands */
void
handle_key(char c)
{
  switch (c) {
  case '\e':			/* Esc */
    exit(0);
    break;
  case '\r':			/* Enter */
  case '\n':
    clear();
    break;
  case '!':
    mode++;			/* point, point accumulate, line, line acc. */
    if (mode > 3)
      mode = 0;
    clear();
    break;
  }
}

/* get and plot one screen full of data */
void
animate(void *data)
{
  static short x, y, z = 0, X, Y, buff[sizeof(short)];
  static int i, j;

  if (!(mode % 2))
    clear();
  for (j = 0 ; j < h_points ; j++) {
    /* Read two shorts from stdin (channel 1 & 2 */
    if ((i = read(0, buff, sizeof(short))) != sizeof(short))
      exit(i);
    x = (short)(*buff);	/* get channel 1 sample, increment pointer */

    if ((i = read(0, buff, sizeof(short))) != sizeof(short))
      exit(i);
    y = (short)(*buff);	/* get channel 2 sample, increment pointer */

    if ((i = write(1, &z, sizeof(short))) != sizeof(short))
      exit(i);		/* write one short to stdout */

    if (mode < 2)
      DrawPixel(128 + x, 127 - y);
    else if (j)
      DrawLine(128 + X, 127 - Y, 128 + x, 127 - y);
    X = x; Y = y;
  }
  SyncDisplay();
  AddTimeOut(MSECREFRESH, animate, NULL);
}

int
main(int argc, char **argv)	/* main program */
{
  if ((argc = OpenDisplay(argc, argv)) == 0)
    exit(1);
  if (argc > 1)
    h_points = strtol(argv[1], NULL, 0);

  init_widgets();
  clear();
  animate(NULL);
  MainLoop();
  exit(0);
}
