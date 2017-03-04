/*
 * @(#)$Id: xy_sx.c,v 2.0 2008/12/17 17:35:46 baccala Exp $
 *
 * Copyright (C) 1996 - 2001 Tim Witham <twitham@quiknet.com>
 *
 * (see the files README and COPYING for more details)
 *
 * This file implements the libsx specific UI for the xy command
 *
 */

#include "config.h"
#if HAVE_X11_SX_LIBSX_H
#include <X11/SX/libsx.h>
#else
#include <libsx.h>
#endif

Widget quit;			/* quit button */
Widget plot[5];			/* plot menu */
Widget draw_widget;		/* xy drawing area */
int bg, fg;

extern int mode, quit_key_pressed;
extern void handle_key();

/* clear the drawing area and reset the menu check marks */
void
clear()
{
  int i;
  for (i = 0 ; i < 4 ; i++) {
    SetMenuItemChecked(plot[i + 1], mode == i);
  }
  ClearDrawArea();
}

/* callback to redisplay the drawing area */
void
redisplay(Widget w, int new_width, int new_height, void *data) {
  clear();
}

/* callback for keypress events on the drawing area */
void
keys(Widget w, char *input, int up_or_down, void *data)
{
  if (!up_or_down)		/* 0 press, 1 release */
    return;
  if (input[1] == '\0')		/* single character to handle */
    if (input[0] > 0)
      handle_key(input[0]);
}

/* quit button callback */
void
dismiss(Widget w, void *data)
{
  quit_key_pressed = 1;
}

/* plot mode menu callback */
void
plotmode(Widget w, void *data)
{
  char *c = (char *)data;

  mode = *c - '0';
  clear();
}

/* build the menubar and drawing area */
void
init_widgets()
{
  quit = MakeButton("Quit", dismiss, NULL);

  plot[0] = MakeMenu(" Plot Mode ");
  plot[1] = MakeMenuItem(plot[0], "Point", plotmode, "0");
  plot[2] = MakeMenuItem(plot[0], "Point Accumulate", plotmode, "1");
  plot[3] = MakeMenuItem(plot[0], "Line", plotmode, "2");
  plot[4] = MakeMenuItem(plot[0], "Line Accumulate", plotmode, "3");
  SetWidgetPos(plot[0], PLACE_RIGHT, quit, NO_CARE, NULL);

  draw_widget = MakeDrawArea(256, 256, redisplay, NULL);
  SetKeypressCB(draw_widget, keys);
  SetWidgetPos(draw_widget, PLACE_UNDER, quit, NO_CARE, NULL);

  ShowDisplay();
  bg = GetNamedColor("black");
  fg = GetNamedColor("white");
  SetBgColor(draw_widget, bg);
  SetFgColor(draw_widget, fg);
}
