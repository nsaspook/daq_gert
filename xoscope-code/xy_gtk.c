/*
 * @(#)$Id: xy_gtk.c,v 2.0 2008/12/17 17:35:46 baccala Exp $
 *
 * Copyright (C) 1996 - 1999 Tim Witham <twitham@quiknet.com>
 *
 * (see the files README and COPYING for more details)
 *
 * This file implements the GTK specific UI for the xy command
 *
 */

#include <gtk/gtk.h>
#include "display.h"
#include "com_gtk.h"

extern int mode;
extern void handle_key();

GdkPixmap *pixmap;

/* clear the drawing area and reset the menu check marks */
void
clear()
{
  ClearDrawArea();
}

/* plot mode menu callback */
void
plotmode(GtkWidget *w, void *data)
{
  char *c = (char *)data;

  mode = *c - '0';
  clear();
}

/* Create a new backing pixmap of the appropriate size */
static gint
configure_event (GtkWidget *widget, GdkEventConfigure *event)
{
  if (pixmap)
    gdk_pixmap_unref(pixmap);

  pixmap = gdk_pixmap_new(widget->window,
			  widget->allocation.width,
			  widget->allocation.height,
			  -1);
  ClearDrawArea();
  return TRUE;
}

void
get_main_menu(GtkWidget *window, GtkWidget ** menubar)
{
  static GtkItemFactoryEntry menu_items[] =
  {
    {"/Close", NULL, hit_key, (int)"\e", NULL},
    {"/Plot Mode", NULL, NULL, 0, "<Branch>"},
    {"/Plot Mode/tear", NULL, NULL, 0, "<Tearoff>"},
    {"/Plot Mode/Point", NULL, plotmode, (int)"0", NULL},
    {"/Plot Mode/Point Accumulate", NULL, plotmode, (int)"1", NULL},
    {"/Plot Mode/Line", NULL, plotmode, (int)"2", NULL},
    {"/Plot Mode/Line Accumulate", NULL, plotmode, (int)"3", NULL},
  };
  gint nmenu_items = sizeof(menu_items) / sizeof(menu_items[0]);

  GtkItemFactory *factory;

  factory = gtk_item_factory_new(GTK_TYPE_MENU_BAR, "<main>", NULL);
  gtk_item_factory_create_items(factory, nmenu_items, menu_items, NULL);

  if (menubar)
    *menubar = gtk_item_factory_get_widget (factory, "<main>");
}

/* build the menubar and drawing area */
void
init_widgets()
{

  int i;
  GtkWidget *menubar;

  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_signal_connect(GTK_OBJECT (window), "delete_event",
		     GTK_SIGNAL_FUNC (delete_event), NULL);
  gtk_signal_connect(GTK_OBJECT(window),"key_press_event",
		     (GtkSignalFunc) key_press_event, NULL);

  vbox = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(window), vbox);

  get_main_menu(window, &menubar);
  gtk_box_pack_start(GTK_BOX(vbox), menubar, FALSE, TRUE, 0);
  gtk_widget_show(menubar);

  drawing_area = gtk_drawing_area_new();
  gtk_drawing_area_size(GTK_DRAWING_AREA(drawing_area), 256, 256);
  gtk_signal_connect(GTK_OBJECT(drawing_area), "expose_event",
		     GTK_SIGNAL_FUNC(expose_event), NULL);
  gtk_signal_connect(GTK_OBJECT(drawing_area),"configure_event",
		     GTK_SIGNAL_FUNC(configure_event), NULL);

  gtk_box_pack_start(GTK_BOX(vbox), drawing_area, TRUE, TRUE, 0);
  gtk_widget_show(drawing_area);
  gtk_widget_show(vbox);
  gtk_widget_show(window);

  gc = gdk_gc_new(drawing_area->window);
  for (i=0 ; i < 16 ; i++) {
    color[i] = i;
    gdk_color_parse(colors[i], &gdkcolor[i]);
    gdkcolor[i].pixel = (gulong)(gdkcolor[i].red * 65536 +
				 gdkcolor[i].green * 256 + gdkcolor[i].blue);
    gdk_color_alloc(gtk_widget_get_colormap(window), &gdkcolor[i]);
/*      printf("%d %s %ld:\tr:%d g:%d b:%d\n", */
/*  	   i, colors[i], gdkcolor[i].pixel, */
/*  	   gdkcolor[i].red, gdkcolor[i].green, gdkcolor[i].blue); */
  }
  gdk_gc_set_background(gc, &gdkcolor[0]);
  SetColor(15);
  ClearDrawArea();
}

void
MainLoop()
{
  animate(NULL);
  gtk_main();
}
