#!/bin/sh
#
# Just creates the modalias elements in an appstream file.
#
# Unfortunately, can't work on the bare comedi source; the input files
# needed a good bit of manual editing.

tr a-z A-Z < pci.devices.2 | sed -e '/^$/d' -e 's/^ */  <modalias>/' -e 's/{0X/pci:v0000/' -e 's/, 0X/d0000/' -e 's/},/*/' -e 's|$|</modalias>|'

tr a-z A-Z < pcmcia.devices | sed -e '/^$/d' -e 's/^\t*/  <modalias>/' -e 's/PCMCIA_DEVICE_MANF_CARD(0X/pcmcia:m/' -e 's/, 0X/c/' -e 's/),/*/' -e 's|$|</modalias>|'
