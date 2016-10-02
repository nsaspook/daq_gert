diff --git a/arch/arm/boot/dts/overlays/Makefile b/arch/arm/boot/dts/overlays/Makefile
index ea9e3bb..ef33366 100644
--- a/arch/arm/boot/dts/overlays/Makefile
+++ b/arch/arm/boot/dts/overlays/Makefile
@@ -104,6 +104,7 @@ dtbo-$(RPI_DT_OVERLAYS) += vga666.dtbo
 dtbo-$(RPI_DT_OVERLAYS) += w1-gpio.dtbo
 dtbo-$(RPI_DT_OVERLAYS) += w1-gpio-pullup.dtbo
 dtbo-$(RPI_DT_OVERLAYS) += wittypi.dtbo
+dtbo-$(RPI_DT_OVERLAYS) += rpi-spigert.dtbo
 
 targets += dtbs dtbs_install
 targets += $(dtbo-y)
diff --git a/drivers/spi/Kconfig b/drivers/spi/Kconfig
index c9d1558..8a29e8f 100644
--- a/drivers/spi/Kconfig
+++ b/drivers/spi/Kconfig
@@ -689,6 +689,14 @@ config SPI_SPIDEV
 	  Note that this application programming interface is EXPERIMENTAL
 	  and hence SUBJECT TO CHANGE WITHOUT NOTICE while it stabilizes.
 
+config SPI_COMEDI
+        tristate "Comedi mode SPI device driver support for daq_gert"
+        help
+          This supports the daq_gert Comedi mode SPI protocol driver.
+
+          Note that this application programming interface is EXPERIMENTAL
+          and hence SUBJECT TO CHANGE WITHOUT NOTICE while it stabilizes.
+
 config SPI_TLE62X0
 	tristate "Infineon TLE62X0 (for power switching)"
 	depends on SYSFS
diff --git a/drivers/spi/spi.c b/drivers/spi/spi.c
index dee1cb8..5e404b1 100644
--- a/drivers/spi/spi.c
+++ b/drivers/spi/spi.c
@@ -989,7 +989,7 @@ static int spi_transfer_one_message(struct spi_master *master,
 				keep_cs = true;
 			} else {
 				spi_set_cs(msg->spi, false);
-				udelay(10);
+				udelay(xfer->cs_change_usecs ? xfer->cs_change_usecs : 10);
 				spi_set_cs(msg->spi, true);
 			}
 		}
diff --git a/drivers/staging/comedi/Kconfig b/drivers/staging/comedi/Kconfig
index ac0f010..e48aca8 100644
--- a/drivers/staging/comedi/Kconfig
+++ b/drivers/staging/comedi/Kconfig
@@ -84,6 +84,14 @@ config COMEDI_SERIAL2002
 	  To compile this driver as a module, choose M here: the module will be
 	  called serial2002.
 
+config COMEDI_DAQ_GERT
+        tristate "Driver for RPi GPIO and SPI connected hardware"
+        ---help---
+          Enable support for RPi GPIO and SPI connected hardware
+
+          To compile this driver as a module, choose M here: the module will be
+          called daq_gert.
+
 config COMEDI_SSV_DNP
 	tristate "SSV Embedded Systems DIL/Net-PC support"
 	depends on X86_32 || COMPILE_TEST
diff --git a/drivers/staging/comedi/drivers/Makefile b/drivers/staging/comedi/drivers/Makefile
index c3b8f2d..53b04e1 100644
--- a/drivers/staging/comedi/drivers/Makefile
+++ b/drivers/staging/comedi/drivers/Makefile
@@ -11,6 +11,7 @@ obj-$(CONFIG_COMEDI_BOND)		+= comedi_bond.o
 obj-$(CONFIG_COMEDI_TEST)		+= comedi_test.o
 obj-$(CONFIG_COMEDI_PARPORT)		+= comedi_parport.o
 obj-$(CONFIG_COMEDI_SERIAL2002)		+= serial2002.o
+obj-$(CONFIG_COMEDI_DAQ_GERT)           += daq_gert.o
 
 # Comedi ISA drivers
 obj-$(CONFIG_COMEDI_AMPLC_DIO200_ISA)	+= amplc_dio200.o
diff --git a/include/linux/spi/spi.h b/include/linux/spi/spi.h
index cce80e6..d967398 100644
--- a/include/linux/spi/spi.h
+++ b/include/linux/spi/spi.h
@@ -698,6 +698,7 @@ struct spi_transfer {
 	u32		speed_hz;
 
 	struct list_head transfer_list;
+        u8              cs_change_usecs;
 };
 
 /**
