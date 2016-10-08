/* 
 * File:   daq.h
 * Author: root
 *
 * Created on September 21, 2012, 6:49 PM
 */

#ifndef DAQ_H
#define DAQ_H

#ifdef __cplusplus
extern "C" {
#endif

#define PVV_C   0
#define CCV_C   1
#define SYV_C   2
#define B1V_C   3
#define B2V_C   4
#define INV_C   5
#define VD5_C   7
#define PVC_C   8
#define CCC_C   9
#define BAC_C   10    

#define LPCHANC        16

#include <stdint.h>
#include <comedilib.h>
#include "bmc.h"

    int subdev_ai; /* change this to your input subdevice */
    int chan_ai; /* change this to your channel */
    int range_ai; /* more on this later */
    int aref_ai; /* more on this later */
    int maxdata_ai, ranges_ai, channels_ai;

    int subdev_dio; /* change this to your input subdevice */
    int chan_dio; /* change this to your channel */
    int range_dio; /* more on this later */
    int aref_dio; /* more on this later */
    int maxdata_dio, ranges_dio, channels_dio, datain_dio;

    comedi_t *it;
    comedi_range *ad_range;
    int8_t ADC_OPEN, DIO_OPEN, ADC_ERROR, DEV_OPEN, DIO_ERROR;

    extern struct bmcdata bmc;
    extern struct didata datain;
    extern struct dodata dataout;

    int init_daq(double, double, int);
    int init_dac(double, double, int);
    int init_dio(void);
    int adc_range(double, double);
    int dac_range(double, double);
    double get_adc_volts(int);
    int set_dac_volts(int, double);
    int set_dac_raw(int, lsampl_t);
    int get_dio_bit(int);
    int put_dio_bit(int, int);
    int set_dio_input(int);
    int set_dio_output(int);
    int get_data_sample(void);
    double lp_filter(double, int, int);
#ifdef __cplusplus
}
#endif

#endif /* DAQ_H */

