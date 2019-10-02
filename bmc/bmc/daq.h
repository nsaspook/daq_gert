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
	
#define JUST_BITS	false

#include <stdint.h>
#include <comedilib.h>
#include "bmc.h"

	extern volatile struct bmcdata bmc;
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

