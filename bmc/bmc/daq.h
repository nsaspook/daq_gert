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

#include <stdlib.h>
#include <stdio.h> /* for printf() */
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <comedilib.h>
#include <signal.h>
#include <time.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>

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

#define JUST_BITS true

    struct didata {
        uint32_t D0 : 1; // 
        uint32_t D1 : 1; // 
        uint32_t D2 : 1; // 
        uint32_t D3 : 1; // 
        uint32_t D4 : 1; // 
        uint32_t D5 : 1; // 
        uint32_t D6 : 1; // 
        uint32_t D7 : 1; // 
    };

    union dio_buf_type {
        uint32_t dio_buf;
        struct didata d;
    };

    struct bmcdata {
        double pv_voltage, cc_voltage, input_voltage, b1_voltage, b2_voltage, system_voltage, logic_voltage;
        double pv_current, cc_current, battery_current;
        struct didata datain;
        union dio_buf_type dataout;
        int32_t adc_sample[32];
        int32_t dac_sample[32];
        int32_t utc;
    };

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

