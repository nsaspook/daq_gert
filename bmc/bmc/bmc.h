/* 
 * File:   bmc.h
 * Author: root
 *
 * Created on September 21, 2012, 12:54 PM
 */

#ifndef BMC_H
#define	BMC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define TRUE    1
#define FALSE   0
    
struct didata {
    unsigned char D0 : 1; // 
    unsigned char D1 : 1; // 
    unsigned char D2 : 1; // 
    unsigned char D3 : 1; // 
    unsigned char D4 : 1; // 
    unsigned char D5 : 1; // 
    unsigned char D6 : 1; // 
    unsigned char D7 : 1; // 
} volatile ditype;

struct dodata {
     unsigned char D0 : 1; // 
     unsigned char D1 : 1; // 
     unsigned char D2 : 1; // 
     unsigned char D3 : 1; // 
     unsigned char D4 : 1; // 
     unsigned char D5 : 1; // 
     unsigned char D6 : 1; // 
     unsigned char D7 : 1; // 
} volatile dotype;
     
typedef struct bmcdata {
    double pv_voltage,cc_voltage, input_voltage, b1_voltage, b2_voltage, system_voltage,logic_voltage;
    double pv_current,cc_current,battery_current;
    struct didata datain;
    struct dodata dataout;
    int32_t adc_sample[32];
    int32_t utc;
}
volatile bmctype;

#ifdef	__cplusplus
}
#endif

#endif	/* BMC_H */

