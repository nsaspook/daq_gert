/* 
 * File:   bmc.h
 * Author: root
 *
 * Created on September 21, 2012, 12:54 PM
 */

#ifndef BMC_H
#define BMC_H

#ifdef __cplusplus
extern "C" {
#endif

#define TRUE    1
#define FALSE   0

    struct didata {
        uint32_t D0 : 1; // 
        uint32_t D1 : 1; // 
        uint32_t D2 : 1; // 
        uint32_t D3 : 1; // 
        uint32_t D4 : 1; // 
        uint32_t D5 : 1; // 
        uint32_t D6 : 1; // 
        uint32_t D7 : 1; // 
    } volatile ditype;

    union dio_buf_type {
        uint32_t dio_buf;
        struct didata d;
    };

    typedef struct bmcdata {
        double pv_voltage, cc_voltage, input_voltage, b1_voltage, b2_voltage, system_voltage, logic_voltage;
        double pv_current, cc_current, battery_current;
        struct didata datain;
        union dio_buf_type dataout;
        int32_t adc_sample[32];
        int32_t dac_sample[32];
        int32_t utc;
    }
    volatile bmctype;

#ifdef __cplusplus
}
#endif

#endif /* BMC_H */

