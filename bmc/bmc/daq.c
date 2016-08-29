#include "daq.h"
#include <stdio.h>	/* for printf() */
#include <unistd.h>

int subdev_ai = 0; /* change this to your input subdevice */
int chan_ai = 0; /* change this to your channel */
int range_ai = 0; /* more on this later */
int aref_ai = AREF_GROUND; /* more on this later */
int maxdata_ai, ranges_ai, channels_ai;

int subdev_ao = 0; /* change this to your input subdevice */
int chan_ao = 0; /* change this to your channel */
int range_ao = 0; /* more on this later */
int aref_ao = AREF_GROUND; /* more on this later */
int maxdata_ao, ranges_ao, channels_ao;

int subdev_dio = 0; /* change this to your input subdevice */
int chan_dio = 0; /* change this to your channel */
int range_dio = 0; /* more on this later */
int maxdata_dio, ranges_dio, channels_dio, datain_dio;

comedi_t *it;
comedi_range *ad_range, *da_range;
int8_t ADC_OPEN = FALSE, DIO_OPEN = FALSE, ADC_ERROR = FALSE, DEV_OPEN = FALSE,
        DIO_ERROR = FALSE, HAS_AO = FALSE, DAC_ERROR = FALSE;

int init_daq(double min_range, double max_range, int range_update) {
    int i = 0;

    if (!DEV_OPEN) {
        it = comedi_open("/dev/comedi0");
        if (it == NULL) {
            comedi_perror("comedi_open");
            ADC_OPEN = FALSE;
            DEV_OPEN = FALSE;
            return -1;
        }
        DEV_OPEN = TRUE;
    }

    subdev_ai = comedi_find_subdevice_by_type(it, COMEDI_SUBD_AI, subdev_ai);
    if (subdev_ai < 0) {
        return -2;
        ADC_OPEN = FALSE;
    }


    subdev_ao = comedi_find_subdevice_by_type(it, COMEDI_SUBD_AO, subdev_ao);
    if (subdev_ao < 0) {
        HAS_AO = FALSE;
    } else {
        HAS_AO = TRUE;
    }

    printf("Subdev AI  %i ", subdev_ai);
    channels_ai = comedi_get_n_channels(it, subdev_ai);
    printf("Analog  Channels %i ", channels_ai);
    maxdata_ai = comedi_get_maxdata(it, subdev_ai, i);
    printf("Maxdata %i ", maxdata_ai);
    ranges_ai = comedi_get_n_ranges(it, subdev_ai, i);
    printf("Ranges %i ", ranges_ai);
    ad_range = comedi_get_range(it, subdev_ai, i, ranges_ai - 1);
    if (range_update) {
        ad_range->min = min_range;
        ad_range->max = max_range;
    }
    printf(": ad_range .min = %.3f, max = %.3f\r\n", ad_range->min,
            ad_range->max);

    if (HAS_AO) {
        printf("Subdev AO  %i ", subdev_ao);
        channels_ao = comedi_get_n_channels(it, subdev_ao);
        printf("Analog  Channels %i ", channels_ao);
        maxdata_ao = comedi_get_maxdata(it, subdev_ao, i);
        printf("Maxdata %i ", maxdata_ao);
        ranges_ao = comedi_get_n_ranges(it, subdev_ao, i);
        printf("Ranges %i ", ranges_ao);
        da_range = comedi_get_range(it, subdev_ao, i, ranges_ao - 1);
        printf(": da_range .min = %.3f, max = %.3f\r\n", da_range->min,
                da_range->max);
    }

    ADC_OPEN = TRUE;
    comedi_set_global_oor_behavior(COMEDI_OOR_NUMBER);
    return 0;
}


int init_dac(double min_range, double max_range, int range_update) {
    int i = 0;

    if (!DEV_OPEN) {
        it = comedi_open("/dev/comedi0");
        if (it == NULL) {
            comedi_perror("comedi_open");
            ADC_OPEN = FALSE;
            DEV_OPEN = FALSE;
            return -1;
        }
        DEV_OPEN = TRUE;
    }

    subdev_ao = comedi_find_subdevice_by_type(it, COMEDI_SUBD_AO, subdev_ao);
    if (subdev_ao < 0) {
        HAS_AO = FALSE;
    } else {
        HAS_AO = TRUE;
    }

    if (HAS_AO) {
        printf("Subdev AO  %i ", subdev_ao);
        channels_ao = comedi_get_n_channels(it, subdev_ao);
        printf("Analog  Channels %i ", channels_ao);
        maxdata_ao = comedi_get_maxdata(it, subdev_ao, i);
        printf("Maxdata %i ", maxdata_ao);
        ranges_ao = comedi_get_n_ranges(it, subdev_ao, i);
        printf("Ranges %i ", ranges_ao);
        da_range = comedi_get_range(it, subdev_ao, i, ranges_ao - 1);
        printf(": da_range .min = %.3f, max = %.3f\r\n", da_range->min,
                da_range->max);
    }

    comedi_set_global_oor_behavior(COMEDI_OOR_NUMBER);
    return 0;
}

int adc_range(double min_range, double max_range) {
    if (ADC_OPEN) {
        ad_range->min = min_range;
        ad_range->max = max_range;
        return 0;
    } else {
        return -1;
    }
}

int dac_range(double min_range, double max_range) {
    if (ADC_OPEN) {
        da_range->min = min_range;
        da_range->max = max_range;
        return 0;
    } else {
        return -1;
    }
}

int set_dac_volts(int chan, double voltage) {
    lsampl_t data;
    int retval;

    DAC_ERROR = FALSE;
    data = comedi_from_phys(voltage, da_range, maxdata_ao);
    retval = comedi_data_write(it, subdev_ao, chan, range_ao, aref_ao, data);
    if (retval < 0) {
        comedi_perror("comedi_data_write in set_dac_volts");
        DAC_ERROR = TRUE;
    }
    return retval;
}

int set_dac_raw(int chan, lsampl_t voltage) {
    int retval;

    DAC_ERROR = FALSE;
    retval = comedi_data_write(it, subdev_ao, chan, range_ao, aref_ao, voltage);
    if (retval < 0) {
        comedi_perror("comedi_data_write in set_dac_raw");
        DAC_ERROR = TRUE;
    }
    return retval;
}

double get_adc_volts(int chan) {
    lsampl_t data;
    int retval;

    ADC_ERROR = FALSE;
    retval = comedi_data_read_delayed(it, subdev_ai, chan, range_ai, aref_ai, &data, 1000);
    if (retval < 0) {
        comedi_perror("comedi_data_read in get_adc_volts");
        ADC_ERROR = TRUE;
        return 0.0;
    }
    bmc.adc_sample[chan] = data;
    return comedi_to_phys(data, ad_range, maxdata_ai);
}

int set_dio_output(int chan) {
    return comedi_dio_config(it,
            subdev_dio,
            chan,
            COMEDI_OUTPUT);
}

int set_dio_input(int chan) {
    return comedi_dio_config(it,
            subdev_dio,
            chan,
            COMEDI_INPUT);
}

int get_dio_bit(int chan) {
    lsampl_t data;
    int retval;

    DIO_ERROR = FALSE;
    retval = comedi_data_read(it, subdev_dio, chan, range_dio, aref_dio, &data);
    if (retval < 0) {
        comedi_perror("comedi_data_read in get_dio_bits");
        DIO_ERROR = TRUE;
        return 0;
    }
    if (data != 0) data = 1;
    return data;
}

int put_dio_bit(int chan, int bit_data) {
    lsampl_t data = bit_data;
    int retval;

    DIO_ERROR = FALSE;
    retval = comedi_data_write(it, subdev_dio, chan, range_dio, aref_dio, data);
    if (retval < 0) {
        comedi_perror("comedi_data_write in put_dio_bits");
        DIO_ERROR = TRUE;
        return -1;
    }
    return 0;
}

int init_dio(void) {
    int i = 0;

    if (!DEV_OPEN) {
        it = comedi_open("/dev/comedi0");
        if (it == NULL) {
            comedi_perror("comedi_open");
            DIO_OPEN = FALSE;
            DEV_OPEN = FALSE;
            return -1;
        }
        DEV_OPEN = TRUE;
    }

    subdev_dio = comedi_find_subdevice_by_type(it, COMEDI_SUBD_DIO, subdev_dio);
    if (subdev_dio < 0) {
        return -1;
        DIO_OPEN = FALSE;
    }

    printf("Subdev DIO %i ", subdev_dio);
    channels_dio = comedi_get_n_channels(it, subdev_dio);
    printf("Digital Channels %i ", channels_dio);
    maxdata_dio = comedi_get_maxdata(it, subdev_dio, i);
    printf("Maxdata %i ", maxdata_dio);
    ranges_dio = comedi_get_n_ranges(it, subdev_dio, i);
    printf("Ranges %i \r\n", ranges_dio);
    DIO_OPEN = TRUE;
    return 0;
}

/*
typedef struct bmcdata {
    double pv_voltage,cc_voltage, input_voltage, b1_voltage, b2_voltage, system_voltage,logic_voltage;
    double pv_current,cc_current,battery_current;
    struct didata datain;
    struct dodata dataout;
    int32_t utc;
}
 */
int get_data_sample(void) {
    int i;

    bmc.pv_voltage = get_adc_volts(PVV_C);
    bmc.cc_voltage = get_adc_volts(CCV_C);
    //        bmc.input_voltage = lp_filter(get_adc_volts(INV_C), INV_C, TRUE);
    //        bmc.b1_voltage = lp_filter(get_adc_volts(B1V_C), B1V_C, TRUE);
    //        bmc.b2_voltage = lp_filter(get_adc_volts(B2V_C), B2V_C, TRUE);
    //        bmc.pv_current = lp_filter(get_adc_volts(PVC_C), PVC_C, TRUE);
    //        bmc.cc_current = lp_filter(get_adc_volts(CCC_C), CCC_C, TRUE);
    //        bmc.battery_current = lp_filter(get_adc_volts(BAC_C), BAC_C, TRUE);
    //    bmc.system_voltage = get_adc_volts(SYV_C);
    //    bmc.logic_voltage = get_adc_volts(VD5_C);

    bmc.datain.D0 = get_dio_bit(6); // GPIO 25
    bmc.datain.D1 = get_dio_bit(7); // GPIO 4
    bmc.datain.D2 = get_dio_bit(0); // read output bit wpi 0
    bmc.datain.D3 = get_dio_bit(1); // read output bit wpi 1
    //    bmc.datain.D4 = get_dio_bit(12);
    //    bmc.datain.D5 = get_dio_bit(13);
    //    bmc.datain.D6 = get_dio_bit(15); // GPIO 14 
    //    bmc.datain.D7 = get_dio_bit(16); // GPIO 15 
    put_dio_bit(0, bmc.dataout.D0); // GPIO 17
    put_dio_bit(1, bmc.dataout.D1); // GPIO 18
    put_dio_bit(2, bmc.dataout.D2); // GPIO 21
    put_dio_bit(3, bmc.dataout.D3); // GPIO 22
    put_dio_bit(4, bmc.dataout.D4);
    put_dio_bit(5, bmc.dataout.D5);
    put_dio_bit(6, bmc.dataout.D6);
    put_dio_bit(7, bmc.dataout.D7);
}

double lp_filter(double new, int bn, int slow) // low pass filter, slow rate of change for new, LPCHANC channels, slow/fast select (-1) to zero channel
{
    static double smooth[LPCHANC] = {0};
    double lp_speed, lp_x;

    if ((bn >= LPCHANC) || (bn < 0)) // check for proper array position
        return new;
    if (slow) {
        lp_speed = 0.033;
    } else {
        lp_speed = 0.125;
    }
    lp_x = ((smooth[bn]*100.0) + (((new * 100.0)-(smooth[bn]*100.0)) * lp_speed)) / 100.0;
    smooth[bn] = lp_x;
    if (slow == (-1)) { // reset and return zero
        lp_x = 0.0;
        smooth[bn] = 0.0;
    }
    return lp_x;
}
