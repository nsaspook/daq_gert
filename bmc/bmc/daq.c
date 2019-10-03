#include "daq.h"
#include <stdio.h> /* for printf() */
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

int subdev_di = 0; /* change this to your input subdevice */
int chan_di = 0; /* change this to your channel */
int range_di = 0; /* more on this later */
int maxdata_di, ranges_di, channels_di, datain_di;

int subdev_do = 0; /* change this to your input subdevice */
int chan_do = 0; /* change this to your channel */
int range_do = 0; /* more on this later */
int maxdata_do, ranges_do, channels_do, datain_do;

int subdev_dio; /* change this to your input subdevice */
int aref_dio; /* more on this later */

int subdev_counter; /* change this to your input subdevice */
int chan_counter = 0; /* change this to your channel */
int range_counter = 0; /* more on this later */
int maxdata_counter, ranges_counter, channels_counter, datain_counter;

comedi_t *it;
comedi_range *ad_range, *da_range;
bool ADC_OPEN = false, DIO_OPEN = false, ADC_ERROR = false, DEV_OPEN = false,
	DIO_ERROR = false, HAS_AO = false, DAC_ERROR = false, PWM_OPEN = false,
	PWM_ERROR = false;

int init_daq(double min_range, double max_range, int range_update)
{
	int i = 0;

	if (!DEV_OPEN) {
		it = comedi_open("/dev/comedi0");
		if (it == NULL) {
			comedi_perror("comedi_open");
			ADC_OPEN = false;
			DEV_OPEN = false;
			return -1;
		}
		DEV_OPEN = true;
	}

	subdev_ai = comedi_find_subdevice_by_type(it, COMEDI_SUBD_AI, subdev_ai);
	if (subdev_ai < 0) {
		return -2;
		ADC_OPEN = false;
	}


	subdev_ao = comedi_find_subdevice_by_type(it, COMEDI_SUBD_AO, subdev_ao);
	if (subdev_ao < 0) {
		HAS_AO = false;
	} else {
		HAS_AO = true;
	}

	printf("Subdev AI  %i ", subdev_ai);
	channels_ai = comedi_get_n_channels(it, subdev_ai);
	printf("Analog  Channels %i ", channels_ai);
	maxdata_ai = comedi_get_maxdata(it, subdev_ai, i);
	printf("Maxdata %i ", maxdata_ai);
	ranges_ai = comedi_get_n_ranges(it, subdev_ai, i);
	printf("Ranges %i ", ranges_ai);
	ad_range = comedi_get_range(it, subdev_ai, i, range_ai);
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
		da_range = comedi_get_range(it, subdev_ao, i, range_ao);
		printf(": da_range .min = %.3f, max = %.3f\r\n", da_range->min,
			da_range->max);
	}

	ADC_OPEN = true;
	comedi_set_global_oor_behavior(COMEDI_OOR_NUMBER);
	return 0;
}

int init_dac(double min_range, double max_range, int range_update)
{
	int i = 0;

	if (!DEV_OPEN) {
		it = comedi_open("/dev/comedi0");
		if (it == NULL) {
			comedi_perror("comedi_open");
			ADC_OPEN = false;
			DEV_OPEN = false;
			return -1;
		}
		DEV_OPEN = true;
	}

	subdev_ao = comedi_find_subdevice_by_type(it, COMEDI_SUBD_AO, subdev_ao);
	if (subdev_ao < 0) {
		HAS_AO = false;
	} else {
		HAS_AO = true;
	}

	if (HAS_AO) {
		printf("Subdev AO  %i ", subdev_ao);
		channels_ao = comedi_get_n_channels(it, subdev_ao);
		printf("Analog  Channels %i ", channels_ao);
		maxdata_ao = comedi_get_maxdata(it, subdev_ao, i);
		printf("Maxdata %i ", maxdata_ao);
		ranges_ao = comedi_get_n_ranges(it, subdev_ao, i);
		printf("Ranges %i ", ranges_ao);
		da_range = comedi_get_range(it, subdev_ao, i, range_ao);
		printf(": da_range .min = %.3f, max = %.3f\r\n", da_range->min,
			da_range->max);
	}

	comedi_set_global_oor_behavior(COMEDI_OOR_NUMBER);
	return 0;
}

int adc_range(double min_range, double max_range)
{
	if (ADC_OPEN) {
		ad_range->min = min_range;
		ad_range->max = max_range;
		return 0;
	} else {
		return -1;
	}
}

int dac_range(double min_range, double max_range)
{
	if (ADC_OPEN) {
		da_range->min = min_range;
		da_range->max = max_range;
		return 0;
	} else {
		return -1;
	}
}

int set_dac_volts(int chan, double voltage)
{
	lsampl_t data;
	int retval;

	data = comedi_from_phys(voltage, da_range, maxdata_ao);
	bmc.dac_sample[chan] = data;
	retval = comedi_data_write(it, subdev_ao, chan, range_ao, aref_ao, data);
	if (retval < 0) {
		comedi_perror("comedi_data_write in set_dac_volts");
		DAC_ERROR = true;
	}
	return retval;
}

int set_dac_raw(int chan, lsampl_t voltage)
{
	int retval;

	retval = comedi_data_write(it, subdev_ao, chan, range_ao, aref_ao, voltage);
	if (retval < 0) {
		comedi_perror("comedi_data_write in set_dac_raw");
		DAC_ERROR = true;
	}
	return retval;
}

double get_adc_volts(int chan)
{
	lsampl_t data[16];
	int retval;

	retval = comedi_data_read_n(it, subdev_ai, chan, range_ai, aref_ai, &data[0], 8);
	if (retval < 0) {
		comedi_perror("comedi_data_read in get_adc_volts");
		ADC_ERROR = true;
		return 0.0;
	}
	bmc.adc_sample[chan] = data[0];
	return comedi_to_phys(data[0], ad_range, maxdata_ai);
}

int set_dio_output(int chan)
{
	return comedi_dio_config(it,
		subdev_dio,
		chan,
		COMEDI_OUTPUT);
}

int set_dio_input(int chan)
{
	return comedi_dio_config(it,
		subdev_dio,
		chan,
		COMEDI_INPUT);
}

int get_dio_bit(int chan)
{
	lsampl_t data;
	int retval;

	retval = comedi_data_read(it, subdev_di, chan, range_di, aref_dio, &data);
	if (retval < 0) {
		comedi_perror("comedi_data_read in get_dio_bits");
		DIO_ERROR = true;
		return 0;
	}
	return data;
}

int put_dio_bit(int chan, int bit_data)
{
	lsampl_t data = bit_data;
	int retval;

	retval = comedi_data_write(it, subdev_do, chan, range_do, aref_dio, data);
	if (retval < 0) {
		comedi_perror("comedi_data_write in put_dio_bits");
		DIO_ERROR = true;
		return -1;
	}
	return 0;
}

int init_dio(void)
{
	int i = 0;

	if (!DEV_OPEN) {
		it = comedi_open("/dev/comedi0");
		if (it == NULL) {
			comedi_perror("comedi_open");
			DIO_OPEN = false;
			DEV_OPEN = false;
			return -1;
		}
		DEV_OPEN = true;
	}

	subdev_di = comedi_find_subdevice_by_type(it, COMEDI_SUBD_DI, subdev_di);
	if (subdev_di < 0) {
		return -1;
		DIO_OPEN = false;
	}
	subdev_do = comedi_find_subdevice_by_type(it, COMEDI_SUBD_DO, subdev_do);
	if (subdev_do < 0) {
		return -1;
		DIO_OPEN = false;
	}

	subdev_counter = comedi_find_subdevice_by_type(it, COMEDI_SUBD_COUNTER, subdev_counter);
	if (subdev_counter < 0) {
		return -1;
		PWM_OPEN = false;
	}

	printf("Subdev DI  %i ", subdev_di);
	channels_di = comedi_get_n_channels(it, subdev_di);
	printf("Digital Channels %i ", channels_di);
	maxdata_di = comedi_get_maxdata(it, subdev_di, i);
	printf("Maxdata %i ", maxdata_di);
	ranges_di = comedi_get_n_ranges(it, subdev_di, i);
	printf("Ranges %i \r\n", ranges_di);

	printf("Subdev DO  %i ", subdev_do);
	channels_do = comedi_get_n_channels(it, subdev_do);
	printf("Digital Channels %i ", channels_do);
	maxdata_do = comedi_get_maxdata(it, subdev_do, i);
	printf("Maxdata %i ", maxdata_do);
	ranges_do = comedi_get_n_ranges(it, subdev_do, i);
	printf("Ranges %i \r\n", ranges_do);

	printf("Subdev COU %i ", subdev_counter);
	channels_counter = comedi_get_n_channels(it, subdev_counter);
	printf("Digital Channels %i ", channels_counter);
	maxdata_counter = comedi_get_maxdata(it, subdev_counter, i);
	printf("Maxdata %i ", maxdata_counter);
	ranges_counter = comedi_get_n_ranges(it, subdev_counter, i);
	printf("Ranges %i \r\n", ranges_counter);
	DIO_OPEN = true;
	return 0;
}

int get_data_sample(void)
{
	unsigned int obits;
	//	bmc.pv_voltage = get_adc_volts(PVV_C);
	//	bmc.cc_voltage = get_adc_volts(CCV_C);

	bmc.datain.D0 = get_dio_bit(0);

	if (JUST_BITS) { // send I/O bit by bit
		put_dio_bit(0, bmc.dataout.d.D0);
		put_dio_bit(1, bmc.dataout.d.D1);
		put_dio_bit(2, bmc.dataout.d.D2);
		put_dio_bit(3, bmc.dataout.d.D3);
		put_dio_bit(4, bmc.dataout.d.D4);
		put_dio_bit(5, bmc.dataout.d.D5);
		put_dio_bit(6, bmc.dataout.d.D6);
		put_dio_bit(7, bmc.dataout.d.D7);
	} else { // send I/O as a byte mask
		obits = bmc.dataout.dio_buf;
		comedi_dio_bitfield2(it, subdev_do, 0xff, &obits, 0);
	}

	return 0;
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
