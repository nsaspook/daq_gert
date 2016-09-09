/* 
 * File:   finger.h
 * Author: root
 *
 * Created on September 7, 2016, 9:14 AM
 */

#ifndef FINGER_H
#define	FINGER_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <GenericTypeDefs.h>

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
	/*unsigned types*/
	typedef unsigned char uint8_t;
	typedef unsigned int uint16_t;
	typedef unsigned long uint32_t;
	typedef unsigned long long uint64_t;
	/*signed types*/
	typedef signed char int8_t;
	typedef signed int int16_t;
	typedef signed long int32_t;
	typedef signed long long int64_t;
#endif

#define abs(x) ((x) > 0 ? (x) : -(x))

#define	TIMEROFFSET	26474           // timer0 16bit counter value for 1 second to overflow

#define	TIMERCHARGE_BASE_X10		65400		// 5.5 uA time, large plate ~150us
#define	TIMERCHARGE_BASE_1		64000		// .55 uA time, large plate max sens ~700us
#define	TIMERCHARGE_BASE_2		61543		// .55 uA time, large plate low sens ~1000us
#define	TIMERCHARGE_BASE_3		65000		// .55 uA time, small plate max sens ~200us
#define	TIMERCHARGE_BASE_4		62543		// .55 uA time, small plate low sens ~750us
#define	TIMERDISCHARGE			60000		// discharge and max touch data update period 1.8ms
#define TIMERPROCESS			40000

#define	PDELAY	25200 // ~50hz for tick timer
#define TICK_SEC	50ul
#define TICK_10TH_SIC	TICK_SEC/10ul

#define ADC_READS	8
#define ZERO_NOISE	15
#define SCAN_MAX_CHAN	1
#define TRIP 25l //Difference between pressed
	//and un-pressed switch
#define TRIP_DIFF	20l
#define HYST 8l //amount to change
	//from pressed to un-pressed
#define PRESSED 1
#define UNPRESSED 0
#define	CHOP_BITS	1

	/* LCD defines */

#define MESG_W          80			// message string buffer

	/* DIO defines */
#define LOW		(uint8_t)0		// digital output state levels, sink
#define	HIGH		(uint8_t)1		// digital output state levels, source
#define	ON		LOW       		//
#define OFF		HIGH			//
#define	S_ON            LOW       		// low select/on for chip/led
#define S_OFF           HIGH			// high deselect/off chip/led
#define	R_ON            HIGH       		// control relay states
#define R_OFF           LOW			// control relay states
#define R_ALL_OFF       0x00
#define R_ALL_ON	0xff
#define NO		LOW
#define YES		HIGH

#define DLED0		LATAbits.LATA7
#define DLED1		LATCbits.LATC0

	/* bit set is LED off */
#define ROLL_PATTERN0	0b10111001110110111000110011011011
#define ROLL_PATTERN1	0b00000000000000000000000000000001
#define FLED0		LATBbits.LATB0
#define FLED1		LATBbits.LATB1

	//	CTMU section
	uint16_t touch_base_calc(uint8_t);
	void touch_channel(uint8_t);
	int16_t ctmu_touch(uint8_t, uint8_t);
	int16_t ctmu_setup(uint8_t, uint8_t);
	void ctmu_zero_set(void);

	struct spi_stat_type {
		volatile uint32_t adc_count, adc_error_count,
		int_count, last_int_count,
		time_tick;
		volatile uint8_t comm_ok, data_in;
	};

	struct finger_move_type {
		int16_t zero_ref, zero_max, zero_min, moving_avg, moving_val;
		int32_t avg_val;
		int16_t moving_diff;
		uint8_t zero_noise : 1;
	};

#ifdef	__cplusplus
}
#endif

#endif	/* FINGER_H */

