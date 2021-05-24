/*
 * TC12400 driver for PIC32MK v0.1
 * uses SPI5 mode1 at 4MHz no interrupts
 * external interrupt 2 is used to detect chip switch events
 */

#include "tic12400.h"
#include "tictest.h"

/*
 * command structure data
 * the parity bit must be set correctly for the command to execute on the chip
 */
const ticbuf_type setup32 = {
	.wr = 1,
	.addr = 0x32,
	.data = 0,
	.par = 1,
};
const ticbuf_type setup21 = {
	.wr = 1,
	.addr = 0x21,
	.data = 0,
	.par = 0,
};
const ticbuf_type setup1c = {
	.wr = 1,
	.addr = 0x1c,
	.data = 0,
	.par = 1,
};
const ticbuf_type setup1b = {
	.wr = 1,
	.addr = 0x1b,
	.data = 0xffffff,
	.par = 0,
};
const ticbuf_type setup1a = {
	.wr = 1,
	.addr = 0x1a,
	.data = 0xc000,
	.par = 1,
};
const ticbuf_type setup1a_trigger = {
	.wr = 1,
	.addr = 0x1a,
	.data = 0x0a00, // trigger and do config register CRC 
	.par = 1,
};
const ticbuf_type setup22 = {
	.wr = 1,
	.addr = 0x22,
	.data = 0xffffff,
	.par = 0,
};
const ticbuf_type setup23 = {
	.wr = 1,
	.addr = 0x23,
	.data = 0xffffff,
	.par = 1,
};
const ticbuf_type setup24 = {
	.wr = 1,
	.addr = 0x24,
	.data = 0xfff,
	.par = 0,
};
const ticbuf_type setup1d = {
	.wr = 1,
	.addr = 0x1d,
	.data = 011111111, // octal constant, all inputs 1mA wetting current
	.par = 0,
};
const ticbuf_type ticread05 = {
	.wr = 0,
	.addr = 0x05,
	.data = 0,
	.par = 1,
};
const ticbuf_type ticdevid01 = {
	.wr = 0,
	.addr = 0x01,
	.data = 0,
	.par = 0,
};
const ticbuf_type ticstat02 = {
	.wr = 0,
	.addr = 0x02,
	.data = 0,
	.par = 0,
};
const ticbuf_type ticreset1a = {
	.wr = 1,
	.addr = 0x1a,
	.data = 0x1,
	.par = 0,
};

/*
 * global status and value registers
 */
volatile uint32_t tic12400_status = 0, tic12400_counts = 0, tic12400_value_counts = 0;
volatile uint32_t tic12400_value = 0;
ticread_type *ticstatus = (ticread_type*) & tic12400_status;
ticread_type *ticvalue = (ticread_type*) & tic12400_value;
volatile bool tic12400_init_fail = false, tic12400_event = false; // chip error detection flag
volatile bool tic12400_parity_status = false;

/*
 * software reset of the chip using SPI
 * all registers set to their default values
 */
void tic12400_reset(void)
{
	tic12400_wr(&ticreset1a, 2);
}

/*
 * chip detection and configuration for all inputs with interrupts for
 * switch state changes with debounce
 * returns false on configuration failure
 */
bool tic12400_init(void)
{
	tic12400_status = tic12400_wr(&ticstat02, 0); // get status to check for proper operation

	if ((ticstatus->data > por_bit) || !ticstatus->por) { // check for any high bits beyond POR bits set
		tic12400_init_fail = true;
		goto fail;
	}

	tic12400_wr(&setup32, 0); //all set to compare mode, 0x32
	tic12400_wr(&setup21, 0); //Compare threshold all bits 2V, 0x21
	tic12400_wr(&setup1c, 0); //all set to GND switch type, 0x1c
	tic12400_wr(&setup1b, 0); //all channels are enabled, 0x1b
	tic12400_wr(&setup22, 0); //set switch interrupts, 0x22
	tic12400_wr(&setup23, 0); //set switch interrupts, 0x23
	tic12400_wr(&setup24, 0); // enable interrupts, 0x24
	tic12400_wr(&setup1d, 0); // set wetting currents, 0x1d
	tic12400_wr(&setup1a, 0); // set switch debounce to max 4 counts, 0x1a
	tic12400_status = tic12400_wr(&setup1a_trigger, 2); // trigger switch detections & CRC, 0x1a

	if (ticstatus->spi_fail) {
		tic12400_init_fail = true;
		goto fail;
	}

	tic12400_status = tic12400_wr(&ticdevid01, 0); // get device id, 0x01
	/*
	 * configure event handler for tic12400 interrupts
	 */
	//	EVIC_ExternalInterruptCallbackRegister(EXTERNAL_INT_2, tic12400_interrupt, 0);
	//	EVIC_ExternalInterruptEnable(EXTERNAL_INT_2);

fail:
	return !tic12400_init_fail; // flip to return true if NO configuration failures
}

/*
 * send tic12400 commands to SPI port 5 with possible delay after transfer
 * returns 32-bit spi response from the tic12400
 */
uint32_t tic12400_wr(const ticbuf_type * buffer, uint16_t del)
{
	static uint32_t rbuffer = 0;

	SPI5_WriteRead((void*) buffer, 4, (void*) &rbuffer, 4);

	if (ticvalue->parity_fail) { // check for command parity errors
		tic12400_parity_status = true;
	};

	if (del) {
		sleep_us(del * 1000);
	}

	return rbuffer;
}

/*
 * switch data reading testing routine
 * tic12400 value is updated in external interrupt #2 ISR
 */
uint32_t tic12400_get_sw(void)
{
	if (tic12400_init_fail) { // Trouble in River City
		return 0;
	}

	if (tic12400_value & (raw_mask_0)) {
		//		BSP_LED1_Clear();
	} else {
		//		BSP_LED1_Set();
	}

	if (tic12400_value & (raw_mask_11)) {
		//		BSP_LED2_Clear();
	} else {
		//		BSP_LED2_Set();
	}

	tic12400_event = false;
	return tic12400_value;
}

/*
 * 32-bit 1's parity check
 * https://graphics.stanford.edu/~seander/bithacks.html#ParityNaive
 */
bool tic12400_parity(uint32_t v)
{
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	return(0x6996 >> v) & 1;
}

/*
 * external interrupt 2 ISR
 * switch SPI status and switch data updates
 * toggles debug led and clears interrupt by reading status
 * sets event flag for user application notification
 */
void tic12400_interrupt(uint32_t a, uintptr_t b)
{
	tic12400_value = tic12400_wr(&ticread05, 0); // read switch
	tic12400_status = tic12400_wr(&ticstat02, 0); // read status
	//	RESET_LED_Toggle();

	if (ticvalue->ssc && tic12400_parity(tic12400_value)) { // only trigger on switch state change
		//		BSP_LED3_Toggle();
		tic12400_event = true;
		tic12400_value_counts++;
	}
	tic12400_counts++;
}
