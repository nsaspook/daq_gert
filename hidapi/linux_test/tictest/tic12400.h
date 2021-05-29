#ifndef TIC12400_H    /* Guard against multiple inclusion */
#define TIC12400_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define TIC12400_DRIVER "V0.5"

#define por_bit 0x01
/*
 * switch bit masks in the raw 32-bit register from the TIC12400
 */
#define raw_mask_0 0b010
#define raw_mask_11 0b100000000000000

/*
 * TIC12400 command structure
 */
typedef struct __attribute__((packed))
{
	uint32_t par : 1;
	uint32_t data : 24;
	uint32_t addr : 6;
	uint32_t wr : 1;
}
ticbuf_type;

/*
 * TIC12400 response structure
 */
typedef struct __attribute__((packed))
{
	uint32_t par : 1;
	uint32_t data : 24;
	uint32_t oi : 1;
	uint32_t temp : 1;
	uint32_t vs_th : 1;
	uint32_t ssc : 1;
	uint32_t parity_fail : 1;
	uint32_t spi_fail : 1;
	uint32_t por : 1;
}
ticread_type;

void tic12400_reset(void);
bool tic12400_init(void);
uint32_t tic12400_wr(const ticbuf_type *, uint16_t);
uint32_t tic12400_get_sw(void);
void tic12400_read_sw(uint32_t, uintptr_t);
bool tic12400_parity(uint32_t);

extern volatile uint32_t tic12400_status, tic12400_counts, tic12400_value_counts;
extern volatile uint32_t tic12400_value;
extern volatile bool tic12400_init_fail, tic12400_event;
extern ticread_type *ticstatus;
extern ticread_type *ticvalue;
extern volatile bool tic12400_parity_status;

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


	/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* TIC12400_H */

/* *****************************************************************************
 End of File
 */
