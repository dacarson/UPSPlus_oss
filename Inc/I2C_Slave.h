#ifndef __I2C_SLAVE_H
#define __I2C_SLAVE_H

#include <stdint.h>
#include "ups_state.h"

/* Double-buffered register image; ISR latches active_reg_image on ADDR+READ, TX from reg_image[latched] */
extern uint8_t reg_image[2][256];
extern volatile uint8_t active_reg_image;

#ifdef __cplusplus
extern "C" {
#endif

/* Pending write - ISR stores write data here; main loop applies to state */
extern i2c_pending_write_t i2c_pending_write;

void MX_I2C1_Slave_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SLAVE_H */
