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
void MX_I2C1_ProbeMasterSetup(void);
void I2C1_RunIna219Probe(void);
uint8_t I2C1_GetInaProbeOutputPresent(void);
uint8_t I2C1_GetInaProbeBatteryPresent(void);
uint8_t I2C1_GetSlaveTxnActive(void);
uint16_t I2C1_GetLastAddrUs(void);
uint16_t I2C1_GetLastStopUs(void);
uint8_t I2C1_ReadIna219Shunt(uint8_t is_output, int16_t *shunt_raw);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SLAVE_H */
