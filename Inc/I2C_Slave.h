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
uint8_t I2C1_IsAddrFlagSet(void);
uint16_t I2C1_GetLastAddrUs(void);
uint16_t I2C1_GetLastStopUs(void);
uint8_t I2C1_ReadIna219Shunt(uint8_t is_output, int16_t *shunt_raw);

/* Scheduler-tick-resolution (10 ms, 32-bit, wraparound-safe) mirrors of the last ADDR/STOP
 * events matched to the STM32's own slave address. Pushed in from main.c's tick ISR
 * (I2C1_SetTickCounter) since sched_flags is private to main.c; captured into these at the
 * same moments as the existing microsecond (TIM3) timestamps above. Unlike those, these have
 * no ~65.5 ms wraparound ceiling, so callers can test "how long ago" over multi-second ranges. */
void I2C1_SetTickCounter(uint32_t tick_counter);
uint32_t I2C1_GetLastAddrTick(void);
uint32_t I2C1_GetLastStopTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SLAVE_H */
