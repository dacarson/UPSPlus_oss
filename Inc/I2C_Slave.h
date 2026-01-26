#ifndef __I2C_SLAVE_H
#define __I2C_SLAVE_H

#include <stdint.h>

/* I2C slave receive buffer - 256 bytes to support indices 0-255 */
extern volatile uint8_t aReceiveBuffer[256];

#ifdef __cplusplus
extern "C" {
#endif

void MX_I2C1_Slave_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_SLAVE_H */
