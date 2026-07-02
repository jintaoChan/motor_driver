#ifndef LAN9252_APP_H
#define LAN9252_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LAN9252_SPI_Init(void);

uint16_t LAN9252_GPIO_ReadInputs(void);
uint16_t LAN9252_GPIO_ReadDirection(void);
uint16_t LAN9252_GPIO_ReadOutputs(void);
void LAN9252_GPIO_WriteDirection(uint16_t value);
void LAN9252_GPIO_WriteOutputs(uint16_t value);

void cb_get_inputs(void);
void cb_set_outputs(void);

#ifdef __cplusplus
}
#endif

#endif /* LAN9252_APP_H */
