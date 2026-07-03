#ifndef LAN9252_APP_H
#define LAN9252_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LAN9252_SPI_Init(void);
static uint16_t LAN9252_ESC_ReadReg16(uint16_t addr);
static void LAN9252_ESC_WriteReg16(uint16_t addr, uint16_t value);

void cb_get_inputs(void);
void cb_set_outputs(void);

#ifdef __cplusplus
}
#endif

#endif /* LAN9252_APP_H */
