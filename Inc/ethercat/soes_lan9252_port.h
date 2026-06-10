#ifndef SOES_LAN9252_PORT_H
#define SOES_LAN9252_PORT_H

#include "esc_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

const lan9252_stm32_hw_if_t *SOES_LAN9252_GetHwIf(void);

#ifdef __cplusplus
}
#endif

#endif /* SOES_LAN9252_PORT_H */
