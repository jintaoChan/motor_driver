#ifndef __CIA402_H__
#define __CIA402_H__

#include <stdint.h>

/* Minimal CiA402 state handling used for object dictionary interoperability. */
void CIA402_Init(uint16_t *status_word, int8_t *mode_display);
void CIA402_Update(uint8_t al_state, uint16_t control_word, int8_t mode_request,
                   uint16_t *status_word, int8_t *mode_display);

#endif /* __CIA402_H__ */