#ifndef BISS_ENCODER_H
#define BISS_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

typedef struct
{
  uint32_t position;
  uint8_t error;
  uint8_t warning;
  uint8_t crc_rx;
  uint8_t crc_calc;
  uint8_t crc_expected;
  uint8_t crc_ok;
} BISS_Frame_t;

typedef struct
{
  uint8_t frame_start_bit;
  uint8_t byte_swap;
  uint32_t raw_msb32;
} BISS_RuntimeInfo_t;

extern volatile uint8_t g_biss_cfg_shift;
extern volatile uint8_t g_biss_cfg_byte_swap;
extern volatile uint32_t g_biss_raw_be;

bool BISS_ReadFrame(SPI_HandleTypeDef *hspi, BISS_Frame_t *out);
void BISS_GetRuntimeInfo(BISS_RuntimeInfo_t *out);

#endif