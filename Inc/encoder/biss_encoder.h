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

/* DMA-based non-blocking read API -------------------------------------------
 *
 * Usage in main loop:
 *   if (g_biss_dma_state != BISS_DMA_BUSY) BISS_StartDmaRead(&hspi1);
 *   BISS_Frame_t f;
 *   if (BISS_PollDmaResult(&f)) { ... use f ... }
 *
 * HAL callbacks (put in main.c or stm32g4xx_it.c user-code section):
 *   void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h) { BISS_DmaCpltCallback(h); }
 *   void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h)    { BISS_DmaErrorCallback(h); }
 */
typedef enum
{
  BISS_DMA_IDLE  = 0,
  BISS_DMA_BUSY  = 1,
  BISS_DMA_DONE  = 2,
  BISS_DMA_ERROR = 3,
} BISS_DmaState_t;

extern volatile BISS_DmaState_t g_biss_dma_state;

bool BISS_StartDmaRead(SPI_HandleTypeDef *hspi);
bool BISS_PollDmaResult(BISS_Frame_t *out);
void BISS_DmaCpltCallback(SPI_HandleTypeDef *hspi);
void BISS_DmaErrorCallback(SPI_HandleTypeDef *hspi);

/* Decoded position outputs ------------------------------------------------
 * Written by the DMA completion ISR (BISS_DmaCpltCallback).
 * Read directly by FOC or the main loop - no polling needed.              */
extern volatile uint32_t g_biss_position;
extern volatile uint8_t  g_biss_error;
extern volatile uint8_t  g_biss_warning;
extern volatile uint8_t  g_biss_crc_ok;

#endif