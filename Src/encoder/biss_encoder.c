#include "encoder/biss_encoder.h"

static const uint8_t BissFrameBits = 25U;
static const uint8_t BissDataBits = 19U;
static const uint8_t BissPosBits = 17U;
static const uint8_t BissCrcBits = 6U;
static const uint8_t BissAckBits = 13U;
static const uint8_t BissStartBits = 1U;
static const uint8_t BissCdsBits = 1U;
static const uint8_t BissSampleBits = 48U;
static const uint8_t BissTrackWindow = 2U;

#define BISS_SAMPLE_BYTES 6U

static int8_t g_lockedFrameStart = -1;
static uint8_t g_lockFailCount = 0U;
static BISS_RuntimeInfo_t g_runtimeInfo = {0xFFU, 0U, 0U};

volatile uint8_t g_biss_cfg_shift = 0xFFU;
volatile uint8_t g_biss_cfg_byte_swap = 0U;
volatile uint32_t g_biss_raw_be = 0U;

static uint8_t BISS_CRC6_Calc(uint32_t data19)
{
  uint8_t crc = 0U;

  for (uint8_t i = 0U; i < BissDataBits; i++)
  {
    uint8_t dataBit = (uint8_t)((data19 >> (BissDataBits - 1U - i)) & 0x01U);
    uint8_t fb = (uint8_t)(((crc >> 5U) & 0x01U) ^ dataBit);

    crc = (uint8_t)((crc << 1U) & 0x3FU);
    if (fb != 0U)
    {
      crc ^= 0x03U;
    }
  }

  return crc;
}

static uint8_t BISS_GetBit48(uint64_t raw48, uint8_t bitIndex)
{
  if (bitIndex >= BissSampleBits)
  {
    return 0U;
  }

  return (uint8_t)((raw48 >> (BissSampleBits - 1U - bitIndex)) & 0x01ULL);
}

static uint32_t BISS_ExtractBits48(uint64_t raw48, uint8_t startBit, uint8_t bitCount)
{
  uint8_t shift;
  uint64_t mask;

  if ((bitCount == 0U) || (bitCount > 32U) || (startBit + bitCount > BissSampleBits))
  {
    return 0U;
  }

  shift = (uint8_t)(BissSampleBits - (startBit + bitCount));
  mask = (1ULL << bitCount) - 1ULL;
  return (uint32_t)((raw48 >> shift) & mask);
}

static bool BISS_TryDecodeFrame25(uint32_t frame25, BISS_Frame_t *out)
{
  uint32_t data19;
  uint8_t crcRx;
  uint8_t crcCalc;
  uint8_t crcExpected;

  if (out == NULL)
  {
    return false;
  }

  data19 = frame25 >> BissCrcBits;
  crcRx = (uint8_t)(frame25 & 0x3FU);
  crcCalc = BISS_CRC6_Calc(data19);
  crcExpected = (uint8_t)((~crcCalc) & 0x3FU);

  if (crcRx != crcExpected)
  {
    return false;
  }

  out->position = (data19 >> 2U) & ((1UL << BissPosBits) - 1UL);
  out->error = (uint8_t)((data19 >> 1U) & 0x01U);
  out->warning = (uint8_t)(data19 & 0x01U);
  out->crc_rx = crcRx;
  out->crc_calc = crcCalc;
  out->crc_expected = crcExpected;
  out->crc_ok = 1U;

  return true;
}

bool BISS_ReadFrame(SPI_HandleTypeDef *hspi, BISS_Frame_t *out)
{
  uint8_t tx[BISS_SAMPLE_BYTES] = {0U};
  uint8_t rx[BISS_SAMPLE_BYTES] = {0U};
  uint64_t raw48 = 0ULL;
  uint8_t i;
  uint8_t frameStart;
  uint8_t ackStart;
  uint8_t startBit;
  const uint8_t overheadBits = (uint8_t)(BissAckBits + BissStartBits + BissCdsBits);
  const uint8_t maxAckStart = (uint8_t)(BissSampleBits - overheadBits - BissFrameBits);

  if ((hspi == NULL) || (out == NULL))
  {
    return false;
  }

  if (HAL_SPI_TransmitReceive(hspi, tx, rx, BISS_SAMPLE_BYTES, 20U) != HAL_OK)
  {
    return false;
  }

  for (i = 0U; i < BISS_SAMPLE_BYTES; i++)
  {
    raw48 = (raw48 << 8U) | (uint64_t)rx[i];
  }

  g_runtimeInfo.raw_msb32 = (uint32_t)(raw48 >> 16U);
  g_runtimeInfo.byte_swap = 0U;
  g_biss_raw_be = g_runtimeInfo.raw_msb32;
  g_biss_cfg_byte_swap = g_runtimeInfo.byte_swap;

  if (g_lockedFrameStart >= 0)
  {
    int16_t start = (int16_t)g_lockedFrameStart - (int16_t)BissTrackWindow;
    int16_t end = (int16_t)g_lockedFrameStart + (int16_t)BissTrackWindow;

    if (start < 0)
    {
      start = 0;
    }
    if (end > (int16_t)(BissSampleBits - BissFrameBits))
    {
      end = (int16_t)(BissSampleBits - BissFrameBits);
    }

    for (int16_t s = start; s <= end; s++)
    {
      uint32_t frame = BISS_ExtractBits48(raw48, (uint8_t)s, BissFrameBits);
      if (BISS_TryDecodeFrame25(frame, out))
      {
        g_lockedFrameStart = (int8_t)s;
        g_lockFailCount = 0U;
        g_runtimeInfo.frame_start_bit = (uint8_t)s;
        g_biss_cfg_shift = g_runtimeInfo.frame_start_bit;
        return true;
      }
    }

    g_lockFailCount++;
    if (g_lockFailCount >= 3U)
    {
      g_lockedFrameStart = -1;
      g_lockFailCount = 0U;
    }
  }

  for (ackStart = 0U; ackStart <= maxAckStart; ackStart++)
  {
    bool ackLow = true;
    for (i = 0U; i < BissAckBits; i++)
    {
      if (BISS_GetBit48(raw48, (uint8_t)(ackStart + i)) != 0U)
      {
        ackLow = false;
        break;
      }
    }

    if (!ackLow)
    {
      continue;
    }

    /* BiSS-C START bit should be high right after ACK low window. */
    startBit = BISS_GetBit48(raw48, (uint8_t)(ackStart + BissAckBits));
    if (startBit == 0U)
    {
      continue;
    }

    frameStart = (uint8_t)(ackStart + overheadBits);
    if ((uint16_t)frameStart + (uint16_t)BissFrameBits <= (uint16_t)BissSampleBits)
    {
      uint32_t frame = BISS_ExtractBits48(raw48, frameStart, BissFrameBits);
      if (BISS_TryDecodeFrame25(frame, out))
      {
        g_lockedFrameStart = (int8_t)frameStart;
        g_runtimeInfo.frame_start_bit = frameStart;
        g_biss_cfg_shift = g_runtimeInfo.frame_start_bit;
        return true;
      }
    }
  }

  g_runtimeInfo.frame_start_bit = 0xFFU;
  g_biss_cfg_shift = g_runtimeInfo.frame_start_bit;
  return false;
}

void BISS_GetRuntimeInfo(BISS_RuntimeInfo_t *out)
{
  if (out != NULL)
  {
    *out = g_runtimeInfo;
  }
}
