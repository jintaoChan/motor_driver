#include "encoder/biss_encoder.h"
#include "main.h"

static const uint8_t BissFrameBits = 25U;
static const uint8_t BissDataBits = 19U;
static const uint8_t BissPosBits = 17U;
static const uint8_t BissCrcBits = 6U;
static const uint8_t BissAckBits = 13U;
static const uint8_t BissStartBits = 1U;
static const uint8_t BissCdsBits = 1U;
static const uint8_t BissSampleBits = 48U;
static const uint8_t BissTrackWindow = 2U;

#define BISS_CLK_HZ 1000000UL

static int8_t g_lockedFrameStart = -1;
static uint8_t g_lockFailCount = 0U;
static BISS_RuntimeInfo_t g_runtimeInfo = {0xFFU, 0U, 0U};

volatile uint8_t g_biss_cfg_shift = 0xFFU;
volatile uint8_t g_biss_cfg_byte_swap = 0U;
volatile uint32_t g_biss_raw_be = 0U;

static void BISS_DelayCycles(uint32_t cycles)
{
  uint32_t start = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - start) < cycles)
  {
    /* busy wait */
  }
}

static uint32_t BISS_HalfPeriodCycles(void)
{
  static uint8_t dwtReady = 0U;

  if (dwtReady == 0U)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwtReady = 1U;
  }

  {
    uint32_t halfCycles = SystemCoreClock / (2UL * BISS_CLK_HZ);
    if (halfCycles == 0U)
    {
      halfCycles = 1U;
    }
    return halfCycles;
  }
}

static uint32_t BISS_SampleOffsetCycles(uint32_t halfCycles)
{
  /* Sample near the middle of CLK high to maximize setup/hold margin. */
  uint32_t offset = halfCycles / 2UL;
  if (offset == 0U)
  {
    offset = 1U;
  }
  return offset;
}

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
  uint64_t raw48 = 0ULL;
  uint8_t i;
  uint8_t frameStart;
  uint8_t ackStart;
  uint8_t startBit;
  const uint8_t overheadBits = (uint8_t)(BissAckBits + BissStartBits + BissCdsBits);
  const uint8_t maxAckStart = (uint8_t)(BissSampleBits - overheadBits - BissFrameBits);
  GPIO_TypeDef *clkPort = ENCODER_CLK_GPIO_Port;
  GPIO_TypeDef *dataPort = ENCODER_DATA_GPIO_Port;
  uint32_t clkMask = (uint32_t)ENCODER_CLK_Pin;
  uint32_t dataMask = (uint32_t)ENCODER_DATA_Pin;
  uint32_t halfCycles;
  uint32_t sampleOffset;
  uint32_t remainHigh;

  if (out == NULL)
  {
    return false;
  }

  (void)hspi;
  halfCycles = BISS_HalfPeriodCycles();
  sampleOffset = BISS_SampleOffsetCycles(halfCycles);
  remainHigh = halfCycles - sampleOffset;

  clkPort->BSRR = (clkMask << 16U);
  BISS_DelayCycles(halfCycles);

  for (i = 0U; i < BissSampleBits; i++)
  {
    clkPort->BSRR = clkMask;
    BISS_DelayCycles(sampleOffset);
    raw48 = (raw48 << 1U) | ((dataPort->IDR & dataMask) != 0U ? 1ULL : 0ULL);
    BISS_DelayCycles(remainHigh);
    clkPort->BSRR = (clkMask << 16U);
    BISS_DelayCycles(halfCycles);
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
