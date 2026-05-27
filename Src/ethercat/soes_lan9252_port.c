#include "ethercat/soes_lan9252_port.h"

#include "main.h"

extern SPI_HandleTypeDef hspi1;

static uint32_t soes_lock_nesting;
static uint32_t soes_lock_primask;

static void soes_set_cs(bool selected)
{
    HAL_GPIO_WritePin(
        ECAT_CS_GPIO_Port,
        ECAT_CS_Pin,
        selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static int soes_spi_txrx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((tx == NULL) || (len == 0U))
    {
        return -1;
    }

    if (rx != NULL)
    {
        status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, HAL_MAX_DELAY);
    }
    else
    {
        status = HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, HAL_MAX_DELAY);
    }

    return (status == HAL_OK) ? 0 : -1;
}

static void soes_delay_us(uint32_t us)
{
    uint32_t cycles;
    uint32_t start;

    if (us == 0U)
    {
        return;
    }

    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    start = DWT->CYCCNT;
    cycles = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}

static void soes_lock(void)
{
    if (soes_lock_nesting == 0U)
    {
        soes_lock_primask = __get_PRIMASK();
        __disable_irq();
    }

    soes_lock_nesting++;
}

static void soes_unlock(void)
{
    if (soes_lock_nesting == 0U)
    {
        return;
    }

    soes_lock_nesting--;

    if ((soes_lock_nesting == 0U) && (soes_lock_primask == 0U))
    {
        __enable_irq();
    }
}

static const lan9252_stm32_hw_if_t soes_hw_if = {
    .set_cs = soes_set_cs,
    .spi_txrx = soes_spi_txrx,
    .delay_us = soes_delay_us,
    .lock = soes_lock,
    .unlock = soes_unlock,
};

const lan9252_stm32_hw_if_t *SOES_LAN9252_GetHwIf(void)
{
    return &soes_hw_if;
}
