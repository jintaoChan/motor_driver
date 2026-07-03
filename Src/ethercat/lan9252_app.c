#include "ethercat/lan9252_app.h"

#include "main.h"
#include "esc.h"
#include "ethercat/cia402/cia402.h"
#include "ethercat/utypes.h"

/* LAN9252 SPI command */
#define LAN9252_SPI_READ_CMD    0x03U
#define LAN9252_SPI_WRITE_CMD   0x02U

/* LAN9252 register addresses */
#define LAN9252_REG_ID_REV      0x0050U   /* Chip ID[31:16] should be 0x9252 */
#define LAN9252_REG_GPIO_OUT    0x0F10U
#define LAN9252_REG_GPIO_IN     0x0F18U

extern SPI_HandleTypeDef hspi3;
extern volatile uint32_t g_biss_position;

static void LAN9252_SPI_WriteReg(uint16_t addr, uint32_t value)
{
    uint8_t tx[7] = {
        LAN9252_SPI_WRITE_CMD,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF),
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF),
    };

    HAL_GPIO_WritePin(ECAT_CS_GPIO_Port, ECAT_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx, sizeof(tx), 2000);
    HAL_GPIO_WritePin(ECAT_CS_GPIO_Port, ECAT_CS_Pin, GPIO_PIN_SET);
}

static uint32_t LAN9252_SPI_ReadReg(uint16_t addr)
{
    uint8_t tx[3] = {
        LAN9252_SPI_READ_CMD,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF),
    };
    uint8_t rx[4] = {0};

    HAL_GPIO_WritePin(ECAT_CS_GPIO_Port, ECAT_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, tx, 3, 20);
    HAL_SPI_Receive(&hspi3, rx, 4, 20);
    HAL_GPIO_WritePin(ECAT_CS_GPIO_Port, ECAT_CS_Pin, GPIO_PIN_SET);

    /* LAN9252 returns data in little-endian order */
    return ((uint32_t)rx[3] << 24) | ((uint32_t)rx[2] << 16)
        | ((uint32_t)rx[1] << 8) | (uint32_t)rx[0];
}

static uint16_t LAN9252_SPI_ReadReg16(uint16_t addr)
{
    return (uint16_t)(LAN9252_SPI_ReadReg(addr) & 0xFFFFU);
}

static void LAN9252_SPI_WriteReg16(uint16_t addr, uint16_t value)
{
    uint32_t reg = LAN9252_SPI_ReadReg(addr);
    reg = (reg & 0xFFFF0000U) | (uint32_t)value;
    LAN9252_SPI_WriteReg(addr, reg);
}

static uint16_t LAN9252_ESC_ReadReg16(uint16_t addr)
{
  uint16_t value = 0U;
  ESC_read(addr, &value, sizeof(value));
  return value;
}

static void LAN9252_ESC_WriteReg16(uint16_t addr, uint16_t value)
{
  ESC_write(addr, &value, sizeof(value));
}

void LAN9252_SPI_Init(void)
{
    uint32_t id = 0;

    HAL_GPIO_WritePin(ECAT_CS_GPIO_Port, ECAT_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(ECAT_RST_GPIO_Port, ECAT_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(ECAT_RST_GPIO_Port, ECAT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    do
    {
        id = LAN9252_SPI_ReadReg(LAN9252_REG_ID_REV);
    } while ((id >> 16) != 0x9252U);
}

void cb_get_inputs(void)
{
    Obj.Parameters.Lan9252Gpi = LAN9252_ESC_ReadReg16(LAN9252_REG_GPIO_IN);
    Obj.Parameters.PositionActualValue = (int32_t)g_biss_position;
}

void cb_set_outputs(void)
{
    CIA402_Update(
        (uint8_t)(ESCvar.ALstatus & ESCREG_AL_STATEMASK),
        Obj.Parameters.ControlWord,
        Obj.Parameters.OperationMode,
        &Obj.Parameters.StatusWord,
        &Obj.Parameters.OperationModeDisplay);

    LAN9252_ESC_WriteReg16(LAN9252_REG_GPIO_OUT, Obj.Parameters.Lan9252Gpo);

    /* HAL_GPIO_WritePin(
        LD2_GPIO_Port,
        LD2_Pin,
        Obj.Parameters.McuLed ? GPIO_PIN_SET : GPIO_PIN_RESET); */
}
