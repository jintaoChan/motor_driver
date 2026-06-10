/**
******************************************************************************
* @file stdrive102p_driver.h
* @author Motor Control SDK Team, ST Microelectronics
* @brief Header file for the configuration and management of the
*		 STDRIVE102P gate driver.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
* www.st.com/SLA0044
*
******************************************************************************
* @ingroup stdrive102pDriver
*/
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef STDRIVE102P_DRIVER_H
#define STDRIVE102P_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* Includes ------------------------------------------------------------------*/
#include <mc_stm_types.h>

#include <stdint.h>
#include <stdbool.h>

#if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 
	#define STDRIVE102P_spi_TX_ready LL_SPI_IsActiveFlag_TXP
	#define STDRIVE102P_spi_RX_ready LL_SPI_IsActiveFlag_RXP
#else
	#define STDRIVE102P_spi_TX_ready LL_SPI_IsActiveFlag_TXE
	#define STDRIVE102P_spi_RX_ready LL_SPI_IsActiveFlag_RXNE
#endif 

/* constant delays for SPI communication -----------------------------*/
#define STDRIVE102P_SPI_TIMEOUT 20 // milliseconds
#define STDRIVE102P_SPI_FRAME_DELAY 2 // microseconds (coarse)
#define STDRIVE102P_SPI_CS_DELAY 1 // microseconds (coarse)

/* status bits definition and diagnotic management.
Use these #define to decode the meaning of each bit, when reading:
STATUS1, STATUS2 using function STDRIVE102P_GetStatus2()
STATUS1, STATUS3 using function STDRIVE102P_GetStatus3()
STATUS1, AFE_STATUS using function  STDRIVE102P_GetAFEStatus() 
----------------------------------------------------------------------*/

// STATUS 1 - Reg. Address 0x00
#define STDRIVE102P_STATUS1_SPIERROR            (uint8_t)(1)
#define STDRIVE102P_STATUS1_COMPEV_L            (uint8_t)(1 << 1)
#define STDRIVE102P_STATUS1_THDS_L              (uint8_t)(1 << 2)
#define STDRIVE102P_STATUS1_VDDUVLO_L           (uint8_t)(1 << 3)
#define STDRIVE102P_STATUS1_VCCUVLO_L           (uint8_t)(1 << 4)
#define STDRIVE102P_STATUS1_VCPUVLO_L           (uint8_t)(1 << 5)
#define STDRIVE102P_STATUS1_VDSFAULT_L          (uint8_t)(1 << 6)
#define STDRIVE102P_STATUS1_LOCKED              (uint8_t)(1 << 7)

// STATUS 2 - Reg. Address 0x01
#define STDRIVE102P_STATUS2_COMP1EV_L           (uint8_t)(1 << 2)
#define STDRIVE102P_STATUS2_THGOOD              (uint8_t)(1 << 3)
#define STDRIVE102P_STATUS2_VCCGOOD             (uint8_t)(1 << 4)
#define STDRIVE102P_STATUS2_VCCUVLO             (uint8_t)(1 << 5)
#define STDRIVE102P_STATUS2_VCPUVLO             (uint8_t)(1 << 6)
#define STDRIVE102P_STATUS2_RESET               (uint8_t)(1 << 7)

// STATUS 3 - Reg. Address 0x02
#define STDRIVE102P_STATUS3_VDSLS3              (uint8_t)(1)
#define STDRIVE102P_STATUS3_VDSLS2              (uint8_t)(1 << 1)
#define STDRIVE102P_STATUS3_VDSLS1              (uint8_t)(1 << 2)
#define STDRIVE102P_STATUS3_VDSHS3              (uint8_t)(1 << 3)
#define STDRIVE102P_STATUS3_VDSHS2              (uint8_t)(1 << 4)
#define STDRIVE102P_STATUS3_VDSHS1              (uint8_t)(1 << 5)
#define STDRIVE102P_STATUS3_VDDUVLO             (uint8_t)(1 << 6)
#define STDRIVE102P_STATUS3_THSD                (uint8_t)(1 << 7)

// AFE_STATUS - Reg. Address 0x03
#define STDRIVE102P_AFESTATUS_COMP1_FLT         (uint8_t)(1 << 2)
#define STDRIVE102P_AFESTATUS_COMP1_EV          (uint8_t)(1 << 7)


/* nFAULT_SIG_EN - Reg. Address 0x08
Use these #define with the functions STDRIVE102P_SetFAULTpinSignal()
to select which are the STDRIVE102 events to be mapped on the nFAULT pin.
Use these #define with the functions STDRIVE102P_GetFAULTpinSignal()
to detect which are the STDRIVE102 events currently mapped on the nFAULT pin.*/

#define STDRIVE102P_FAULT_SIG_EN_COMP           (uint8_t)(1)
#define STDRIVE102P_FAULT_SIG_EN_VDS            (uint8_t)(1 << 3)
#define STDRIVE102P_FAULT_SIG_EN_THSD           (uint8_t)(1 << 4)
#define STDRIVE102P_FAULT_SIG_EN_VDD_UVLO       (uint8_t)(1 << 5)
#define STDRIVE102P_FAULT_SIG_EN_CP_UVLO        (uint8_t)(1 << 6)
#define STDRIVE102P_FAULT_SIG_EN_VCC_UVLO       (uint8_t)(1 << 7)
#define STDRIVE102P_FAULT_SIG_EN_ALL            (uint8_t)(0xFF)

/* Commands codes - Reg. 0x1A - Clear commands ------------------*/
#define STDRIVE102P_CMD_CLEAR_STATUS_L          (uint8_t)(0x90)
#define STDRIVE102P_CMD_CLEAR_HW_FAULTS	        (uint8_t)(0x05)
#define STDRIVE102P_CMD_CLEAR_ALL               (STDRIVE102P_CMD_CLEAR_STATUS_L | STDRIVE102P_CMD_CLEAR_HW_FAULTS)
#define STDRIVE102P_CMD_CLEAR_RESET             (uint8_t)(0x68)

/* Commands codes - Reg. 0x1B - Reset commands ------------------*/
#define STDRIVE102P_CMD_GLOBAL_RESET            (uint8_t)(0xA7)

/* Commands codes - Reg. 0x1C - Lock/Unlock commands ------------*/
#define STDRIVE102P_CMD_LOCK_REGS               (uint8_t)(0x71)
#define STDRIVE102P_CMD_UNLOCK_REGS             (uint8_t)(0x4B)


/** @addtogroup MCSDK
* @{
*/
/** @addtogroup stdrive102pDriver
* @{
*/
/** @defgroup stdrive102pErrCode STDRIVE102P driver Error Codes
  * @brief Error codes returned by the functions in the STDRIVE102P firmware library.
  * In case of no errors, the returned error code is 0.
  * @{
*/
#define STDRIVE102P_SYS_ERROR           (uint16_t)(0x8000)    /**< @brief Unexpected system error in the firmware library. */
#define STDRIVE102P_SPI_TIMEOUT_ERROR   (uint16_t)(0x4000)    /**< @brief A timeout has occurred int he SPI communication. */
#define STDRIVE102P_CONFIG_DATA_ERROR   (uint16_t)(0x2000)    /**< @brief A wrong data format was passed to the FW library fucntions. */
#define STDRIVE102P_USER2_ERROR         (uint16_t)(0x1000)    /**< @brief Error code not assigned, at user disposal. */
#define STDRIVE102P_USER1_ERROR         (uint16_t)(0x0800)    /**< @brief Error code not assigned, at user disposal. */
#define STDRIVE102P_VDS_ERROR           (uint16_t)(0x0040)    /**< @brief The VDS monitoring of the STDRIVE102 has been triggered. */
#define STDRIVE102P_CHG_PUMP_ERROR      (uint16_t)(0x0020)    /**< @brief The Charge pump UVLO of the STDRIVE102 has been triggered. */
#define STDRIVE102P_VCC_UVLO_ERROR      (uint16_t)(0x0010)    /**< @brief The VCC UVLO of the STDRIVE102 has been triggered. */
#define STDRIVE102P_VDD_UVLO_ERROR      (uint16_t)(0x0008)    /**< @brief The VDD UVLO of the STDRIVE102 has been triggered. */
#define STDRIVE102P_THERMAL_SD_ERROR    (uint16_t)(0x0004)    /**< @brief The thermal protection of the STDRIVE102 has been triggered. */
#define STDRIVE102P_COMPARATOR_ERROR    (uint16_t)(0x0002)    /**< @brief The comparator of the STDRIVE102 has been triggered. */
#define STDRIVE102P_SPI_CMD_ERROR       (uint16_t)(0x0001)    /**< @brief An SPI error protocol has been detected by the STDRIVE102. */

  /**
  * @}
  */
/**
* @}
*/
/**
* @}
*/

/** @addtogroup MCSDK
* @{
*/
/** @addtogroup stdrive102pDriver
* @{
*/
/**
* @brief stdrv102p_Timings_t Enumerator for the TCC: gate driver timing  selection.
*/
typedef enum
{
  P_L0_280_140ns = 0,        /**< @brief TCC timing level 0. */
  P_L1_560_280ns,            /**< @brief TCC timing level 1. */
  P_L2_840_420ns,            /**< @brief TCC timing level 2. */
  P_L3_1120_560ns,           /**< @brief TCC timing level 3. */
  P_L4_1400_700ns,           /**< @brief TCC timing level 4. */
  P_L5_1680_840ns,           /**< @brief TCC timing level 5. */
  P_L6_1960_980ns,           /**< @brief TCC timing level 6. */
  P_L7_2240_1120ns,          /**< @brief TCC timing level 7. */
  P_L8_2520_1260ns,          /**< @brief TCC timing level 8. */
  P_L9_2800_1400ns,          /**< @brief TCC timing level 9. */
  P_L10_3080_1540ns,         /**< @brief TCC timing level 10. */
  P_L11_3360_1680ns,         /**< @brief TCC timing level 11. */
  P_L12_3800_1900ns,         /**< @brief TCC timing level 12. */
  P_L13_4400_2200ns,         /**< @brief TCC timing level 13. */
  P_L14_4800_2400ns,         /**< @brief TCC timing level 14. */
  P_L15_5400_2700ns          /**< @brief TCC timing level 15. */
} stdrv102p_Timings_t;

/**
* @brief stdrv102p_gateLevels_t Enumerator for the IGATE: gate current selection.
*/
typedef enum
{
  P_L0_25mA = 0,           /**< @brief IGATE current level 0. */
  P_L1_75mA,               /**< @brief IGATE current level 1. */
  P_L2_150mA,              /**< @brief IGATE current level 2. */
  P_L3_250mA,              /**< @brief IGATE current level 3. */
  P_L4_300mA,              /**< @brief IGATE current level 4. */
  P_L5_350mA,              /**< @brief IGATE current level 5. */
  P_L6_400mA,              /**< @brief IGATE current level 6. */
  P_L7_500mA,              /**< @brief IGATE current level 7. */
  P_L8_550mA,              /**< @brief IGATE current level 8. */
  P_L9_600mA,              /**< @brief IGATE current level 9. */
  P_L10_650mA,             /**< @brief IGATE current level 10. */
  P_L11_700mA,             /**< @brief IGATE current level 11. */
  P_L12_800mA,             /**< @brief IGATE current level 12. */
  P_L13_850mA,             /**< @brief IGATE current level 13. */
  P_L14_900mA,             /**< @brief IGATE current level 14. */
  P_L15_1000mA             /**< @brief IGATE current level 15. */
} stdrv102p_gateLevels_t;

/**
* @brief stdriv102p_eqSel_t Enumerator for the EQ mode of the gate drivers.
*/
typedef enum
{
  P_IGATE_NOT_EQ = 0,              /**< @brief Selects IGATE,off = 2IGATE,on and TCC,on = 2TCC,off. */
  P_IGATE_EQ_TCC_HIGH_RANGE = 1,   /**< @brief Selects IGATE,off = IGATE,on and TCC,on = TCC,off (high range TCC). */
  P_IGATE_EQ_TCC_LOW_RANGE = 3     /**< @brief Selects IGATE,off = IGATE,on and TCC,on = TCC,off (low range TCC). */
} stdriv102p_eqSel_t;

/**
* @brief stdrv102p_in_mode_t Enumerator for the digital input mode of the gate drivers.
*/
typedef enum
{
  P_EN_IN = 0,                /**< @brief Selects ENx/INx mode */
  P_DIRECT_TCCWAIT = 4,       /**< @brief Selects INHx/INLx direct mode, with insertion of TCC wait. */
  P_DIRECT_NO_WAIT = 6,       /**< @brief Selects INHx/INLx direct mode, without insertion of TCC wait. */
  P_NO_INTERLOCKING = 7       /**< @brief Selects INHx/INLx direct mode, with interlocking disabled. */
} stdrv102p_in_mode_t;

/**
* @brief STDRIVE102P_GateDrivers_t Structure for the gate drivers configuration.
* This structure sets/gets the parameters of the functions STDRIVE102P_SetGateDrivers() and STDRIVE102P_GetGateDrivers().
*/
typedef struct
{
  stdrv102p_gateLevels_t igate_on;       /**< @brief IGATE: gate current selection. */
  stdrv102p_Timings_t tcc;               /**< @brief TCC: gate driver timing  selection. */
  stdriv102p_eqSel_t eq_mode_sel;        /**< @brief EQ mode of the gate drivers. */
  stdrv102p_in_mode_t drv_mode_sel;      /**< @brief Digital input mode of the gate drivers. */
} STDRIVE102P_GateDrivers_t;

/**
* @brief stdrv102p_uvlo_vals_t Enumerator for the undervoltage lock-out threshold selection.
*/
typedef enum
{
  P_UVLO_5V5 = 0,       /**< @brief Selects the low threshold for the VCC and charge pump UVLO. */
  P_UVLO_7V8            /**< @brief Selects the high threshold for the VCC and charge pump UVLO. */
} stdrv102p_uvlo_vals_t;

/**
* @brief stdrv102p_pwrgood_vals_t Enumerator for the VCC power good threhsold selection.
*/
typedef enum
{
  P_PGOOD_7V75 = 0,     /**< @brief Selects the low threshold for the VCC power good warning. */
  P_PGOOD_9V65          /**< @brief Selects the high threshold for the VCC power good warning. */
} stdrv102p_pwrgood_vals_t;

/**
* @brief STDRIVE102P_uvlo_t Structure for the UVLO and power good configuration.
* This structure sets/gets the parameters of the functions STDRIVE102P_SetUvlo() and STDRIVE102P_GetUvlo().
*/
typedef struct
{
  stdrv102p_uvlo_vals_t uvlo_sel;         /**< @brief Selects the threshold for the VCC and charge pump UVLO. */
  stdrv102p_pwrgood_vals_t pwrgood_sel;   /**< @brief Selects the threshold for the VCC power good. */
} STDRIVE102P_uvlo_t;

/**
* @brief stdrv102p_vdsDisTime_t Enumerator to select the disable time of the VDS monitoring.
*/
typedef enum
{
  P_VDS_TDIS_500us = 0,           /**< @brief Disable time: 500 microseconds. */
  P_VDS_TDIS_750us = 1,           /**< @brief Disable time: 750 microseconds. */
  P_VDS_TDIS_1ms = 2,             /**< @brief Disable time: 1 millisecond. */
  P_VDS_TDIS_2ms = 3,             /**< @brief Disable time: 2 milliseconds. */
  P_VDS_TDIS_3ms = 4,             /**< @brief Disable time: 3 milliseconds. */
  P_VDS_TDIS_5ms = 5,             /**< @brief Disable time: 5 milliseconds. */
  P_VDS_TDIS_10ms = 6,            /**< @brief Disable time: 10 milliseconds. */
  P_VDS_TDIS_20ms = 7             /**< @brief Disable time: 20 milliseconds. */
} stdrv102p_vdsDisTime_t;

/**
* @brief stdrv102p_vdsDeglitch_t Enumerator to select the deglitch time of the VDS monitoring filter.
*/
typedef enum 
{
  P_VDS_DGT_9us = 0,            /**< @brief Deglitch time: 9 microseconds. */
  P_VDS_DGT_8us = 1,            /**< @brief Deglitch time: 8 microseconds. */
  P_VDS_DGT_7us = 2,            /**< @brief Deglitch time: 7 microseconds. */
  P_VDS_DGT_6us = 3,            /**< @brief Deglitch time: 6 microseconds. */
  P_VDS_DGT_4us5 = 4,           /**< @brief Deglitch time: 4.5 microseconds. */
  P_VDS_DGT_3us5 = 5,           /**< @brief Deglitch time: 3.5 microseconds. */
  P_VDS_DGT_2us5 = 6            /**< @brief Deglitch time: 2.5 microseconds. */
} stdrv102p_vdsDeglitch_t;	

/**
* @brief STDRIVE102P_VDSmonitor_t Structure to configure the VDS monitoring filter.
* This structure sets/gets the parameters of the functions STDRIVE102P_SetVDSmonitor() and STDRIVE102P_GetVDSmonitor().
*/
typedef struct
{
  bool vds_soft_off;                       /**< @brief if true, enables the soft-off feature of the VDS monitoring. */
  bool vds_count_enable;                   /**< @brief if true, enables the counter of the VDS monitoring events. */
  stdrv102p_vdsDisTime_t vds_disable;      /**< @brief Sets the disable time after a VDS monitoring event. */
  stdrv102p_vdsDeglitch_t vds_deglitch;    /**< @brief Sets the deglitch time of the VDS monitoring filter. */
  uint8_t vds_counter;                     /**< @brief Sets the numebr of counts of the VDS monitoring. When it reaches 0, a futher event generates a latch. */
} STDRIVE102P_VDSmonitor_t;

/**
* @brief stdrv102p_pga_gain_t Enumerator to select the gain of the PGA in the analog front-end of the STDRIVE102.
*/
typedef enum
{
  P_PGA_GAIN_4 = 0,                   /**< @brief PGA gain = 4. */
  P_PGA_GAIN_8 = 1,                   /**< @brief PGA gain = 8. */
  P_PGA_GAIN_16 = 2,                  /**< @brief PGA gain = 16. */
  P_PGA_GAIN_32 = 3                   /**< @brief PGA gain = 32. */	
} stdrv102p_pga_gain_t;

/**
* @brief stdrv102p_inp_config_t Enumerator to select which is the positive input of the comparator in the analog front-end of the STDRIVE102.
*/
typedef	enum
{
  P_COMP_DIRECT_INP = 0,           /**< @brief Comparator input connected to the posisive input of the PGA. */
  P_COMP_PGA_OUTPUT = 1           /**< @brief Comparator input connected to the output of the PGA. */
} stdrv102p_inp_config_t;

/**
* @brief stdrv102p_inn_config_t Enumerator to select which is the negative input of the comparator in the analog front-end of the STDRIVE102.
*/
typedef enum
{
  P_COMP_TH_INTERNAL = 0,        /**< @brief Use the internal adjustable threshold. */
  P_COMP_TH_EXT_CREF = 1         /**< @brief Use the voltage reference present on the CREF pin of the STDRIVE102. */	
} stdrv102p_inn_config_t;

/**
* @brief stdrv102p_compDeglitch_t Enumerator to select the deglitch time of the comparators's filter.
*/
typedef enum
{
  P_COMP_DGT_disabled = 0,         /**< @brief Deglitch filter is disabled. */
  P_COMP_DGT_600ns = 1,            /**< @brief Deglitch filter is 600 nanoseconds. */
  P_COMP_DGT_1300ns = 2,           /**< @brief Deglitch filter is 1300 nanoseconds. */
  P_COMP_DGT_2500ns = 3            /**< @brief Deglitch filter is 2500 nanoseconds. */
} stdrv102p_compDeglitch_t;

/**
* @brief stdrv102p_compDisTime_t Enumerator to select the disable time after a comparator's event.
*/
typedef enum
{
  P_COMP_DIST_0us = 0,              /**< @brief No Disable time. */
  P_COMP_DIST_10us = 1,             /**< @brief Disable time is 10 microseconds. */
  P_COMP_DIST_15us = 2,             /**< @brief Disable time is 15 microseconds. */
  P_COMP_DIST_20us = 3,             /**< @brief Disable time is 20 microseconds. */
  P_COMP_DIST_25us = 4,             /**< @brief Disable time is 25 microseconds. */
  P_COMP_DIST_35us = 5,             /**< @brief Disable time is 35 microseconds. */
  P_COMP_DIST_45us = 6,             /**< @brief Disable time is 45 microseconds. */
  P_COMP_DIST_55us = 7,             /**< @brief Disable time is 55 microseconds. */
  P_COMP_DIST_75us = 8,             /**< @brief Disable time is 75 microseconds. */
  P_COMP_DIST_110us = 9,            /**< @brief Disable time is 110 microseconds. */
  P_COMP_DIST_160us = 10,           /**< @brief Disable time is 160 microseconds. */
  P_COMP_DIST_220us = 11,           /**< @brief Disable time is 220 microseconds. */
  P_COMP_DIST_330us = 12,           /**< @brief Disable time is 330 microseconds. */
  P_COMP_DIST_550us = 13,           /**< @brief Disable time is 550 microseconds. */
  P_COMP_DIST_880us = 14,           /**< @brief Disable time is 880 microseconds. */
  P_COMP_DIST_1100us = 15           /**< @brief Disable time is 1100 microseconds. */
} stdrv102p_compDisTime_t;

/**
* @brief STDRIVE102P_AFEchannel_t Structure to configure the AFE channel.
* This structure sets/gets the parameters of the functions STDRIVE102P_SetAFEchannel() and STDRIVE102P_GetAFEchannel().
*/
typedef struct
{
  bool pga_enable;                                /**< @brief if true, enables the PGA of the AFE. */
  bool comp_enable;                               /**< @brief if true, enables the comparator of the AFE. */
  stdrv102p_inp_config_t comp_pos_input;          /**< @brief configures the positive input of the comparator. */
  stdrv102p_inn_config_t comp_neg_input;          /**< @brief configures the negative input of the comparator. */
  bool comp_inversion;                            /**< @brief if true, inverts the output of the comparator. */
  stdrv102p_pga_gain_t pga_gain;                  /**< @brief configure the gain of the PGA. */
  bool comp_soft_off;                             /**< @brief if true, enables the soft-off feature when the comparator is triggered. */
  bool comp_fault_enable;                         /**< @brief if true, enables the fault management of the AFE. */
  bool comp_counter_enable;                       /**< @brief if true, enables the event counter of the comparator. */
  stdrv102p_compDeglitch_t comp_deglitch;         /**< @brief configures the degltich time of the comparator's filter. */
  stdrv102p_compDisTime_t comp_disable;           /**< @brief configures the disable time after a comparator event. */
  uint8_t comp_counter;                           /**< @brief Sets the numebr of counts of the AFE comparator. When it reaches 0, a futher event generates a latch. */
} STDRIVE102P_AFEchannel_t;

/**
* @brief stdrv102p_counter_index_t Enumerator to select which counter must be written or read.
* Use this enumerator with the functions STDRIVE102P_SetCounter() and STDRIVE102P_GetCounter().
*/
typedef enum
{
  P_VDS_COUNTER = 0x07,               /**< @brief Selects the coutner of the VDS monitoring */
  P_AFE_CH1_COUNTER = 0x13            /**< @brief Selects the coutner of the AFE channel */
} stdrv102p_counter_index_t;

/**
* @brief stdrv102p_pga_offset_t Enumerator to select the offset present on the PGA output.
* According to the offset selected it is possible to read unipolar or bipolar signals on the PGA inputs.
*/
typedef enum
{
  P_PGA_NO_OFFSET = 0,          /**< @brief No offset on PGA output. Suitable for unipolar signals */
  P_PGA_HALF_VDD = 1            /**< @brief No offset on PGA output. Suitable for bipolar signals */
} stdrv102p_pga_offset_t;

/**
* @brief stdrv102p_compInternalThreshold_t Enumerator to select the level of the internal threshold applied to the AFE comparator.
*/
typedef enum
{
  P_COMP_TH_INT_100mV = 0,          /**< @brief internal threshold is 100 mV */
  P_COMP_TH_INT_250mV = 1,          /**< @brief internal threshold is 250 mV */
  P_COMP_TH_INT_300mV = 2,          /**< @brief internal threshold is 300 mV */
  P_COMP_TH_INT_500mV = 3,          /**< @brief internal threshold is 500 mV */
  P_COMP_TH_INT_600mV = 4,          /**< @brief internal threshold is 600 mV */
  P_COMP_TH_INT_1V2 = 5,            /**< @brief internal threshold is 1200 mV */
  P_COMP_TH_INT_1V65 = 6,           /**< @brief internal threshold is VDD/2 */
  P_COMP_TH_INT_1V85 = 7,           /**< @brief internal threshold is 11/20 VDD */
  P_COMP_TH_INT_2V25 = 8,           /**< @brief internal threshold is 14/20 VDD */
  P_COMP_TH_INT_2V85 = 9,           /**< @brief internal threshold is 17/20 VDD */
  P_COMP_TH_INT_3V15 = 10           /**< @brief internal threshold is 19/20 VDD */
} stdrv102p_compInternalThreshold_t;

/**
* @brief stdrv102p_safeStateSel_t Enumerator to select the safe state of the power stage after the comparator is triggered.
*/	
typedef enum
{
  P_MOS_ALL_OFF = 0,             /**< @brief All the MOSFETs in the power stage are turned off. */
  P_MOS_HS_ONLY_OFF = 2,         /**< @brief Only the high side MOSFETs in the power stage are turned off. */
  P_MOS_FAULT_BYPASS = 3         /**< @brief No MOSFETs in the power stage is turned off. */
} stdrv102p_safeStateSel_t;

/**
* @brief STDRIVE102P_AFEcommon_t Structure to configure the additional configuration parameters of the AFE.
* This structure sets/gets the parameters of the functions STDRIVE102P_SetAFEcommon() and STDRIVE102P_GetAFEcommon().
*/	
typedef struct
{
  stdrv102p_pga_offset_t pga_ref_sel;                   /**< @brief Configures the output offset of the PGA. */
  stdrv102p_compInternalThreshold_t comp_internal_th;   /**< @brief Configures the internal threshold for the comparator. */
  stdrv102p_safeStateSel_t driver_safe_state;           /**< @brief Configures the safe state of the power stage after a comparator event. */
} STDRIVE102P_AFEcommon_t;

/**
* @brief STDRIVE102P_hw_interface_t Structure to define the SPI interface of the STDRIVE102.
*/
typedef struct
{
  SPI_TypeDef *spi_hdl;                 /**< @brief SPI handler. */
  GPIO_TypeDef *nCS_gpio_port;          /**< @brief SPI nCS/nSS GPIO port (SW managed). */
  uint32_t nCS_gpio_pin;                /**< @brief SPI nCS/nSS GPIO pin (SW managed). */
} STDRIVE102P_hw_interface_t;

/**
* @brief STDRIVE102P_defaultParams_t This structure is used to manage all the configurations of the STDRIVE102.
*/
typedef struct
{
  STDRIVE102P_hw_interface_t spi;        /**< @brief SPI configuration structure. */
  STDRIVE102P_GateDrivers_t gd;          /**< @brief Gate drivers configuration structure. */
  STDRIVE102P_uvlo_t uv;                 /**< @brief UVLO thresholds configuration structure. */
  STDRIVE102P_VDSmonitor_t vdsm;         /**< @brief VDS monitoring configuration structure. */
  STDRIVE102P_AFEchannel_t afe_ch;       /**< @brief AFE configuration structure. */
  STDRIVE102P_AFEcommon_t afe_cm;        /**< @brief AFE additional configuration structure. */
}STDRIVE102P_defaultParams_t;


/**
* @brief STDRIVE102P_Handle_t handler definition. 
* The handler is passed to all the library function to manage a spedific instance of the STDRIVE102P.
*/
typedef struct
{
  const STDRIVE102P_defaultParams_t *defaultParams;
} STDRIVE102P_Handle_t;


/* Exported functions ------------------------------------------------------- */

__weak void STDRIVE102P_Init(STDRIVE102P_Handle_t *pHandler, const STDRIVE102P_defaultParams_t *params);
            
__weak void STDRIVE102P_FaultManagement( STDRIVE102P_Handle_t *pHandler);
__weak void STDRIVE102P_StartManagement( STDRIVE102P_Handle_t *pHandler);
__weak void STDRIVE102P_StopManagement( STDRIVE102P_Handle_t *pHandler);
__weak void STDRIVE102P_MF_Management( STDRIVE102P_Handle_t *pHandler);

uint16_t STDRIVE102P_ReadReg (STDRIVE102P_Handle_t *pHandler, uint8_t reg_addr, uint8_t *reg_value, uint8_t *status1);
uint16_t STDRIVE102P_WriteReg (STDRIVE102P_Handle_t *pHandler, uint8_t reg_addr, uint8_t reg_value, uint8_t *status1, uint8_t *status2);

uint16_t STDRIVE102P_SetGateDrivers (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_GateDrivers_t gd_par);
uint16_t STDRIVE102P_GetGateDrivers (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_GateDrivers_t *gd_par);

uint16_t STDRIVE102P_SetUvlo (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_uvlo_t uv_par);
uint16_t STDRIVE102P_GetUvlo (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_uvlo_t *uv_par);

uint16_t STDRIVE102P_SetVDSmonitor (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_VDSmonitor_t vds_par);
uint16_t STDRIVE102P_GetVDSmonitor (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_VDSmonitor_t *vds_par);

uint16_t STDRIVE102P_SetAFEchannel (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEchannel_t afe_par);
uint16_t STDRIVE102P_GetAFEchannel (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEchannel_t *afe_par);
 
uint16_t STDRIVE102P_SetAFEcommon (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEcommon_t afe_par);
uint16_t STDRIVE102P_GetAFEcommon (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEcommon_t *afe_par);

uint16_t STDRIVE102P_AFE_Enable (STDRIVE102P_Handle_t *pHandler);
uint16_t STDRIVE102P_AFE_Disable (STDRIVE102P_Handle_t *pHandler);

uint16_t STDRIVE102P_SetCounter (STDRIVE102P_Handle_t *pHandler, stdrv102p_counter_index_t cnt, uint8_t value);
uint16_t STDRIVE102P_GetCounter (STDRIVE102P_Handle_t *pHandler, stdrv102p_counter_index_t cnt, uint8_t *value);

uint16_t STDRIVE102P_SetFAULTpinSignal (STDRIVE102P_Handle_t *pHandler, uint8_t val);
uint16_t STDRIVE102P_GetFAULTpinSignal (STDRIVE102P_Handle_t *pHandler, uint8_t *val);

uint16_t STDRIVE102P_GetStatus2 (STDRIVE102P_Handle_t *pHandler, uint8_t *status1, uint8_t *status2);
uint16_t STDRIVE102P_GetStatus3 (STDRIVE102P_Handle_t *pHandler, uint8_t *status3, uint8_t *status1);
uint16_t STDRIVE102P_GetAFEStatus (STDRIVE102P_Handle_t *pHandler, uint8_t *afe_status, uint8_t *status1);

uint16_t STDRIVE102P_ClearFault_Cmd (STDRIVE102P_Handle_t *pHandler, uint8_t cmd);
uint16_t STDRIVE102P_DevReset_Cmd (STDRIVE102P_Handle_t *pHandler);
uint16_t STDRIVE102P_Lock_Cmd (STDRIVE102P_Handle_t *pHandler);
uint16_t STDRIVE102P_Unlock_Cmd (STDRIVE102P_Handle_t *pHandler);


/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif /* __cpluplus */


#endif /* STDRIVE102P_DRIVER_H */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
