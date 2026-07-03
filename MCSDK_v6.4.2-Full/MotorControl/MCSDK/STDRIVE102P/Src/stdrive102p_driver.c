/**
******************************************************************************
* @file stdrive102p_driver.c
* @author Motor Control SDK Team, ST Microelectronics
* @brief Firmware library for the configuration and management of the
*		 STDRIVE102P gate driver.
*
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
/* Includes ------------------------------------------------------------------*/
#include "stdrive102p_driver.h"

/** @addtogroup MCSDK
  * @{
  */

  

/** @addtogroup COMPLEX_DRIVERS
  * @{
  */

/** @defgroup stdrive102pDriver STDRIVE102P driver
  * @brief Configuration and management functions for the STDRIVE102P gate driver
  *
  * @{
  */
/**
  * @brief  It perform a write operation on the SPI bus of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] reg_addr Address of the register to be written.
  * @param  [in] reg_value Value to be written in the register.
  * @param  [out] status1 Value of STATUS1 regiter returned by the STDRIVE102.
  * @param  [out] status2 Value of STATUS1 regiter returned by the STDRIVE102.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_WriteReg (STDRIVE102P_Handle_t *hdl, uint8_t reg_addr, uint8_t reg_value, uint8_t *status1, uint8_t *status2)
{	
  uint16_t spi_tx_value;
  uint16_t spi_rx_value;
	
  uint32_t seq = 0; // sequence index of the SPI state machine
  uint32_t ticks; // number of ticks corresponding to SPI timeout
  uint32_t tickFreq; // frequency of the ticks
  volatile uint32_t wait_nCS_cnt = 0; // counter for delay implementation
	
  uint16_t error_code = 0;
	

  if ((hdl == NULL) || (status1 == NULL) || (status2 == NULL))
  {
    error_code = STDRIVE102P_SYS_ERROR; 
    return (error_code);
  }


  spi_tx_value = ((uint16_t)reg_addr & 0x007F) << 8;
  spi_tx_value &= 0xFF00;
  spi_tx_value |= ((uint16_t)reg_value & 0x00FF);
	
  // SPI timeout measure based on SysTick
  // Use System Core Clock to calculate the SPI minimum timeout
  // Do not consider other prescalers in the systick clock path
  tickFreq = SystemCoreClock;
  tickFreq = tickFreq /  (SysTick->LOAD + 1);
  if (LL_SYSTICK_GetClkSource() == LL_SYSTICK_CLKSOURCE_HCLK_DIV8)
  {
    tickFreq /= 8;
  }
  ticks = (STDRIVE102P_SPI_TIMEOUT * tickFreq) / 1000 + 1;
		
	
  while (true)
  {
    switch (seq)
    {
      case 0:
      LL_SPI_Enable(hdl->defaultParams->spi.spi_hdl);
      seq++;
      break;
	
      case 1:
      #if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 
      LL_SPI_StartMasterTransfer(hdl->defaultParams->spi.spi_hdl);
      #endif
      seq += STDRIVE102P_spi_TX_ready(hdl->defaultParams->spi.spi_hdl);
      break;
	
      case 2:
      /* nCS low ----------------------------------------*/
      LL_GPIO_ResetOutputPin(hdl->defaultParams->spi.nCS_gpio_port, hdl->defaultParams->spi.nCS_gpio_pin);
      #ifdef STDRIVE102P_SPI_CS_DELAY
      /* Coarse delay between nCS falling edge and communication start     */
      /* Note: Variable divided by 10 to compensate partially              */
      /*       CPU processing cycles, scaling in us split to not           */
      /*       exceed 32 bits register capacity and handle low frequency.  */
      wait_nCS_cnt = SystemCoreClock;
      wait_nCS_cnt /= 1000000UL;
      wait_nCS_cnt *= STDRIVE102P_SPI_CS_DELAY;
      wait_nCS_cnt /= 10;
      
      while(wait_nCS_cnt != 0UL)
      {
        wait_nCS_cnt--;
      }	
      #endif //SPI_CS_DELAY
      seq++;
      break;
      
      case 3:
      LL_SPI_TransmitData16(hdl->defaultParams->spi.spi_hdl, spi_tx_value);
      seq++;
      break;
      
      case 4:
      seq += STDRIVE102P_spi_RX_ready(hdl->defaultParams->spi.spi_hdl);
      break;
      
      case 5:
      spi_rx_value = LL_SPI_ReceiveData16(hdl->defaultParams->spi.spi_hdl);
      *status1 = (uint8_t)((spi_rx_value >> 8) & 0x00FF);
      *status2 = (uint8_t)(spi_rx_value & 0x00FF);
      seq++;
      break;
      
      case 6:
      seq += STDRIVE102P_spi_TX_ready(hdl->defaultParams->spi.spi_hdl);
      break;
      
      case 7:
      #if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 
      seq++;
      #else
      if (LL_SPI_IsActiveFlag_BSY(hdl->defaultParams->spi.spi_hdl) == 0) 
        seq++;
      #endif			
      break;
      
      case 8:
      /* error code calculation based on the STDRIVE102 STATUS1 reg. */
      /* complement active-low bits (no error should return 0)       */
      error_code = (uint16_t)(*status1 ^ 0xFE);
      error_code &= 0xFF7F; // remove LOCKED bit (set at 0)
      seq++;
      break;
      
      case 9:
      LL_SPI_ClearFlag_OVR(hdl->defaultParams->spi.spi_hdl);
      LL_SPI_Disable(hdl->defaultParams->spi.spi_hdl);
      #if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 			
      LL_SPI_ClearFlag_MODF(hdl->defaultParams->spi.spi_hdl);
      #endif
      
      #ifdef STDRIVE102P_SPI_CS_DELAY
      /* Coarse delay between communication stop and nCS rising edge      */
      /* Note: Variable divided by 10 to compensate partially             */
      /*       CPU processing cycles, scaling in us split to not          */
      /*       exceed 32 bits register capacity and handle low frequency. */
      wait_nCS_cnt = SystemCoreClock;
      wait_nCS_cnt /= 1000000UL;
      wait_nCS_cnt *= STDRIVE102P_SPI_CS_DELAY;
      wait_nCS_cnt /= 10;
      
      while(wait_nCS_cnt != 0UL)
      {
        wait_nCS_cnt--;
      }	
      #endif //SPI_CS_DELAY
      
      // nCS set high
      LL_GPIO_SetOutputPin(hdl->defaultParams->spi.nCS_gpio_port, hdl->defaultParams->spi.nCS_gpio_pin);
                        
      #ifdef STDRIVE102P_SPI_FRAME_DELAY
      /* Waits before starting the next communication (next nCS falling edge) */
      /* Note: Variable divided by 10 to compensate partially              */
      /*       CPU processing cycles, scaling in us split to not          */
      /*       exceed 32 bits register capacity and handle low frequency. */
      wait_nCS_cnt = SystemCoreClock;
      wait_nCS_cnt /= 1000000UL;
      wait_nCS_cnt *= STDRIVE102P_SPI_FRAME_DELAY;
      wait_nCS_cnt /= 10;
      
      while(wait_nCS_cnt != 0UL)
      {
      wait_nCS_cnt--;
      }	
      #endif //SPI_FRAME_DELAY	
      
      return (error_code);
      
      default:
      error_code = STDRIVE102P_SYS_ERROR;
      return (error_code);
    }

    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      ticks--;
    }
    
    if(ticks == 0)
    {
      error_code = STDRIVE102P_SPI_TIMEOUT_ERROR;
      seq = 9;
    }
  }		
}

/**
  * @brief  It perform a read operation on the SPI bus of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] reg_addr Address of the register to be read.
  * @param  [out] reg_value Value read.
  * @param  [out] status1 Value of STATUS1 regiter returned by the STDRIVE102.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_ReadReg (STDRIVE102P_Handle_t *hdl, uint8_t reg_addr, uint8_t *reg_value, uint8_t *status1)
{
  uint16_t spi_tx_value;
  uint16_t spi_rx_value;
	
  uint32_t seq = 0; // sequence index of the SPI state machine
  uint32_t ticks; // number of ticks corresponding to SPI timeout
  uint32_t tickFreq; // frequency of the ticks
  volatile uint32_t wait_nCS_cnt = 0; // counter for delay implementation

  uint16_t error_code = 0;

  if ((hdl == NULL) || (status1 == NULL))
  {
    error_code = STDRIVE102P_SYS_ERROR; 
    return (error_code);
  }


  spi_tx_value = ((uint16_t)reg_addr & 0x00FF);
  spi_tx_value = (spi_tx_value << 8);
  spi_tx_value |= 0x8000;
  spi_tx_value &= 0xFF00;

  /* SPI timeout measure based on SysTick                       */
  /* Use System Core Clock to calculate the SPI minimum timeout */
  /* Do not consider other prescalers in the systick clock path */
  tickFreq = SystemCoreClock;
  tickFreq = tickFreq / (SysTick->LOAD + 1);
  if (LL_SYSTICK_GetClkSource() == LL_SYSTICK_CLKSOURCE_HCLK_DIV8)
  {
    tickFreq /= 8;
  }
  ticks = (STDRIVE102P_SPI_TIMEOUT * tickFreq) / 1000 + 1;


  while (true)
  {
    switch (seq)
    {
      case 0:
      LL_SPI_Enable(hdl->defaultParams->spi.spi_hdl);
      seq++;			
      break;
      
      case 1:
      #if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 
      LL_SPI_StartMasterTransfer(hdl->defaultParams->spi.spi_hdl);
      #endif
      seq += STDRIVE102P_spi_TX_ready(hdl->defaultParams->spi.spi_hdl);
      break;
      
      case 2:
      // nCS low
      LL_GPIO_ResetOutputPin(hdl->defaultParams->spi.nCS_gpio_port, hdl->defaultParams->spi.nCS_gpio_pin);
      #ifdef STDRIVE102P_SPI_CS_DELAY
      /* Coarse delay between nCS falling edge and communication start     */
      /* Note: Variable divided by 10 to compensate partially              */
      /*       CPU processing cycles, scaling in us split to not          */
      /*       exceed 32 bits register capacity and handle low frequency. */
      wait_nCS_cnt = SystemCoreClock;
      wait_nCS_cnt /= 1000000UL;
      wait_nCS_cnt *= STDRIVE102P_SPI_CS_DELAY;
      wait_nCS_cnt /= 10;
      while(wait_nCS_cnt != 0UL)
      {
        wait_nCS_cnt--;
      }	
      #endif //SPI_CS_DELAY		
      seq++;
      break;
      
      case 3:
      LL_SPI_TransmitData16(hdl->defaultParams->spi.spi_hdl, spi_tx_value);
      seq++;
      break;
      
      case 4:
      seq += STDRIVE102P_spi_RX_ready(hdl->defaultParams->spi.spi_hdl);
      break;
      
      case 5:
      spi_rx_value = LL_SPI_ReceiveData16(hdl->defaultParams->spi.spi_hdl);
      *status1 = (uint8_t)((spi_rx_value >> 8) & 0x00FF);
      *reg_value = (uint8_t)(spi_rx_value & 0x00FF);
      seq++;
      break;
      
      case 6:
      seq += STDRIVE102P_spi_TX_ready(hdl->defaultParams->spi.spi_hdl);
      break;
      
      case 7:
      #if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 
      seq++;
      #else
      if (LL_SPI_IsActiveFlag_BSY(hdl->defaultParams->spi.spi_hdl) == 0) 
        seq++;
      #endif
      break;

      case 8:
      // error code calculation based on the STDRIVE102 STATUS1 reg.
      // complement active-low bits (no error should return 0)
      error_code = (uint16_t)(*status1 ^ 0xFE);
      error_code &= 0xFF7F; // remove LOCKED bit (set at 0)
      seq++;
      break;
      
      case 9:
      LL_SPI_ClearFlag_OVR(hdl->defaultParams->spi.spi_hdl);
      LL_SPI_Disable(hdl->defaultParams->spi.spi_hdl);
      #if ((defined STM32H5xx_LL_SPI_H) || (defined STM32H7xx_LL_SPI_H) || (defined STM32U5xx_LL_SPI_H)) 			
      LL_SPI_ClearFlag_MODF(hdl->defaultParams->spi.spi_hdl);
      #endif
      
      #ifdef STDRIVE102P_SPI_CS_DELAY
      /* Coarse delay between communication stop and nCS rising edge   */
      /* Note: Variable divided by 10 to compensate partially              */
      /*       CPU processing cycles, scaling in us split to not          */
      /*       exceed 32 bits register capacity and handle low frequency. */
      wait_nCS_cnt = SystemCoreClock;
      wait_nCS_cnt /= 1000000UL;
      wait_nCS_cnt *= STDRIVE102P_SPI_CS_DELAY;
      wait_nCS_cnt /= 10;
      
      while(wait_nCS_cnt != 0UL)
      {
      wait_nCS_cnt--;
      }	
      #endif //SPI_CS_DELAY
      
      /* nCS set high -------------------------------------------------*/
      LL_GPIO_SetOutputPin(hdl->defaultParams->spi.nCS_gpio_port, hdl->defaultParams->spi.nCS_gpio_pin);
                      
      #ifdef STDRIVE102P_SPI_FRAME_DELAY
      /* Waits before starting the next communication (next nCS falling edge) */
      /* Note: Variable divided by 10 to compensate partially              */
      /*       CPU processing cycles, scaling in us split to not          */
      /*       exceed 32 bits register capacity and handle low frequency. */
      wait_nCS_cnt = SystemCoreClock;
      wait_nCS_cnt /= 1000000UL;
      wait_nCS_cnt *= STDRIVE102P_SPI_FRAME_DELAY;
      wait_nCS_cnt /= 10;
      
      while(wait_nCS_cnt != 0UL)
      {
        wait_nCS_cnt--;
      }	
      #endif //SPI_FRAME_DELAY
      
      return (error_code);
      
      default:
      error_code = STDRIVE102P_SYS_ERROR;
      return (error_code);
    }

    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      ticks--;
    }

    if(ticks == 0)
    {
      error_code = STDRIVE102P_SPI_TIMEOUT_ERROR;
      seq = 9;
    }
  }		
}

/**
  * @brief  It set the configuration of the gate drivers of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] gd_par Structure with the configuration parameters of the gate drivers.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_SetGateDrivers (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_GateDrivers_t gd_par)
{
  uint8_t drv_cfg_reg_4 = 0;
  uint8_t sys_cfg_reg_5 = 0;
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code = 0;
  
  if (pHandler == NULL)
    return STDRIVE102P_SYS_ERROR;
  if ((uint8_t)gd_par.igate_on > 15)
    return STDRIVE102P_CONFIG_DATA_ERROR;
  if ((uint8_t)gd_par.tcc > 15)
    return STDRIVE102P_CONFIG_DATA_ERROR;
  
  if ((gd_par.drv_mode_sel != P_EN_IN) && (gd_par.drv_mode_sel != P_DIRECT_TCCWAIT) && 
      (gd_par.drv_mode_sel != P_DIRECT_NO_WAIT) && (gd_par.drv_mode_sel != P_NO_INTERLOCKING))
    return STDRIVE102P_CONFIG_DATA_ERROR;
                
  if ((gd_par.eq_mode_sel != P_IGATE_NOT_EQ) && (gd_par.eq_mode_sel != P_IGATE_EQ_TCC_HIGH_RANGE) && 
      (gd_par.eq_mode_sel != P_IGATE_EQ_TCC_LOW_RANGE))
    return STDRIVE102P_CONFIG_DATA_ERROR;
                  
  error_code |= STDRIVE102P_Unlock_Cmd (pHandler);
  
  /* configure register at DRV_CFG at addr. 0x04 */
  drv_cfg_reg_4 = ((uint8_t)gd_par.igate_on & 0x0F);
  drv_cfg_reg_4 |= (((uint8_t)gd_par.tcc & 0x0F) << 4);
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x04, drv_cfg_reg_4, &status1, &status2);
  
  /* configure register at SYS_CFG at addr. 0x05                */
  /* At first read the value of SYS_CFG to avoid bits overwrite */
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x05, &sys_cfg_reg_5, &status1);
  
  sys_cfg_reg_5 &= 0xE0;
  sys_cfg_reg_5 |=(((uint8_t)gd_par.eq_mode_sel << 3) & 0x18);
  
  /* the value of the enum stdrv102p_in_mode_t must be bitwise complemented           */
  /* in order to match the configuration in the bits: MODE_SEL, TCC_WAIT, INTLOCK     */
  /* in SYS_CFG register at address 0x05                                              */
  sys_cfg_reg_5 |= (((uint8_t)gd_par.drv_mode_sel ^ 0x07) & 0x07);
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x05, sys_cfg_reg_5, &status1, &status2);
  error_code |= STDRIVE102P_Lock_Cmd (pHandler);
  
  return (error_code);
}

/**
  * @brief  It get the configuration of the gate drivers of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] gd_par The configuration parameters get from the STDRIVE102 is stored in this structure.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetGateDrivers (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_GateDrivers_t *gd_par)
{
  uint8_t drv_cfg_reg_4 = 0;
  uint8_t sys_cfg_reg_5 = 0;
  uint8_t status1;
  uint16_t error_code = 0;
  
  if ((pHandler == NULL) || (gd_par == NULL)) return STDRIVE102P_SYS_ERROR;
  
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x04, &drv_cfg_reg_4, &status1);
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x05, &sys_cfg_reg_5, &status1);
  
  gd_par->igate_on = (stdrv102p_gateLevels_t)(drv_cfg_reg_4 & 0x0F);
  gd_par->tcc = (stdrv102p_Timings_t)((drv_cfg_reg_4 >> 4) & 0x0F);
  
  if ((sys_cfg_reg_5 & 0x08) == 0) gd_par->eq_mode_sel = P_IGATE_NOT_EQ;
  else if ((sys_cfg_reg_5 & 0x10) == 0) gd_par->eq_mode_sel = P_IGATE_EQ_TCC_HIGH_RANGE;
  else gd_par->eq_mode_sel = P_IGATE_EQ_TCC_LOW_RANGE;  
  
  if ((sys_cfg_reg_5 & 0x04) != 0) gd_par->drv_mode_sel = P_EN_IN ;
  else if ((sys_cfg_reg_5 & 0x01) == 0) gd_par->drv_mode_sel = P_NO_INTERLOCKING;
  else if ((sys_cfg_reg_5 & 0x02) == 0) gd_par->drv_mode_sel = P_DIRECT_NO_WAIT;
  else gd_par->drv_mode_sel = P_DIRECT_TCCWAIT;
  
  return (error_code); 
}

/**
  * @brief  It seletcs the undervoltage lock-out thresholds of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] uv_par Structure with selectors of the UVLO thresholds.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_SetUvlo (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_uvlo_t uv_par)
{
  uint8_t sys_cfg_reg_5 = 0;
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code = 0;
	
  if (pHandler == NULL)
    return STDRIVE102P_SYS_ERROR;
        
  /* configure register at SYS_CFG at addr. 0x05        */
  /* read the value of SYS_CFG to avoid bits overwrite  */
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x05, &sys_cfg_reg_5, &status1);
	
  if (uv_par.uvlo_sel == P_UVLO_5V5)
  {
    sys_cfg_reg_5 &= 0xDF;
  }
  else if(uv_par.uvlo_sel == P_UVLO_7V8)
  {
    sys_cfg_reg_5 |= 0x20;
  }
  else
  {
    return STDRIVE102P_CONFIG_DATA_ERROR;
  }
	
	
  if(uv_par.pwrgood_sel == P_PGOOD_7V75)
  {
    sys_cfg_reg_5 &= 0xBF;
  }
  else if(uv_par.pwrgood_sel == P_PGOOD_9V65)
  {
    sys_cfg_reg_5 |= 0x40;
  }
  else
  {
    return STDRIVE102P_CONFIG_DATA_ERROR;
  }
          
  error_code |= STDRIVE102P_Unlock_Cmd (pHandler);
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x05, sys_cfg_reg_5, &status1, &status2);
  error_code |= STDRIVE102P_Lock_Cmd (pHandler);

  return (error_code); 
}

/**
  * @brief  It get the configuration of the undervoltage lock-out thresholds of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] uv_par The configuration of the UVLO thresholds is stored in this structure.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetUvlo (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_uvlo_t *uv_par)
{
  uint8_t sys_cfg_reg_5 = 0;
  uint8_t status1;
  uint16_t error_code;
  
  if ((pHandler == NULL) || (uv_par == NULL))
    return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_ReadReg (pHandler, 0x05, &sys_cfg_reg_5, &status1);
  
  uv_par->uvlo_sel = (stdrv102p_uvlo_vals_t)((sys_cfg_reg_5 >> 5) & 0x01);
  uv_par->pwrgood_sel = (stdrv102p_pwrgood_vals_t)((sys_cfg_reg_5 >> 6) & 0x01);
  
  return (error_code); 	
}

/**
  * @brief  It seletcs the configuration of the VDS monitoring protection of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] vds_par Structure with the configuration parameters of the VDS monitoring protection.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_SetVDSmonitor (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_VDSmonitor_t vds_par)
{
  uint8_t vds_cfg_reg_6 = 0;
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code = 0;
          
  if (pHandler == NULL)
    return STDRIVE102P_SYS_ERROR;
        
  if ((uint8_t)vds_par.vds_disable > 7)
    return STDRIVE102P_CONFIG_DATA_ERROR;
        
  if ((uint8_t)vds_par.vds_deglitch > 6)
    return STDRIVE102P_CONFIG_DATA_ERROR;        
  
  error_code |= STDRIVE102P_Unlock_Cmd (pHandler);
  
  if(vds_par.vds_soft_off) vds_cfg_reg_6 |= 0x80;
  if(vds_par.vds_count_enable) vds_cfg_reg_6 |= 0x40;
  vds_cfg_reg_6 |= (((uint8_t)vds_par.vds_disable << 3) & 0x38);
  vds_cfg_reg_6 |= ((uint8_t)vds_par.vds_deglitch & 0x07);
  
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x06, vds_cfg_reg_6, &status1, &status2);
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x07, vds_par.vds_counter, &status1, &status2);
  error_code |= STDRIVE102P_Lock_Cmd (pHandler);
  
  return (error_code); 
}

/**
  * @brief  It get the configuration of the VDS monitoring protection of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] vds_par The configuration of the VDS monitoring protection is stored in this structure.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetVDSmonitor (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_VDSmonitor_t *vds_par)
{
  uint8_t vds_cfg_reg_6 = 0;
  uint8_t vds_cnt_reg_7 = 0;
  uint8_t status1;
  uint16_t error_code = 0;
  
  if ((pHandler == NULL) || (vds_par == NULL))
    return STDRIVE102P_SYS_ERROR;
  
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x06, &vds_cfg_reg_6, &status1);
  
  vds_par->vds_soft_off = ((vds_cfg_reg_6 & 0x80) != 0) ? true : false ;
  vds_par->vds_count_enable = ((vds_cfg_reg_6 & 0x40) != 0) ? true : false;
  vds_par->vds_disable = (stdrv102p_vdsDisTime_t)((vds_cfg_reg_6 >> 3) & 0x07);
  vds_par->vds_deglitch = (stdrv102p_vdsDeglitch_t)(vds_cfg_reg_6 & 0x07);
  
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x07, &vds_cnt_reg_7, &status1);
  
  vds_par->vds_counter = vds_cnt_reg_7;
  
  return (error_code); 
}

/**
  * @brief  It seletcs the configuration of the Analog Front-End channel of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] afe_par Structure with the configuration parameters of the AFE channel.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_SetAFEchannel (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEchannel_t afe_par)
{
  uint8_t afe_cfg_reg = 0;
  uint8_t afe_flt_reg = 0;
  uint8_t comp_cfg_reg = 0;
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code = 0;

  if (pHandler == NULL)
    return STDRIVE102P_SYS_ERROR;	
        
  if (((uint8_t)afe_par.comp_pos_input > 1) || ((uint8_t)afe_par.comp_neg_input > 1) || 
      ((uint8_t)afe_par.pga_gain > 3))
    return STDRIVE102P_CONFIG_DATA_ERROR;
    
  if (((uint8_t)afe_par.comp_deglitch > 3) || ((uint8_t)afe_par.comp_disable > 15))
    return STDRIVE102P_CONFIG_DATA_ERROR;
    
  error_code |= STDRIVE102P_Unlock_Cmd (pHandler);
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0F, &comp_cfg_reg, &status1);
  
  if(afe_par.pga_enable) afe_cfg_reg |= 0x80;
  if(afe_par.comp_enable) afe_cfg_reg |= 0x40;
  if(afe_par.comp_inversion) afe_cfg_reg |= 0x04;
  
  afe_cfg_reg |= (((uint8_t)afe_par.comp_pos_input << 4) & 0x10);
  afe_cfg_reg |= (((uint8_t)afe_par.comp_neg_input << 3) & 0x08); 
  afe_cfg_reg |= ((uint8_t)afe_par.pga_gain & 0x03);
  
  
  if(afe_par.comp_fault_enable) afe_flt_reg |= 0x80;
  if(afe_par.comp_counter_enable) afe_flt_reg |= 0x40;
  
  afe_flt_reg |= (((uint8_t)afe_par.comp_deglitch << 4) & 0x30);
  afe_flt_reg |= ((uint8_t)afe_par.comp_disable & 0x0F);
  	
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x0C, afe_cfg_reg, &status1, &status2);
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x10, afe_flt_reg, &status1, &status2);
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x13, afe_par.comp_counter, &status1, &status2);
  
  if (afe_par.comp_soft_off)
  {
    error_code |= STDRIVE102P_WriteReg (pHandler, 0x0F, (comp_cfg_reg | 0x01), &status1, &status2);	
  }
  else
  {
    error_code |= STDRIVE102P_WriteReg (pHandler, 0x0F, (comp_cfg_reg & 0xFE), &status1, &status2);
  }
  
  error_code |= STDRIVE102P_Lock_Cmd (pHandler);

  return (error_code); 
}

/**
  * @brief  It get the configuration of the Analog Front-End of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] afe_par The configuration of the selected channel is stored in the afe_par structure.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetAFEchannel (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEchannel_t *afe_par)
{
  uint8_t afe_cfg_reg = 0;
  uint8_t afe_flt_reg = 0;
  uint8_t comp_cfg_reg = 0;
  uint8_t afe_counter_reg = 0;
  uint8_t status1;
  uint16_t error_code = 0;
  
  if ((pHandler == NULL) | (afe_par == NULL)) return STDRIVE102P_SYS_ERROR;
  
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0C, &afe_cfg_reg, &status1);
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x10, &afe_flt_reg, &status1);
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x13, &afe_counter_reg, &status1);
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0F, &comp_cfg_reg, &status1);
  
  afe_par->pga_enable = ((afe_cfg_reg & 0x80) != 0) ? true : false;
  afe_par->comp_enable = ((afe_cfg_reg & 0x40) != 0) ? true : false;
  
  afe_par->comp_pos_input = (stdrv102p_inp_config_t)((afe_cfg_reg >> 4) & 0x01);
  afe_par->comp_neg_input = (stdrv102p_inn_config_t)((afe_cfg_reg >> 3) & 0x01);
  
  afe_par->comp_inversion = ((afe_cfg_reg & 0x04) != 0) ? true : false;
  afe_par->pga_gain = (stdrv102p_pga_gain_t)(afe_cfg_reg & 0x03);
  
  afe_par->comp_fault_enable = ((afe_flt_reg & 0x80) != 0) ? true : false;
  afe_par->comp_counter_enable = ((afe_flt_reg & 0x40) != 0) ? true : false;
  
  afe_par->comp_deglitch = (stdrv102p_compDeglitch_t)((afe_flt_reg >> 4) & 0x03);
  afe_par->comp_disable = (stdrv102p_compDisTime_t)(afe_flt_reg & 0x0F);
  
  afe_par->comp_counter = afe_counter_reg;
  
  afe_par->comp_soft_off = ((comp_cfg_reg & 0x01) != 0) ? true : false;
  
  return (error_code); 
}

/**
  * @brief  It seletcs the additional configurations of the Analog Front-End of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] afe_par Structure with the additional configuration parameters of the AFE.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_SetAFEcommon (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEcommon_t afe_par)
{
  uint8_t afe_main_cfg_reg = 0;
  uint8_t comp_cfg_reg = 0;
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code = 0;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
  
  if (((uint8_t)afe_par.comp_internal_th > 10) || ((uint8_t)afe_par.driver_safe_state > 3) || 
      ((uint8_t)afe_par.driver_safe_state == 1))
    return STDRIVE102P_CONFIG_DATA_ERROR;
  
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0B, &afe_main_cfg_reg, &status1);
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0F, &comp_cfg_reg, &status1);      
  
  afe_main_cfg_reg &= 0xE0;
        
  if (afe_par.pga_ref_sel == P_PGA_HALF_VDD)
  {
    afe_main_cfg_reg |= 0x10;
  }
  else if (afe_par.pga_ref_sel == P_PGA_NO_OFFSET)
  {
    afe_main_cfg_reg &= 0xEF;
  }
  else
  {
    return STDRIVE102P_CONFIG_DATA_ERROR;
  }
          
  afe_main_cfg_reg |= ((uint8_t)afe_par.comp_internal_th & 0x0F);
  
  comp_cfg_reg &= 0xE7;
  comp_cfg_reg |= (((uint8_t)afe_par.driver_safe_state << 3) & 0x18);
  
  error_code |= STDRIVE102P_Unlock_Cmd (pHandler);	
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x0B, afe_main_cfg_reg, &status1, &status2);
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x0F, comp_cfg_reg, &status1, &status2);	
  error_code |= STDRIVE102P_Lock_Cmd (pHandler);
  
  return (error_code);
}

/**
  * @brief  It get the additional configuration of the Analog Front-End of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] afe_par The additional configuration parameters of the AFE are stored in this structure.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetAFEcommon (STDRIVE102P_Handle_t *pHandler, STDRIVE102P_AFEcommon_t *afe_par)
{	
  uint8_t afe_main_cfg_reg = 0;
  uint8_t comp_cfg_reg = 0;
  uint8_t status1;
  uint16_t error_code = 0;

  if ((pHandler == NULL) || (afe_par == NULL)) return STDRIVE102P_SYS_ERROR;

  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0B, &afe_main_cfg_reg, &status1);
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0F, &comp_cfg_reg, &status1);

  if ((afe_main_cfg_reg & 0x10) == 0) afe_par->pga_ref_sel = P_PGA_NO_OFFSET;
  else afe_par->pga_ref_sel = P_PGA_HALF_VDD;

  afe_par->comp_internal_th = (stdrv102p_compInternalThreshold_t)(afe_main_cfg_reg & 0x0F);

  afe_par->driver_safe_state = (stdrv102p_safeStateSel_t)((comp_cfg_reg >> 3) & 0x03);

  if (((afe_main_cfg_reg & 0x0F) > 10) || (((comp_cfg_reg >> 3) & 0x03) == 1))
    error_code |= STDRIVE102P_CONFIG_DATA_ERROR;

  return (error_code);	
}

/**
  * @brief  It enables the Analog Front-End of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_AFE_Enable (STDRIVE102P_Handle_t *pHandler)
{
  uint8_t status1;
  uint8_t status2;
  uint8_t temp_reg_val = 0;
  uint16_t error_code = 0;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
  
  error_code |= STDRIVE102P_Unlock_Cmd (pHandler);	
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0B, &temp_reg_val, &status1);
  temp_reg_val |= 0x80;
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x0B, temp_reg_val, &status1, &status2);
  error_code |= STDRIVE102P_Lock_Cmd (pHandler);
	
  return (error_code);
}

/**
  * @brief  It disables the Analog Front-End of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_AFE_Disable (STDRIVE102P_Handle_t *pHandler)
{
  uint8_t status1;
  uint8_t status2;
  uint8_t temp_reg_val = 0;
  uint16_t error_code = 0;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
  
  error_code |= STDRIVE102P_Unlock_Cmd (pHandler);	
  error_code |= STDRIVE102P_ReadReg (pHandler, 0x0B, &temp_reg_val, &status1);
  temp_reg_val &= 0x7F;
  error_code |= STDRIVE102P_WriteReg (pHandler, 0x0B, temp_reg_val, &status1, &status2);
  error_code |= STDRIVE102P_Lock_Cmd (pHandler);
	
  return (error_code);
}

/**
  * @brief  It sets one of the event counters present in the STDRIVE102 protections.
  * @param  [in] hdl Driver handler.
  * @param  [in] cnt Specifies which counter (VDS monitoring or AFE) must be set.
  * @param  [in] value Specifies the number of counts to be written in the selected counter.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_SetCounter (STDRIVE102P_Handle_t *pHandler, stdrv102p_counter_index_t cnt, uint8_t value)
{
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code = 0;
	
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
        
  if ((cnt == P_VDS_COUNTER) || (cnt == P_AFE_CH1_COUNTER))
  {
    error_code = STDRIVE102P_WriteReg (pHandler, (uint8_t)cnt, value , &status1, &status2);
  }
  else
  {
    error_code = STDRIVE102P_CONFIG_DATA_ERROR;
  }       
  return (error_code);  
}

/**
  * @brief  It read the current counts remaining in one of the event counters of the STDRIVE102 protections.
  * @param  [in] hdl Driver handler.
  * @param  [in] cnt Specifies which counter (VDS monitoring or AFE) must be read.
  * @param  [out] value Current number of counts read from the selected counter.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetCounter (STDRIVE102P_Handle_t *pHandler, stdrv102p_counter_index_t cnt, uint8_t *value)
{
  uint8_t status1;
  uint16_t error_code = 0;
	
  if ((pHandler == NULL) || (value == NULL)) return STDRIVE102P_SYS_ERROR;
        
  if ((cnt == P_VDS_COUNTER) || (cnt == P_AFE_CH1_COUNTER))
  {
    error_code = STDRIVE102P_ReadReg (pHandler, (uint8_t)cnt, value, &status1);
  }
  else
  {
    error_code = STDRIVE102P_CONFIG_DATA_ERROR;
  }  
  return (error_code);
}

/**
  * @brief  It set which are the faults events to be reported on the nFAULT pin of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [in] val Each bit in this value enables a different event on the nFAULT pin.
  *	    Use a combination of the defines STDRIVE102P_FAULT_SIG_EN_x to select 
  *         which are the signals to be reported on the nFAULT pin. 
  *         Find the defines in the first part of the "stdrive102p_driver.h".
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_SetFAULTpinSignal (STDRIVE102P_Handle_t *pHandler, uint8_t val)
{
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_WriteReg (pHandler, 0x08, val, &status1, &status2);
  
  return (error_code);
}

/**
  * @brief  It retrieve which are the faults events reported on the nFAULT pin of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] val Each bit set to 1 corresponds to a specific event enabled on the nFAULT pin.
  *         The bits are organized according to the STDRIVE102P_FAULT_SIG_EN_x defined 
  *         in the first part of the "stdrive102p_driver.h".
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetFAULTpinSignal (STDRIVE102P_Handle_t *pHandler, uint8_t *val)
{
  uint8_t status1;
  uint16_t error_code;
  
  if ((pHandler == NULL) || (val == NULL)) return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_ReadReg (pHandler, 0x08, val, &status1);
  
  return (error_code);
}

/**
  * @brief  It gets the STATUS2 and STATUS1 registers of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] status1 Value of the STATUS1 register. Use STDRIVE102P_STATUS1_x 
  *         defined in the first part of the "stdrive102p_driver.h" to decode the meaning of each bit.
  * @param  [out] status2 Value of the STATUS2 register. Use STDRIVE102P_STATUS2_x 
  *         defined in the first part of the "stdrive102p_driver.h" to decode the meaning of each bit.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetStatus2 (STDRIVE102P_Handle_t *pHandler, uint8_t *status1, uint8_t *status2)
{
  uint16_t error_code;
  
  if ((pHandler == NULL) || (status1 == NULL) || (status2 == NULL))
    return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_ReadReg (pHandler, 0x01, status2, status1);
  
  return (error_code);
}

/**
  * @brief  It gets the STATUS3 and STATUS1 registers of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] status3 Value of the STATUS3 register. Use STDRIVE102P_STATUS3_x 
  *         defined in the first part of the "stdrive102p_driver.h" to decode the meaning of each bit.
  * @param  [out] status1 Value of the STATUS1 register. Use STDRIVE102P_STATUS1_x 
  *         defined in the first part of the "stdrive102p_driver.h" to decode the meaning of each bit.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetStatus3 (STDRIVE102P_Handle_t *pHandler, uint8_t *status3, uint8_t *status1)
{
  uint16_t error_code;
  
  if ((pHandler == NULL) || (status1 == NULL) || (status3 == NULL))
    return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_ReadReg (pHandler, 0x02, status3, status1);
  
  return (error_code);
}

/**
  * @brief  It gets the AFE_STATUS and STATUS1 registers of the STDRIVE102.
  * @param  [in] hdl Driver handler.
  * @param  [out] afe_status Value of the AFE_STATUS register. Use STDRIVE102P_AFESTATUS_x 
  *         defined in the first part of the "stdrive102p_driver.h" to decode the meaning of each bit.
  * @param  [out] status1 Value of the STATUS1 register. Use STDRIVE102P_STATUS1_x 
  *         defined in the first part of the "stdrive102p_driver.h" to decode the meaning of each bit.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_GetAFEStatus (STDRIVE102P_Handle_t *pHandler, uint8_t *afe_status, uint8_t *status1)
{
  uint16_t error_code;
  
  if ((pHandler == NULL) || (status1 == NULL) || (afe_status == NULL))
    return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_ReadReg (pHandler, 0x03, afe_status, status1);
  
  return (error_code);
}

/**
  * @brief  It sends commands to the STDRIVE102 to clear the different fault conditions.
  * @param  [in] hdl Driver handler.
  * @param  [in] cmd Selects which is the fault condition to be cleared.
  * This parameter can be one of the following defined commands codes:
  *     @arg STDRIVE102P_CMD_CLEAR_STATUS_L: resets the latched bits in the STDRIVE102 status registers.
  *     @arg STDRIVE102P_CMD_CLEAR_HW_FAULTS: releases the hardware latch of the STDRIVE102 (gate drivers operative).
  *     @arg STDRIVE102P_CMD_CLEAR_ALL: resets the hardware latch and clear the status bits.
  *     @arg STDRIVE102P_CMD_CLEAR_RESET: clear the reset bit in the STATUS2 register.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_ClearFault_Cmd (STDRIVE102P_Handle_t *pHandler, uint8_t cmd)
{
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
  
  if ((cmd == STDRIVE102P_CMD_CLEAR_STATUS_L) || (cmd == STDRIVE102P_CMD_CLEAR_HW_FAULTS) ||
      (cmd == STDRIVE102P_CMD_CLEAR_ALL) || (cmd == STDRIVE102P_CMD_CLEAR_RESET))
  {
    error_code = STDRIVE102P_WriteReg (pHandler, 0x1A, cmd, &status1, &status2);
  }
  else
  {
    error_code = STDRIVE102P_CONFIG_DATA_ERROR;
  }
  return (error_code);
}

/**
  * @brief  It resets the STDRIVE102. All registers are set to their default values.
  * @param  [in] hdl Driver handler.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_DevReset_Cmd (STDRIVE102P_Handle_t *pHandler)
{
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
          
  error_code = STDRIVE102P_Unlock_Cmd (pHandler);
  error_code = STDRIVE102P_WriteReg (pHandler, 0x1B, STDRIVE102P_CMD_GLOBAL_RESET, &status1, &status2);
          
  return (error_code);
}

/**
  * @brief  It locks the protected registers of the STDRIVE102: they cannot be written.
  * @param  [in] hdl Driver handler.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_Lock_Cmd (STDRIVE102P_Handle_t *pHandler)
{
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_WriteReg (pHandler, 0x1C, STDRIVE102P_CMD_LOCK_REGS, &status1, &status2);
  
  return (error_code);
}

/**
  * @brief  It unlocks the protected registers of the STDRIVE102 so they can be written.
  * @param  [in] hdl Driver handler.
  * @retval uint16_t Error code related to SPI communication and the STDRIVE102 state.
*/
uint16_t STDRIVE102P_Unlock_Cmd (STDRIVE102P_Handle_t *pHandler)
{
  uint8_t status1;
  uint8_t status2;
  uint16_t error_code;
  
  if (pHandler == NULL) return STDRIVE102P_SYS_ERROR;
  
  error_code = STDRIVE102P_WriteReg (pHandler, 0x1C, STDRIVE102P_CMD_UNLOCK_REGS, &status1, &status2);
  
  return (error_code);
}

/**
  * @brief  It initialize the STDRIVE102 handler and configure the registers of the STDRIVE102 according to the specified configuration.
  * @param  [in] pHandler Pointer to the driver handler used by the firmware library.
  * @param  [in] params Pointer to the structure containing all the default parameters for the STDRIVE102 initialization.
*/
__weak void STDRIVE102P_Init( STDRIVE102P_Handle_t *pHandler, const STDRIVE102P_defaultParams_t *params)
{
  if ((pHandler == NULL) || (params == NULL)) return;
	
  pHandler->defaultParams = params;
        
  STDRIVE102P_AFE_Disable (pHandler);// disable AFE
  STDRIVE102P_ClearFault_Cmd (pHandler, STDRIVE102P_CMD_CLEAR_STATUS_L);
  STDRIVE102P_ClearFault_Cmd (pHandler, STDRIVE102P_CMD_CLEAR_RESET);// reset command
  STDRIVE102P_SetGateDrivers (pHandler, pHandler->defaultParams->gd);// set gate drivers
  STDRIVE102P_SetUvlo (pHandler, pHandler->defaultParams->uv);
  STDRIVE102P_SetVDSmonitor (pHandler, pHandler->defaultParams->vdsm); // set vds monitor
  STDRIVE102P_SetAFEchannel (pHandler, pHandler->defaultParams->afe_ch); // set AFE CH
  STDRIVE102P_SetAFEcommon (pHandler, pHandler->defaultParams->afe_cm); // set AFE common
  STDRIVE102P_SetFAULTpinSignal (pHandler, STDRIVE102P_FAULT_SIG_EN_ALL);
  STDRIVE102P_AFE_Enable (pHandler);
	
  return;
}

/**
  * @brief  It performs the fault management routine, when a fault occurs in the motor control application.
  * @param  [in] pHandler Pointer to the STDRIVE102 driver handler.
*/
__weak void STDRIVE102P_FaultManagement( STDRIVE102P_Handle_t *pHandler)
{
  return;
}

/**
  * @brief  It performs the start management routine, as soon as the motor is started in the motor control application.
  * @param  [in] pHandler Pointer to the STDRIVE102 driver handler.
*/
__weak void STDRIVE102P_StartManagement( STDRIVE102P_Handle_t *pHandler)
{
  // basic implementation: every time the motor starts
  // check the dev reset bit; if not at 0, it means the device is in reset state
  // so the function rewrites all the registers to configure the STDRIVE102
  
  uint8_t status1; 
  uint8_t status2;
  
  if (pHandler == NULL)
    return;
  
  STDRIVE102P_GetStatus2 (pHandler, &status1, &status2);
        
  if ((status2 & STDRIVE102P_STATUS2_RESET) == STDRIVE102P_STATUS2_RESET )
  {
    STDRIVE102P_AFE_Disable (pHandler);// disable AFE
    STDRIVE102P_ClearFault_Cmd (pHandler, STDRIVE102P_CMD_CLEAR_STATUS_L);
    STDRIVE102P_ClearFault_Cmd (pHandler, STDRIVE102P_CMD_CLEAR_RESET);// reset command
    STDRIVE102P_SetGateDrivers (pHandler, pHandler->defaultParams->gd);// set gate drivers
    STDRIVE102P_SetUvlo (pHandler, pHandler->defaultParams->uv);
    STDRIVE102P_SetVDSmonitor (pHandler, pHandler->defaultParams->vdsm); // set vds monitor
    STDRIVE102P_SetAFEchannel (pHandler, pHandler->defaultParams->afe_ch); // set AFE CH
    STDRIVE102P_SetAFEcommon (pHandler, pHandler->defaultParams->afe_cm); // set AFE common
    STDRIVE102P_SetFAULTpinSignal (pHandler, STDRIVE102P_FAULT_SIG_EN_ALL);
    STDRIVE102P_AFE_Enable (pHandler);
  }
  
  return;
}

/**
  * @brief  It performs the stop management routine, as soon as the motor is stopped in the motor control application.
  * @param  [in] pHandler Pointer to the STDRIVE102 driver handler.
*/
__weak void STDRIVE102P_StopManagement( STDRIVE102P_Handle_t *pHandler)
{
  return;
}

/**
  * @brief  It performs a management routine, during the MF task in the motor control application.
  * @param  [in] pHandler Pointer to the STDRIVE102 driver handler.
*/
__weak void STDRIVE102P_MF_Management( STDRIVE102P_Handle_t *pHandler)
{
  return;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/