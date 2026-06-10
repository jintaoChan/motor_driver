<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    regular_conversion_manager.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the regular_conversion_manager component of the Motor Control SDK:
  *           Register conversion 
  *           Execute regular conv directly from Temperature and VBus sensors
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
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "regular_conversion_manager.h"
#include "mc_config.h"

/** @addtogroup MCSDK
  * @{
  */


/** @addtogroup COMMON_MC
  * @{
  */


/** @defgroup RCM Regular Conversion Manager 
  * @brief Regular Conversion Manager component of the Motor Control SDK
  *
  * MotorControl SDK makes an extensive usage of ADCs. Some conversions are timing critical
  * like current reading, and some have less constraints. If an ADC offers both Injected and Regular,
  * channels, critical conversions will be systematically done on Injected channels, because they 
  * interrupt any ongoing regular conversion so as to be executed without delay.
  * Others conversions, mainly Bus voltage, and Temperature sensing are performed with regular channels.
  * If users wants to perform ADC conversions with an ADC already used by MC SDK, they must use regular
  * conversions. It is forbidden to use Injected channel on an ADC that is already in use for current reading.
  * As usera and MC-SDK may share ADC regular scheduler, this component intents to manage all the 
  * regular conversions.
  * 
  * If users wants to execute their own conversion, they first have to register it through the 
  * RCM_RegisterRegConv() API. Multiple conversions can be registered in the RCM array (up to #RCM_MAX_CONV), 
  * which is processed in a circular way but only one can be scheduled at a time.
  *
  * User regular conversion, as well as Vbus and temperature, are executed by the high frequency task. Each high 
  * frequency task executes one of the conversion registered in the RCM array.
  *
  * To retrieve the result of a conversion the user must use  RCM_GetRegularConv() API.
  *
  * Example: of conversion registration:
  * 
  * RegConv_t UserConv =
  * {
  *  .regADC                = ADC1,
  *  .channel               = LL_ADC_CHANNEL_0,
  *  .samplingTime          = LL_ADC_SAMPLINGTIME_6CYCLES_5,
  * }; 
  * 
  * result = RCM_RegisterRegConv(&UserConv);
  *
  * Example of conversion reading:
  *
  * result = RCM_GetRegularConv(&UserConv);
  *
  * @note: conversions must registered before the enabling of the Irq that triggers the high frequency task 
  *        or speed timer task 
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/


/* Private defines -----------------------------------------------------------*/
/**
  * @brief Number of regular conversion allowed By default.
  *  
  * In single drive configuration, it is defined to 4. 2 of them are consumed by 
  * Bus voltage and temperature reading. This leaves 2 handles available for 
  * user conversions
  *
  * In dual drives configuration, it is defined to 6. 2 of them are consumed by 
  * Bus voltage and temperature reading for each motor. This leaves 2 handles 
  * available for user conversion.
  *
  <#if MC.DRIVE_NUMBER == "1">
  * Defined to 4 here. 
  <#else><#-- MC.DRIVE_NUMBER != 1 -->
  * Defined to 6 here. 
  </#if><#-- MC.DRIVE_NUMBER == 1 -->
  */
#define RCM_MAX_CONV <#if MC.DRIVE_NUMBER == "1" > 4U <#else> 6U </#if>


/* Global variables ----------------------------------------------------------*/

static RegConv_t *RCM_handle_array[RCM_MAX_CONV];
static uint8_t RCM_array_index = 0U; /*!< handled by RCM to point on the element for conversion. */
static uint8_t RCM_conversion_nb = 0U; /*!< total number of valid element in the array */

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Registers a regular conversion.
  * 
  * This function registers a regular ADC conversion that can be later scheduled for execution. It
  * returns the status of the registration.
  *
  * The registration may fail if there is no space left for additional conversions. The 
  * maximum number of regular conversion that can be registered is defined by #RCM_MAX_CONV.
  *
  * @note if HSO is used as sensor-less algortihm, the registration shall be done with an ADC
  *       not used for phase current and phase voltage sensing.     
  *
  * @param  regConv Pointer to the regular conversion parameters. 
  *         Contains ADC, Channel and sampling time to be used.
  *
  * @retval bool true if the conversion is registered correctly, false otherwise still ongoing. 
  *
  */
bool RCM_RegisterRegConv(RegConv_t *regConv)
{
  bool retVal = true;
#ifdef NULL_PTR_CHECK_REG_CON_MNG 
  if (MC_NULL == regConv)
  {
    retVal = false;
  }
  else
  {
#endif
    
    if (RCM_conversion_nb < RCM_MAX_CONV)
    {
      RCM_handle_array[RCM_conversion_nb] = regConv;
      RCM_handle_array[RCM_conversion_nb]->id = RCM_conversion_nb;
      RCM_conversion_nb++;
      
      if (0U == LL_ADC_IsEnabled(regConv->regADC))
      {
<#if CondFamily_STM32F0>
<#-- useless as there is only one ADC -->
        <#elseif  CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G4 || CondFamily_STM32H5>
        LL_ADC_DisableIT_EOC(regConv->regADC);
        LL_ADC_ClearFlag_EOC(regConv->regADC);
        LL_ADC_DisableIT_JEOC(regConv->regADC);
        LL_ADC_ClearFlag_JEOC(regConv->regADC);
        <#elseif  CondFamily_STM32F4 || CondFamily_STM32F7>
        LL_ADC_DisableIT_EOCS(regConv->regADC);
        LL_ADC_ClearFlag_EOCS(regConv->regADC);
        LL_ADC_DisableIT_JEOS(regConv->regADC);
        LL_ADC_ClearFlag_JEOS(regConv->regADC);
</#if><#-- CondFamily_STM32F0 -->

<#if !CondFamily_STM32F4 && !CondFamily_STM32L4 && !CondFamily_STM32F7>
  <#if CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0>
        LL_ADC_StartCalibration( regConv->regADC);
  <#elseif CondFamily_STM32H7>
        LL_ADC_StartCalibration(regConv->regADC, LL_ADC_CALIB_OFFSET_LINEARITY, LL_ADC_SINGLE_ENDED);
  <#else><#-- CondFamily_STM32F0 == false || CondFamily_STM32G0 == false || CondFamily_STM32C0 == false && CondFamily_STM32H7 == false -->
        LL_ADC_StartCalibration(regConv->regADC, LL_ADC_SINGLE_ENDED);
  </#if><#-- CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0 -->
        while (1U == LL_ADC_IsCalibrationOnGoing(regConv->regADC))  
        {
          /* Nothing to do */
        }
</#if><#-- !CondFamily_STM32F4 && !CondFamily_STM32L4 && !CondFamily_STM32F7 -->
<#if CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
        <#-- This is done only for G4 because clock ratio 1/10 flag this issue -->
        /* ADC Enable (must be done after calibration) */
        /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0 
        * following a calibration phase, could have no effect on ADC 
        * within certain AHB/ADC clock ratio
        */
        while (0U == LL_ADC_IsActiveFlag_ADRDY(regConv->regADC))  
        { 
          LL_ADC_Enable(regConv->regADC);
        }

<#else><#-- CondFamily_STM32G4 == false -->
        LL_ADC_Enable(regConv->regADC);
</#if><#-- CondFamily_STM32G4 -->
      }
      else 
      {
        /* Nothing to do */
      }
<#if NoInjectedChannel>
  <#if CondFamily_STM32G0 || CondFamily_STM32C0><#-- MCU using 2 common sampling groups ; should be removed after CubeMx 6.12, see ticket 179556 -->
      LL_ADC_SetChannelSamplingTime(regConv->regADC, __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel), LL_ADC_SAMPLINGTIME_COMMON_2);
  </#if><#-- CondFamily_STM32G0 || CondFamily_STM32C0 -->
<#else>
      LL_ADC_REG_SetSequencerLength(regConv->regADC, LL_ADC_REG_SEQ_SCAN_DISABLE);
      /* Configure the sampling time (should already be configured by for non user conversions) */
      LL_ADC_SetChannelSamplingTime (regConv->regADC, __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel),
                                     regConv->samplingTime);  
</#if><#-- NoInjectedChannel -->
    }
    else
    {
      retVal = false;
    }
#ifdef NULL_PTR_CHECK_REG_CON_MNG
  }
#endif
  return retVal;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/*
 * Starts the next scheduled regular conversion
 *
 * This function does not poll on ADC read and is foreseen to be used inside
 * high frequency task where ADC are shared between currents reading
 * and user conversion.
 *
 * @note: This function is not part of the public API and users should not call it. 
 */
void RCM_ExecNextConv(void)
{
  if (RCM_conversion_nb > 0u)
  {
<#if CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0>
    LL_ADC_REG_SetDMATransfer(RCM_handle_array[RCM_array_index]->regADC, LL_ADC_REG_DMA_TRANSFER_NONE);
  
    /* ADC STOP condition requested to write CHSELR is true because of the ADCSTOP is set by hardware
       at the end of A/D conversion if the external Trigger of ADC is disabled */
  
    /* By default it is ADSTART = 0, then at the first time the CFGR1 can be written */
  
    /* Disabling External Trigger of ADC */
    LL_ADC_REG_SetTriggerSource(RCM_handle_array[RCM_array_index]->regADC, LL_ADC_REG_TRIG_SOFTWARE);
  
    /* Set Sampling time and channel */
    <#if CondFamily_STM32G0 || CondFamily_STM32C0>
    LL_ADC_SetSamplingTimeCommonChannels(RCM_handle_array[RCM_array_index]->regADC, LL_ADC_SAMPLINGTIME_COMMON_2,
                                         RCM_handle_array[RCM_array_index]->samplingTime);
    <#else><#-- CondFamily_STM32G0 == false && CondFamily_STM32G0 == false -->
    LL_ADC_SetSamplingTimeCommonChannels(RCM_handle_array[RCM_array_index]->regADC, RCM_handle_array[RCM_array_index]->samplingTime);
    </#if><#-- CondFamily_STM32G0 -->
    LL_ADC_REG_SetSequencerChannels(RCM_handle_array[RCM_array_index]->regADC,
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(RCM_handle_array[RCM_array_index]->channel));

    /* Clear EOC */
    LL_ADC_ClearFlag_EOC(RCM_handle_array[RCM_array_index]->regADC);

    /* Start ADC conversion */
    LL_ADC_REG_StartConversion(RCM_handle_array[RCM_array_index]->regADC);

<#else><#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->

    LL_ADC_REG_SetSequencerRanks(RCM_handle_array[RCM_array_index]->regADC,
                                 LL_ADC_REG_RANK_1,
                                 __LL_ADC_DECIMAL_NB_TO_CHANNEL(RCM_handle_array[RCM_array_index]->channel));

    (void)LL_ADC_REG_ReadConversionData12L(RCM_handle_array[RCM_array_index]->regADC);


  <#if CondFamily_STM32F4><#-- F4 requires explicitly bitbanding access otherwise dual drive 1 shunt fails -->
    LL_ADC_ClearFlag_EOCS(RCM_handle_array[RCM_array_index]->regADC);
    /* Bit banding access equivalent to LL_ADC_REG_StartConversionSWStart */
    BB_REG_BIT_SET(&RCM_handle_array[RCM_array_index]->regADC->CR2, ADC_CR2_SWSTART_Pos);
  <#elseif CondFamily_STM32F7>
    LL_ADC_ClearFlag_EOCS(RCM_handle_array[RCM_array_index]->regADC);
    LL_ADC_REG_StartConversionSWStart (RCM_handle_array[RCM_array_index]->regADC);
  <#else><#--  CondFamily_STM32F4 == false &&  CondFamily_STM32F7 == false -->
    LL_ADC_ClearFlag_EOC(RCM_handle_array[RCM_array_index]->regADC);
    LL_ADC_REG_StartConversion(RCM_handle_array[RCM_array_index]->regADC);
  </#if><#-- CondFamily_STM32F4 -->
</#if> <#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->
  }
  else
  {
     /* no conversion registered */
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/*
 * Reads the result of the ongoing regular conversion
 *
 * This function is foreseen to be used inside
 * high frequency task where ADC are shared between current reading
 * and user conversion.
 *
 * @note: This function is not part of the public API and users should not call it. 
 */
void RCM_ReadOngoingConv(void)
{
<#if !NoInjectedChannel>
  uint32_t result;
</#if>

  if (RCM_conversion_nb > 0u)
  {
<#if NoInjectedChannel>
     while (LL_ADC_IsActiveFlag_EOC(RCM_handle_array[RCM_array_index]->regADC) == 0U )
     {
       /* wait for end of conversion */
     }
<#else>
  <#if CondFamily_STM32F4 || CondFamily_STM32F7>
    result = LL_ADC_IsActiveFlag_EOCS(RCM_handle_array[RCM_array_index]->regADC);
  <#else>
    result = LL_ADC_IsActiveFlag_EOC(RCM_handle_array[RCM_array_index]->regADC);
  </#if>
    if ( 0U == result )
    {
      /* Nothing to do */
    }
    else
  </#if> 
    {
      /* Reading of ADC Converted Value */
      RCM_handle_array[RCM_array_index]->data
                    = LL_ADC_REG_ReadConversionData12L(RCM_handle_array[RCM_array_index]->regADC);
    <#if CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0>
      /* Restore back DMA configuration */
      LL_ADC_REG_SetDMATransfer( RCM_handle_array[RCM_array_index]->regADC, LL_ADC_REG_DMA_TRANSFER_LIMITED );
    </#if><#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->
    }
  
    /* Prepare next conversion */
    if (RCM_array_index == (RCM_conversion_nb - 1U))
    {
      RCM_array_index = 0U;
    }
    else
    {
      RCM_array_index++;
    }
  }
  else
  {
     /* no conversion registered */
  }
}

/*
 * This function is used to exeute a regular conversion.
 * This function polls on the ADC end of conversion.
 * If the ADC is already in use for phase currents or phase voltage sensing, the regular conversion can not
 * be executed instantaneously, therefore this function shall not be used.
 * If it is possible to execute the conversion instantaneously, it will be executed, and result returned.
 *
 * @note: This function is not part of the public API and users should not call it.
 */
uint16_t RCM_ExecRegularConv(RegConv_t *regConv)
{
  uint16_t result;  
#ifdef NULL_PTR_CHECK_REG_CON_MNG
  if (MC_NULL == regConv)
  {
    result = 0U;
  }
  else
  {
#endif
<#if CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0>
  LL_ADC_REG_SetDMATransfer(regConv->regADC, LL_ADC_REG_DMA_TRANSFER_NONE);
  
  /* ADC STOP condition requested to write CHSELR is true because of the ADCSTOP is set by hardware
     at the end of A/D conversion if the external Trigger of ADC is disabled */
  
  /* By default it is ADSTART = 0, then at the first time the CFGR1 can be written */
  
  /* Disabling External Trigger of ADC */
  LL_ADC_REG_SetTriggerSource(regConv->regADC, LL_ADC_REG_TRIG_SOFTWARE);
  
  /* Set Sampling time and channel */
  <#if CondFamily_STM32G0 || CondFamily_STM32C0>
  LL_ADC_SetSamplingTimeCommonChannels(regConv->regADC, LL_ADC_SAMPLINGTIME_COMMON_2,
                                       regConv->samplingTime);
  <#else><#-- CondFamily_STM32G0 == false && CondFamily_STM32G0 == false -->
  LL_ADC_SetSamplingTimeCommonChannels(regConv->regADC, regConv->samplingTime);
  </#if><#-- CondFamily_STM32G0 -->
  LL_ADC_REG_SetSequencerChannels(regConv->regADC,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel));
  
  /* Clear EOC */
  LL_ADC_ClearFlag_EOC(regConv->regADC);
  
  /* Start ADC conversion */
  LL_ADC_REG_StartConversion(regConv->regADC);

<#else><#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->

  LL_ADC_REG_SetSequencerRanks(regConv->regADC,
                               LL_ADC_REG_RANK_1,
                               __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel));

  (void)LL_ADC_REG_ReadConversionData12L(regConv->regADC);


  <#if CondFamily_STM32F4><#-- F4 requires explicitly bitbanding access otherwise dual drive 1 shunt fails -->
  LL_ADC_ClearFlag_EOCS(regConv->regADC);
  /* Bit banding access equivalent to LL_ADC_REG_StartConversionSWStart */
  BB_REG_BIT_SET(&regConv->regADC->CR2, ADC_CR2_SWSTART_Pos);
  <#elseif CondFamily_STM32F7>
  LL_ADC_ClearFlag_EOCS(regConv->regADC);
  LL_ADC_REG_StartConversionSWStart (regConv->regADC);
  <#else><#--  CondFamily_STM32F4 == false &&  CondFamily_STM32F7 == false -->
  LL_ADC_ClearFlag_EOC(regConv->regADC);
  LL_ADC_REG_StartConversion(regConv->regADC);
  </#if><#-- CondFamily_STM32F4 -->
</#if> <#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->
  

  <#if CondFamily_STM32F4 || CondFamily_STM32F7>
  while (LL_ADC_IsActiveFlag_EOCS(regConv->regADC) == 0U )
  <#else>
  while (LL_ADC_IsActiveFlag_EOC(regConv->regADC) == 0U )
  </#if> <#-- CondFamily_STM32F4 || CondFamily_STM32F7 -->
  {
    /* wait for end of conversion */
  }

  /* Reading of ADC Converted Value */
  result = LL_ADC_REG_ReadConversionData12L(regConv->regADC);
    <#if CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0>
  /* Restore back DMA configuration */
  LL_ADC_REG_SetDMATransfer( regConv->regADC, LL_ADC_REG_DMA_TRANSFER_LIMITED );
    </#if><#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->
#ifdef NULL_PTR_CHECK_REG_CON_MNG
  }
#endif
  return result;

}

/* 
 * This function is used to wait for the result of a regular conversion. 
 * @note: This shall be used only right after a call to RCM_ExecNextConv routine.
 *
 */
void RCM_WaitForConv(void)
{
  if (RCM_conversion_nb > 0u)
  {
<#if CondFamily_STM32F4 || CondFamily_STM32F7>
    while (LL_ADC_IsActiveFlag_EOCS(RCM_handle_array[RCM_array_index]->regADC) == 0U )
<#else>
    while (LL_ADC_IsActiveFlag_EOC(RCM_handle_array[RCM_array_index]->regADC) == 0U )
</#if>  
    {
      /* wait for end of conversion */
    }
  }
  else
  {
     /* no conversion registered */
  }
}

<#if MC.M1_CURRENT_MONITOR_READING == true>
/*
 * Starts the next scheduled regular conversion for current sensing
 *
 * This function is foreseen to be used inside
 * high frequency task where ADC are shared between current reading
 * and user conversion.
 *
 * NOTE: This function is not part of the public API and users should not call it. 
 */
void RCM_ExecCurrentSense(CurrMonitor_t *currMon)
{
  <#if CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0>
  /* Set Sampling time and channel */
    <#if CondFamily_STM32G0 || CondFamily_STM32C0>
  LL_ADC_SetSamplingTimeCommonChannels(currMon->regADC, LL_ADC_SAMPLINGTIME_COMMON_2,
                                       currMon->samplingTime);
    <#else><#-- CondFamily_STM32G0 == false && CondFamily_STM32G0 == false -->
  LL_ADC_SetSamplingTimeCommonChannels(currMon->regADC, currMon->samplingTime);
    </#if><#-- CondFamily_STM32G0 -->
  LL_ADC_REG_SetSequencerChannels(currMon->regADC,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(currMon->channel));

  <#else><#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->
  LL_ADC_SetChannelSamplingTime(currMon->regADC, __LL_ADC_DECIMAL_NB_TO_CHANNEL(currMon->channel),
                                currMon->samplingTime);
  LL_ADC_REG_SetSequencerRanks(currMon->regADC,
                               LL_ADC_REG_RANK_1,
                               __LL_ADC_DECIMAL_NB_TO_CHANNEL(currMon->channel));

  </#if> <#-- CondFamily_STM32G0 || CondFamily_STM32C0 || CondFamily_STM32F0 -->

/* Enable ADC source trigge.r */
  <#if CondFamily_STM32G0 || CondFamily_STM32C0>
  LL_ADC_REG_SetTriggerSource(currMon->regADC, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
  <#else>
  LL_ADC_REG_SetTriggerSource(currMon->regADC, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  </#if><#-- CondFamily_STM32G0 || CondFamily_STM32C0 -->
  LL_ADC_REG_SetTriggerEdge(currMon->regADC, LL_ADC_REG_TRIG_EXT_FALLING);
  /* Clear EOC */
  LL_ADC_ClearFlag_EOC(currMon->regADC);
  LL_ADC_EnableIT_EOC(currMon->regADC);
  
  /* Start ADC for regular conversion. */
  LL_ADC_REG_StartConversion(currMon->regADC);
}

/*
 * Reads the result of the current conversion and stops the acquisition
 *
 * NOTE: This function is not part of the public API and users should not call it. 
 */
void RCM_ReadCurrentMonitor(CurrMonitor_t *currMon)
{
  uint32_t tReadValue  = currMon->currentConvFactor * LL_ADC_REG_ReadConversionData12L(currMon->regADC) ;
  currMon->currentMa = (uint16_t) (tReadValue / 65536u);
  while (LL_ADC_REG_IsConversionOngoing(currMon->regADC))
  {
    LL_ADC_REG_StopConversion(currMon->regADC);
    while (LL_ADC_REG_IsStopConversionOngoing(currMon->regADC));
  }
  LL_ADC_REG_SetTriggerSource(currMon->regADC, LL_ADC_REG_TRIG_SOFTWARE);
  LL_ADC_DisableIT_EOC(currMon->regADC);
  if (0xFFFFU == currMon->currentMa)
  {
    /* Nothing to do */
  }
  else
  {
    uint32_t wtemp;
    wtemp = (uint32_t)(currMon->hLowPassFilterBW) - 1U;
    wtemp *= ((uint32_t)currMon->hAvCurr_d);
    wtemp += currMon->currentMa;
    wtemp /= ((uint32_t)currMon->hLowPassFilterBW);
    
    currMon->hAvCurr_d = (uint16_t)wtemp;
  }
}
</#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/




