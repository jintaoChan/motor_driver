<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/sixstep_assign.ftl">
/**
  ******************************************************************************
  * @file    stm32g4xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32G4 Family.
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
  * @ingroup STM32G4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
//cstat -MISRAC2012-Rule-3.1
#include "mc_tasks.h"
//cstat +MISRAC2012-Rule-3.1
#include "parameters_conversion.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32G4xx_IRQ_Handlers STM32G4xx IRQ Handlers
  * @{
  */
  
/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
void BEMF_READING_IRQHandler(void);
void PERIOD_COMM_IRQHandler(void);
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
void TIMx_BRK_M1_IRQHandler(void);
<#if (M1_HALL_SENSOR == true)>
void SPD_TIM_M1_IRQHandler(void);
</#if><#-- (M1_HALL_SENSOR == true) -->

<#function _last_word text sep="_"><#return text?split(sep)?last></#function>
<#function _last_char text><#return text[text?length-1]></#function>

<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  <#if ADC1 || ADC2>
/**
  * @brief  This function handles BEMF sensing interrupt request from ADC1 and ADC2.
  * @param[in] None
  */
void BEMF_ADC1_2_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

    <#if ADC1>
  if (LL_ADC_IsActiveFlag_AWD1(ADC1) != 0U)
  {
    if (LL_ADC_IsEnabledIT_AWD1(ADC1) != 0U)
    {
      /* Clear Flags. */
      LL_ADC_ClearFlag_AWD1(ADC1);
      <#if DWT_CYCCNT_SUPPORTED>
        <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Start(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
        </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
      </#if><#-- DWT_CYCCNT_SUPPORTED -->
      <#if MC.M1_IPD_STARTUP == true>
      if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
      {
        IPD_M1.NumZeroSpeedSamples++;
        IPD_M1.IPDBEMFMeasured = true;
      }
      else
      {
      </#if><#-- MC.M1_IPD_STARTUP == true -->
      <#if MC.M1_OTF_STARTUP == true> 
      TSK_BEMF_ZCD_Task();
      <#else><#-- MC.M1_OTF_STARTUP = false -->
      BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
      </#if><#-- MC.M1_OTF_STARTUP == true -->
      <#if MC.M1_CURRENT_MONITOR_READING == true>
      if  ((RUN == Mci[M1].State) && (MCI_GetFaultState(&Mci[M1]) == (uint32_t)MC_NO_FAULTS))
      {
        uint16_t currentMonSamplPoint = (uint16_t) ((CurrMonitor_M1.samplingPointConvFact * CurrMonitor_M1.samplingDistance2Edge) /1000);
        if ( currentMonSamplPoint < SixStepVars[M1].DutyCycleRef)
        {
          PWMC_SetADCTriggerChannel(&PWM_Handle_M1, SixStepVars[M1].DutyCycleRef-currentMonSamplPoint);
          RCM_ExecCurrentSense(&CurrMonitor_M1);
        }
        else
        {
          /* Nothing to do */
        }        
      }
      else
      {
        /* Nothing to do */
      }
      </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
      <#if MC.M1_IPD_STARTUP == true>
      }
      </#if><#-- MC.M1_IPD_STARTUP == true -->
      <#if DWT_CYCCNT_SUPPORTED>
        <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Stop(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
        </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
      </#if><#-- DWT_CYCCNT_SUPPORTED -->
    }
    else
    {
      /* Nothing to do. */
    }
  }
  else
  {
    /* Nothing to do. */
  }
    </#if><#-- ADC1 -->

    <#if ADC2>
  if (LL_ADC_IsActiveFlag_AWD1(ADC2) != 0U)
  {
    if (LL_ADC_IsEnabledIT_AWD1(ADC2) != 0U)
    {
      /* Clear Flags. */
      LL_ADC_ClearFlag_AWD1(ADC2);
      <#if DWT_CYCCNT_SUPPORTED>
        <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Start(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
        </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
      </#if><#-- DWT_CYCCNT_SUPPORTED -->
      <#if MC.M1_IPD_STARTUP == true>
      if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
      {
        IPD_M1.NumZeroSpeedSamples++;
        IPD_M1.IPDBEMFMeasured = true;
      }
      else
      {
      </#if><#-- MC.M1_IPD_STARTUP == true -->
      <#if MC.M1_OTF_STARTUP == true> 
      TSK_BEMF_ZCD_Task();
      <#else><#-- MC.M1_OTF_STARTUP = false -->
      BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
      </#if><#-- MC.M1_OTF_STARTUP == true -->
      <#if MC.M1_CURRENT_MONITOR_READING == true>
      if  ((RUN == Mci[M1].State) && (MCI_GetFaultState(&Mci[M1]) == (uint32_t)MC_NO_FAULTS))
      {
        uint16_t currentMonSamplPoint = (uint16_t) ((CurrMonitor_M1.samplingPointConvFact * CurrMonitor_M1.samplingDistance2Edge) /1000);
        if ( currentMonSamplPoint < SixStepVars[M1].DutyCycleRef)
        {
          PWMC_SetADCTriggerChannel(&PWM_Handle_M1, SixStepVars[M1].DutyCycleRef-currentMonSamplPoint);
          RCM_ExecCurrentSense(&CurrMonitor_M1);
        }
        else
        {
          /* Nothing to do */
        }        
      }
      else
      {
        /* Nothing to do */
      }
      </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
     <#if MC.M1_IPD_STARTUP == true>
      }
      </#if><#-- MC.M1_IPD_STARTUP == true -->
      <#if DWT_CYCCNT_SUPPORTED>
        <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Stop(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
        </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
      </#if><#-- DWT_CYCCNT_SUPPORTED -->
    }
    else
    {
      /* Nothing to do. */
    }
  }
  else
  {
    /* Nothing to do. */
  }
    </#if><#-- ADC2 -->

  <#if MC.M1_IPD_STARTUP == true>
  if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
  {
    /* Nothing to do. */
  }
  else
  {
  </#if><#-- MC.M1_IPD_STARTUP == true -->
    <#if MC.M1_CURRENT_MONITOR_READING == true && (MC.M1_CUR_MON_ADC == "ADC1" ||  MC.M1_CUR_MON_ADC == "ADC2")>
  if(LL_ADC_IsActiveFlag_EOC(CurrMonitor_M1.regADC) && LL_ADC_IsEnabledIT_EOC(CurrMonitor_M1.regADC))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_EOC(CurrMonitor_M1.regADC);
    RCM_ReadCurrentMonitor(&CurrMonitor_M1);
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
    <#if MC.M1_IPD_STARTUP == true>
  }
    </#if><#-- MC.M1_IPD_STARTUP == true -->
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */

}
  </#if><#-- ADC1 || ADC2 -->

  <#if ADC3>
/**
  * @brief  This function handles BEMF sensing interrupt request from ADC3.
  * @param[in] None
  */
void BEMF_ADC3_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn  2 */

  if (LL_ADC_IsActiveFlag_AWD1(ADC3) != 0U)
  {
    if (LL_ADC_IsEnabledIT_AWD1(ADC3) != 0U)
    {
      /* Clear Flags. */
      LL_ADC_ClearFlag_AWD1(ADC3);
    <#if DWT_CYCCNT_SUPPORTED>
      <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Start(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
      </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
    </#if><#-- DWT_CYCCNT_SUPPORTED -->
    <#if MC.M1_IPD_STARTUP == true>
      if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
      {
        IPD_M1.NumZeroSpeedSamples++;
        IPD_M1.IPDBEMFMeasured = true;
      }
      else
      {
    </#if><#-- MC.M1_IPD_STARTUP == true -->
    <#if MC.M1_OTF_STARTUP == true> 
      TSK_BEMF_ZCD_Task();
    <#else><#-- MC.M1_OTF_STARTUP = false -->
      BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
    </#if><#-- MC.M1_OTF_STARTUP == true -->
    <#if MC.M1_CURRENT_MONITOR_READING == true>
      if  ((RUN == Mci[M1].State) && (MCI_GetFaultState(&Mci[M1]) == (uint32_t)MC_NO_FAULTS))
      {
        uint16_t currentMonSamplPoint = (uint16_t) ((CurrMonitor_M1.samplingPointConvFact * CurrMonitor_M1.samplingDistance2Edge) /1000);
        if ( currentMonSamplPoint < SixStepVars[M1].DutyCycleRef)
        {
          PWMC_SetADCTriggerChannel(&PWM_Handle_M1, SixStepVars[M1].DutyCycleRef-currentMonSamplPoint);
          RCM_ExecCurrentSense(&CurrMonitor_M1);
        }
        else
        {
          /* Nothing to do */
        }        
      }
      else
      {
        /* Nothing to do */
      }
    </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
    <#if MC.M1_IPD_STARTUP == true>
      }
    </#if><#-- MC.M1_IPD_STARTUP == true -->
    <#if DWT_CYCCNT_SUPPORTED>
      <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Stop(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
      </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
    </#if><#-- DWT_CYCCNT_SUPPORTED -->
    }
    else
    {
      /* Nothing to do. */
    }
  }
  else
  {
    /* Nothing to do. */
  }
  <#if MC.M1_IPD_STARTUP == true>
  if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
  {
    /* Nothing to do. */
  }
  else
  {
  </#if><#-- MC.M1_IPD_STARTUP == true -->

    <#if MC.M1_CURRENT_MONITOR_READING == true && MC.M1_CUR_MON_ADC == "ADC3">
  if(LL_ADC_IsActiveFlag_EOC(CurrMonitor_M1.regADC) && LL_ADC_IsEnabledIT_EOC(CurrMonitor_M1.regADC))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_EOC(CurrMonitor_M1.regADC);
    RCM_ReadCurrentMonitor(&CurrMonitor_M1);
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
    <#if MC.M1_IPD_STARTUP == true>
  }
    </#if><#-- MC.M1_IPD_STARTUP == true -->

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 3 */

  /* USER CODE END CURRENT_REGULATION_IRQn 3 */

}
  </#if><#-- ADC3 -->

  <#if ADC4>
/**
  * @brief  This function handles BEMF sensing interrupt request from ADC4.
  * @param[in] None
  */
void BEMF_ADC4_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 4 */

  /* USER CODE END CURRENT_REGULATION_IRQn 4 */

  if (LL_ADC_IsActiveFlag_AWD1(ADC4) != 0U)
  {
    if (LL_ADC_IsEnabledIT_AWD1(ADC4) != 0U)
    {
      /* Clear Flags. */
      LL_ADC_ClearFlag_AWD1(ADC4);
    <#if DWT_CYCCNT_SUPPORTED>
      <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Start(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
      </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
    </#if><#-- DWT_CYCCNT_SUPPORTED -->
    <#if MC.M1_IPD_STARTUP == true>
      if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
      {
        IPD_M1.NumZeroSpeedSamples++;
        IPD_M1.IPDBEMFMeasured = true;
      }
      else
      {
    </#if><#-- MC.M1_IPD_STARTUP == true -->
    <#if MC.M1_OTF_STARTUP == true> 
      TSK_BEMF_ZCD_Task();
    <#else><#-- MC.M1_OTF_STARTUP = false -->
      BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
    </#if><#-- MC.M1_OTF_STARTUP == true -->
    <#if MC.M1_CURRENT_MONITOR_READING == true>
      if  ((RUN == Mci[M1].State) && (MCI_GetFaultState(&Mci[M1]) == (uint32_t)MC_NO_FAULTS))
      {
        uint16_t currentMonSamplPoint = (uint16_t) ((CurrMonitor_M1.samplingPointConvFact * CurrMonitor_M1.samplingDistance2Edge) /1000);
        if ( currentMonSamplPoint < SixStepVars[M1].DutyCycleRef)
        {
          PWMC_SetADCTriggerChannel(&PWM_Handle_M1, SixStepVars[M1].DutyCycleRef-currentMonSamplPoint);
          RCM_ExecCurrentSense(&CurrMonitor_M1);
        }
        else
        {
          /* Nothing to do */
        }        
      }
      else
      {
        /* Nothing to do */
      }
    </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
    <#if MC.M1_IPD_STARTUP == true>
      }
    </#if><#-- MC.M1_IPD_STARTUP == true -->
    <#if DWT_CYCCNT_SUPPORTED>
      <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_Perf_Measure_Stop(&PerfTraces, (uint8_t)MEASURE_TSK_ADCTimerM1);
      </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
    </#if><#-- DWT_CYCCNT_SUPPORTED -->
    }
    else
    {
      /* Nothing to do. */
    }
  }
  else
  {
    /* Nothing to do. */
  }

  <#if MC.M1_IPD_STARTUP == true>
  if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
  {
    /* Nothing to do. */
  }
  else
  {
  </#if><#-- MC.M1_IPD_STARTUP == true -->
    <#if MC.M1_CURRENT_MONITOR_READING == true && MC.M1_CUR_MON_ADC == "ADC4">
  if(LL_ADC_IsActiveFlag_EOC(CurrMonitor_M1.regADC) && LL_ADC_IsEnabledIT_EOC(CurrMonitor_M1.regADC))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_EOC(CurrMonitor_M1.regADC);
    RCM_ReadCurrentMonitor(&CurrMonitor_M1);
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
    <#if MC.M1_IPD_STARTUP == true>
  }
    </#if><#-- MC.M1_IPD_STARTUP == true -->
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 5 */

  /* USER CODE END CURRENT_REGULATION_IRQn 5 */
}
  </#if><#-- ADC4 -->

/**
  * @brief     LFtimer interrupt handler.
  * @param[in] None
  */
void PERIOD_COMM_IRQHandler(void)
{
  /* TIM Capture compare 1 event. */
  <#if DWT_CYCCNT_SUPPORTED>
    <#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Start(&PerfTraces, (uint8_t)MEASURE_TSK_SpeedTimerM1);
    </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
  </#if><#-- DWT_CYCCNT_SUPPORTED -->

  if (LL_TIM_IsActiveFlag_CC1(${_last_word(MC.LF_TIMER_SELECTION)}) != 0U)
  {
    if (LL_TIM_IsEnabledIT_CC1(${_last_word(MC.LF_TIMER_SELECTION)}) != 0U)
    {
      LL_TIM_ClearFlag_CC1(${_last_word(MC.LF_TIMER_SELECTION)});
      TSK_SpeedTIM_task(); 
    }
    else
    {
      /* Nothing to do. */
    }
  }
  else
  {
    /* Nothing to do. */
  }

  <#if DWT_CYCCNT_SUPPORTED>
    <#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Stop(&PerfTraces, (uint8_t)MEASURE_TSK_SpeedTimerM1);
    </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
  </#if><#-- DWT_CYCCNT_SUPPORTED -->
}
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

<#if (M1_HALL_SENSOR == true)>
  <#if MC.M1_CURRENT_MONITOR_READING == true>
/**
  * @brief  This function handles current sensing interrupt request.
  * @param[in] None
  */
void CURRENT_SENSE_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  if(LL_ADC_IsActiveFlag_EOC(CurrMonitor_M1.regADC) && LL_ADC_IsEnabledIT_EOC(CurrMonitor_M1.regADC))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_EOC(CurrMonitor_M1.regADC);
    RCM_ReadCurrentMonitor(&CurrMonitor_M1);
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

  </#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */

  <#if DWT_CYCCNT_SUPPORTED>
    <#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Start(&PerfTraces, (uint8_t)MEASURE_TSK_SpeedTimerM1);
    </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
  </#if><#-- DWT_CYCCNT_SUPPORTED -->

  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state. */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx) != 0U)
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    (void)HALL_TIMx_UP_IRQHandler(&HALL_M1);

    /* USER CODE BEGIN M1 HALL_Update */

    /* USER CODE END M1 HALL_Update   */
  }
  else
  {
    /* Nothing to do. */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state. */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx) != 0)
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    TSK_SpeedTIM_task(); 
    
    /* USER CODE BEGIN M1 HALL_CC1 */
    
    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
    /* Nothing to do. */
  }

  if ((LL_TIM_IsActiveFlag_CC2 (HALL_M1.TIMx) != 0) && LL_TIM_IsEnabledIT_CC2(HALL_M1.TIMx))
  {
    LL_TIM_ClearFlag_CC2(HALL_M1.TIMx);

    if (RUN == Mci[M1].State)
    {
     (void)SixStep_StepCommution();
    }
    else
    {
      /* Nothing to do. */
    }
    LL_TIM_DisableIT_CC2(HALL_M1.TIMx); /* To avoid consecutive event when prescaler change. */

    /* USER CODE BEGIN M1 HALL_CC1 */
    
    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
    /* Nothing to do */
  }

  <#if DWT_CYCCNT_SUPPORTED>
    <#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Stop(&PerfTraces, (uint8_t)MEASURE_TSK_SpeedTimerM1);
    </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
  </#if><#-- DWT_CYCCNT_SUPPORTED -->
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}
</#if><#-- (M1_HALL_SENSOR == true) -->

/**
  * @brief  This function handles TIMx break-in interrupt request.
  * @param  None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */

  if (LL_TIM_IsActiveFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)}) != 0U)
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    (void)PWMC_BRK_IRQHandler(&PWM_Handle_M1);
  }
  else
  {
    /* Nothing to do. */
  }
  
  if (LL_TIM_IsActiveFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)}) != 0U)
  {
    LL_TIM_ClearFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    (void)PWMC_BRK_IRQHandler(&PWM_Handle_M1);
  }
  else
  {
    /* Nothing to do. */
  }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here. */
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
