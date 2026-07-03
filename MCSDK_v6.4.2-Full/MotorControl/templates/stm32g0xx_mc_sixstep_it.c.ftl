<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    stm32g0xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control
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
  * @ingroup STM32G0xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx.h" 

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32G0xx_IRQ_Handlers STM32G0xx IRQ Handlers
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

void BEMF_READING_IRQHandler(void);
void PERIOD_COMM_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
<#if M1_HALL_SENSOR == true>
${IRQHandler_name(_last_word(MC.M1_HALL_TIMER_SELECTION))};
</#if>


/**
  * @brief  This function handles TIMx Break-in interrupt request.
  * @param  None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */   

  if(LL_TIM_IsActiveFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)}) && LL_TIM_IsEnabledIT_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)})) 
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    PWMC_BRK_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */ 
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_TIM_IsActiveFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)}) && LL_TIM_IsEnabledIT_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)})) 
  {
    LL_TIM_ClearFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    PWMC_BRK_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */ 
  }
  else 
  {
     /* No other interrupts are routed to this handler */
  }
  
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */   
}


<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/**
  * @brief  This function handles BEMF sensing interrupt request.
  * @param[in] None
  */
void BEMF_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  if(LL_ADC_IsActiveFlag_AWD1(ADC1) && LL_ADC_IsEnabledIT_AWD1(ADC1))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_AWD1(ADC1);
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
  }
  else
  {
    /* Nothing to do */
  }

  <#if MC.M1_IPD_STARTUP == true>
  if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetRunStateFlag(&IPD_M1)))
  {
    /* Nothing to do. */
  }
  else
  {
  </#if><#-- MC.M1_IPD_STARTUP == true -->
  <#if MC.M1_CURRENT_MONITOR_READING == true >
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
    
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief     LFtimer interrupt handler
  * @param[in] None
  */
void PERIOD_COMM_IRQHandler(void)
{
  /* TIM Capture compare 1 event */

  if(LL_TIM_IsActiveFlag_CC1(${_last_word(MC.LF_TIMER_SELECTION)}) && LL_TIM_IsEnabledIT_CC1(${_last_word(MC.LF_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_CC1(${_last_word(MC.LF_TIMER_SELECTION)});
    TSK_SpeedTIM_task(); 
  }
  else
  {
    /* Nothing to do */
  }
}
</#if><#--  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

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
${IRQHandler_name(SensorTimer)}  
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */ 
  
  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx) != 0)
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);

    /* USER CODE BEGIN HALL_Update */

    /* USER CODE END HALL_Update   */ 
  }
  else
  {
    /* Nothing to do */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx) != 0)
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    TSK_SpeedTIM_task(); 
    
    /* USER CODE BEGIN M1 HALL_CC1 */
    
    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
    /* Nothing to do */
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

  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */ 
}
</#if>

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
