<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/foc_assign.ftl">

/**
  ******************************************************************************
  * @file    stm32u5xx_mc_it.c 
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
  * @ingroup STM32U5xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
<#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32u5xx_ll_exti.h"
</#if>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32U5xx_IRQ_Handlers STM32U5xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

<#-- Specific to FOC algorithm usage -->
/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC1_IRQHandler(void);

void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
<#if (M1_HALL_SENSOR == true)>
void SPD_HALL_TIM_M1_IRQHandler(void);
</#if><#-- (M1_HALL_SENSOR == true) -->
<#if MC.DRIVE_NUMBER != "1">
void TIMx_UP_M2_IRQHandler(void);
void TIMx_BRK_M2_IRQHandler(void);
  <#if (M2_HALL_SENSOR == true)>
void SPD_HALL_TIM_M2_IRQHandler(void);
  </#if><#-- (M2_HALL_SENSOR == true) -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/**
  * @brief  This function handles ADC1 interrupt request.
  * @param  None
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */

  if(LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_JEOS(ADC1);
    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  */
void TIMx_UP_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_M1_IRQn 0 */  
  LL_TIM_ClearFlag_UPDATE(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
<#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ((MC.M1_CS_ADC_NUM == '1')))>
  R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>      
  R1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
</#if>

  /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_M1_IRQn 1 */  
}



<#if FOC>
<#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>     
    <#if _last_word(MC.M1_PWM_TIMER_SELECTION) == "TIM1">
      <#assign CHANNEL = "Channel0">
    <#elseif _last_word(MC.M1_PWM_TIMER_SELECTION) == "TIM8">
      <#assign CHANNEL = "Channel1">
    <#else>
      #error Not supported
    </#if>

/**
  * @brief  This function handles first motor DMAx TC interrupt request. 
  *         
  * @param  None
  */
void ${R1_DMA_M1}_${CHANNEL}_IRQHandler(void)
{
  uint32_t tempReg1 = LL_DMA_IsActiveFlag_HT(${R1_DMA_M1}, ${R1_DMA_CH_M1});
  uint32_t tempReg2 = LL_DMA_IsEnabledIT_HT(${R1_DMA_M1}, ${R1_DMA_CH_M1});

  if ((tempReg1 != 0U) && (tempReg2 != 0U))
  {
    (void)R1_DMAx_HT_IRQHandler(&PWM_Handle_M1);
    LL_DMA_ClearFlag_HT(${R1_DMA_M1}, ${R1_DMA_CH_M1});
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_DMA_IsActiveFlag_TC(${R1_DMA_M1}, ${R1_DMA_CH_M1}) != 0U)
  {
    LL_DMA_ClearFlag_TC(${R1_DMA_M1}, ${R1_DMA_CH_M1});
    (void)R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);
  }
  else
  {
    /* Nothing to do */
  }
}
</#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- FOC -->

<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/**
  * @brief  This function handles BEMF sensing interrupt request.
  * @param[in] None
  */
void BEMF_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  if(LL_DMA_IsActiveFlag_TC1(DMA1) && LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1))
  {
  /* Clear Flags */
    LL_DMA_ClearFlag_TC1( DMA1 );
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */
    BADC_IsZcDetected( &Bemf_ADC_M1, &PWM_Handle_M1._Super );
  }
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief     LFtimer interrupt handler
  * @param[in] None
  */
void PERIOD_COMM_IRQHandler(void)
{
  /* TIM Update event */

  if(LL_TIM_IsActiveFlag_UPDATE(Bemf_ADC_M1.pParams_str->LfTim) && LL_TIM_IsEnabledIT_UPDATE(Bemf_ADC_M1.pParams_str->LfTim))
  {
    LL_TIM_ClearFlag_UPDATE(Bemf_ADC_M1.pParams_str->LfTim);
    BADC_StepChangeEvent(&Bemf_ADC_M1, 0, &PWM_Handle_M1._Super);
    (void)TSK_HighFrequencyTask();
  }
}

</#if><#--  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if (M1_HALL_SENSOR == true)>
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  */
void SPD_HALL_TIM_M1_IRQHandler(void)
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
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx)) 
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    HALL_TIMx_CC_IRQHandler(&HALL_M1);
    /* USER CODE BEGIN HALL_CC1 */

    /* USER CODE END HALL_CC1 */ 
  }
  else
  {
  /* Nothing to do */
  }
 /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */ 
}
</#if>

/**
  * @brief  This function handles first motor BRK interrupt.
  * @param  None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */

  if (LL_TIM_IsActiveFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    <#if (MC.M1_OCP_TOPOLOGY != "NONE") &&  (MC.M1_OCP_DESTINATION == "TIM_BKIN")>
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
    <#elseif (MC.M1_DP_TOPOLOGY != "NONE") &&  (MC.M1_DP_DESTINATION == "TIM_BKIN")>
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
    <#else>
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, ${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    </#if>
  }
  else
  {
    /* Nothing to do */
  }
  
  if (LL_TIM_IsActiveFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
<#if (MC.M1_OCP_TOPOLOGY != "NONE") &&  (MC.M1_OCP_DESTINATION == "TIM_BKIN2")>
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
<#elseif (MC.M1_DP_TOPOLOGY != "NONE") &&  (MC.M1_DP_DESTINATION == "TIM_BKIN2")>
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
<#else>
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, ${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
  }
  else
  {
    /* Nothing to do */
  }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */ 
}


/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
