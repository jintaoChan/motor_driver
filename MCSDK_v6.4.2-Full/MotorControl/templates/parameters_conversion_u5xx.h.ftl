<#ftl>
<#if !MC??>
<#if SWIPdatas??>
<#list SWIPdatas as SWIP>
<#if SWIP.ipName == "MotorControl">
<#if SWIP.parameters??>
<#assign MC = SWIP.parameters>
<#break>
</#if>
</#if>
</#list>
</#if>
<#if !MC??>
<#stop "No MotorControl SW IP data found">
</#if>
<#if configs[0].peripheralParams.get("RCC")??>
<#assign RCC = configs[0].peripheralParams.get("RCC")>
</#if>
<#if !RCC??>
<#stop "No RCC found">
</#if>
</#if>

<#assign FOC = MC.M1_DRIVE_TYPE == "FOC" || MC.M2_DRIVE_TYPE == "FOC">
<#assign SIX_STEP = MC.M1_DRIVE_TYPE == "SIX_STEP" || MC.M2_DRIVE_TYPE == "SIX_STEP">

/**
  ******************************************************************************
  * @file    parameters_conversion_u5xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32U5 Family.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_U5XX_H
#define __PARAMETERS_CONVERSION_U5XX_H

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
<#assign SYSCLKFreq = RCC.get("SYSCLKFreq_VALUE")?number>
<#assign ADV_TIM_CLKFreq = RCC.get("APB2TimFreq_Value")?number> <#-- Advanced timers are all on APB2 -->
<#assign ADV_TIM_CLKFreq2 = RCC.get("APB2TimFreq_Value")?number>
<#assign ADC_TIM_CLKFreq = RCC.get("ADCFreq_Value")?number>
<#assign ADCFreq = RCC.get("ADCFreq_Value")?number>


#define SYSCLK_FREQ      ${SYSCLKFreq}uL
#define TIM_CLOCK_DIVIDER  1 
#define ADV_TIM_CLK_MHz  ${(ADV_TIM_CLKFreq/1000000)?floor}
#define ADC_CLK_MHz     ${(ADC_TIM_CLKFreq/1000000)?floor}
#define HALL_TIM_CLK   ${SYSCLKFreq}uL
#define ADC1_2  ADC1

									  														  
/*************************  IRQ Handler Mapping  *********************/														  
<#if FOC>	
#define CURRENT_REGULATION_IRQHandler          DMA1_Channel1_IRQHandler
#define DMAx_R1_M1_IRQHandler                   DMA1_Channel4_5_IRQHandler
</#if>

<#if SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
#define BEMF_READING_IRQHandler          DMA1_Channel1_IRQHandler
  <#if MC.LF_TIMER_SELECTION == 'LF_TIM2'>
#define PERIOD_COMM_IRQHandler              TIM2_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM4'>
#define PERIOD_COMM_IRQHandler              TIM4_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM3'>
#define PERIOD_COMM_IRQHandler              TIM3_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM16'>
#define PERIOD_COMM_IRQHandler              TIM16_IRQHandler
  </#if>  
</#if>

/*************************  IRQ Handler Mapping  *********************/
<#if MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.M1_PWM_TIMER_SELECTION == 'TIM1'>
#define TIMx_UP_M1_IRQHandler            TIM1_UP_IRQHandler
#define TIMx_BRK_M1_IRQHandler           TIM1_BRK_IRQHandler
<#elseif MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.M1_PWM_TIMER_SELECTION == 'TIM8'>
#define TIMx_UP_M1_IRQHandler            TIM8_UP_IRQHandler
#define TIMx_BRK_M1_IRQHandler           TIM8_BRK_IRQHandler
<#else>
#error TIM not supported
</#if>

/*************************  ADC Physical characteristics  ************/			
#define ADC_TRIG_CONV_LATENCY_CYCLES 3
#define ADC_SAR_CYCLES 12.5 

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

#endif /*__PARAMETERS_CONVERSION_U5XX_H*/

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
