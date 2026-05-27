
<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_config.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler 
  *          structures declarations.
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
  
#ifndef MC_CONFIG_H
#define MC_CONFIG_H

#include "speed_duty_ctrl.h"
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
#include "speed_duty_ctrl.h"
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->
#include "revup_ctrl_sixstep.h"
<#if MC.M1_OTF_STARTUP == true> 
#include "otf_sixstep.h"
</#if><#-- MC.M1_OTF_STARTUP == true-->
<#if MC.M1_IPD_STARTUP == true> 
#include "ipd_sixstep.h"
</#if><#-- MC.M1_IPD_STARTUP == true-->
#include "mc_config_common.h"
#include "pwmc_sixstep.h"
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
#include "bemf_ADC_fdbk_sixstep.h"
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if (M1_HALL_SENSOR == true)>
#include "hall_speed_pos_fdbk_sixstep.h"
</#if><#-- (M1_HALL_SENSOR == true) -->
<#-- ICL feature usage -->
<#if MC.M1_ICL_ENABLED == true>
#include "inrush_current_limiter.h"
</#if><#-- MC.M1_ICL_ENABLED == true  -->

extern PWMC_Handle_t PWM_Handle_M1;
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
extern Bemf_ADC_Handle_t Bemf_ADC_M1;
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
extern SixStepVars_t SixStepVars[NBR_OF_MOTORS];
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
extern RevUpCtrl_6S_Handle_t RevUpControlM1;
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") 
         || MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern SpeednDutyCtrl_Handle_t *pSDC[NBR_OF_MOTORS];
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
extern OpenLoopSixstepCtrl_Handle_t *pOLS[NBR_OF_MOTORS];
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->
extern MCI_Handle_t Mci[NBR_OF_MOTORS];
<#if MC.M1_POSITION_CTRL_ENABLING == true>
extern PID_Handle_t PID_PosParamsM1;
extern PosCtrl_Handle_t PosCtrlM1;
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
extern PID_Handle_t PIDSpeedHandle_M1;
<#if M1_HALL_SENSOR == true>
extern HALL_6S_Handle_t HALL_M1;
</#if><#-- M1_HALL_SENSOR == true -->
<#if MC.M1_OTF_STARTUP == true> 
extern OTF_6S_Handle_t OTF_M1;
</#if><#-- MC.M1_OTF_STARTUP == true-->
<#if MC.M1_CURRENT_MONITOR_READING == true>
extern CurrMonitor_t CurrMonitor_M1;
</#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
<#if MC.M1_IPD_STARTUP == true> 
extern IPD_6S_Handle_t IPD_M1;
</#if><#-- MC.M1_IPD_STARTUP == true-->
<#if MC.M1_ICL_ENABLED == true>
extern ICL_Handle_t ICL_M1;
extern DOUT_handle_t ICLDOUTParamsM1;
</#if><#-- MC.M1_ICL_ENABLED == true -->
/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  

#endif /* MC_CONFIG_H */
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
