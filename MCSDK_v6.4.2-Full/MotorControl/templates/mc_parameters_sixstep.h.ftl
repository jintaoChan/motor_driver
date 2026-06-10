
<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides declarations of HW parameters specific to the 
  *          configuration of the subsystem.
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
#ifndef MC_PARAMETERS_H
#define MC_PARAMETERS_H

<#if MC.ESC_ENABLE>
#include "esc.h"
</#if><#-- MC.ESC_ENABLE -->
#include "mc_interface.h"  
#include "pwmc_sixstep.h"
<#if MC.M1_OTF_STARTUP == true>
#include "otf_sixstep.h"
</#if><#-- MC.M1_OTF_STARTUP == true -->
<#if MC.M1_IPD_STARTUP == true>
#include "ipd_sixstep.h"
</#if><#-- MC.M1_IPD_STARTUP == true -->

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

extern const PWMC_Params_t PWMC_ParamsM1;
  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
extern PWMC_TimerCfg_t ThreePwm_TimerCfgM1;
  <#else>
extern PWMC_TimerCfg_t SixPwm_TimerCfgM1;
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" -->
<#if MC.M1_OTF_STARTUP == true>
extern OTF_TimerCfg_t OTFTimerCfgM1;
</#if><#-- MC.M1_OTF_STARTUP == true -->
<#if MC.M1_IPD_STARTUP == true>
extern IPD_TimerCfg_t IPDTimerCfgM1;
</#if><#-- MC.M1_IPD_STARTUP == true -->
extern ScaleParams_t scaleParams_M1;
<#if MC.ESC_ENABLE == true>
extern const ESC_Params_t ESC_ParamsM1;
</#if><#-- MC.ESC_ENABLE == true -->
<#if MC.DRIVE_NUMBER != "1">
extern ScaleParams_t scaleParams_M2;
</#if><#-- MC.DRIVE_NUMBER > 1 -->
/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  

#endif /* MC_PARAMETERS_H */
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
