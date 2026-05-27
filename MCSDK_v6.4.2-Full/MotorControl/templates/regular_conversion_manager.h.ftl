<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    regular_conversion_manager.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          regular_conversion_manager component of the Motor Control SDK.
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
#ifndef REGULAR_CONVERSION_MANAGER_H
#define REGULAR_CONVERSION_MANAGER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup COMMON_MC
  * @{
  */

  
/** @addtogroup RCM
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief RegConv_t contains all the parameters required to execute a regular conversion
  *
  * it is used by all regular_conversion_manager's client
  *
  */
typedef struct 
{
  ADC_TypeDef *regADC;    /*!< ADC peripheral used for the conversion */
  uint32_t samplingTime;  /*!< ADC sampling time used for the conversion */
  uint8_t channel;        /*!< ADC channel used for the conversion */
  uint16_t data;          /*!< ADC converted value */
  uint8_t id;             /*!< index of the conversion in RCM array */
} RegConv_t;

<#if MC.M1_CURRENT_MONITOR_READING == true>
/**
  * @brief CurrMonitor_t contains all the parameters required to execute a current conversion
  *
  */
typedef struct
{
  ADC_TypeDef *regADC;
  uint8_t  channel;
  uint32_t samplingTime;
  uint32_t currentConvFactor;
  uint16_t currentMa;
  uint8_t  hLowPassFilterBW;
  uint16_t hAvCurr_d;
  uint32_t samplingPointConvFact;
  uint16_t samplingDistance2Edge;
} CurrMonitor_t;

</#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->

typedef void (*RCM_exec_cb_t)(RegConv_t *regConv, uint16_t data, void *UserData);

/* Exported functions ------------------------------------------------------- */

/*  Function used to register a regular conversion */
bool RCM_RegisterRegConv(RegConv_t *regConv);

/* Non blocking function to start conversion inside HF task */
void RCM_ExecNextConv(void);

/* Non blocking function used to read back already started regular conversion */
void RCM_ReadOngoingConv(void);

/*  Function used to execute an already registered regular conversion */
uint16_t RCM_ExecRegularConv(RegConv_t *regConv);

/* This function is used to read the result of a regular conversion stored in the data structure. */
static inline uint16_t RCM_GetRegularConv(const RegConv_t *regConv)
{
#ifdef NULL_PTR_CHECK_REG_CON_MNG
  return ((MC_NULL == regConv) ? 0U : regConv->data);
#else
  return (regConv->data);
#endif
}

/* This function is used to wait for a the result of a regular conversion. */
void RCM_WaitForConv(void);

<#if MC.M1_CURRENT_MONITOR_READING == true>
/* Non blocking function to start the conversion of the current during PWM on-time*/
void RCM_ExecCurrentSense(CurrMonitor_t *currMon);

/* Non blocking function used to read back already started current conversion */
void RCM_ReadCurrentMonitor(CurrMonitor_t *currMon);

/**
  * @brief  Get the current monitor value.
  * @param  currMon: handler of the instance of the current monitor data structure
  * @retval uint16_t current value in mA.
  */
static inline uint16_t RCM_GetCurrentMonitor(CurrMonitor_t *currMon)
{
#ifdef NULL_PTR_CHECK_CUR_MON
  return ((MC_NULL == pHandle) ? 0U : currMon->hAvCurr_d);
#else
  return (currMon->hAvCurr_d);
#endif
}

/**
  * @brief  Get the number of points for current monitor average.
  * @param  currMon: handler of the instance of the current monitor data structure
  * @retval uint16_t # points.
  */
static inline uint16_t RCM_GetCurrentMonitorAvg(CurrMonitor_t *currMon)
{
#ifdef NULL_PTR_CHECK_CUR_MON
  return ((MC_NULL == pHandle) ? 0U : (uint16_t) currMon->hLowPassFilterBW);
#else
  return ((uint16_t)currMon->hLowPassFilterBW);
#endif
}

/**
  * @brief  Set the number of points for current monitor average.
  * @param  currMon: handler of the instance of the current monitor data structure
  * @param  uint8_t: new value to set
  * @retval void.
  */
static inline void RCM_SetCurrentMonitorAvg(CurrMonitor_t *currMon, uint8_t value)
{
#ifdef NULL_PTR_CHECK_CUR_MON
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    currMon->hLowPassFilterBW = value;
#ifdef NULL_PTR_CHECK_CUR_MON
  }
#endif
}

/**
  * @brief  Get the delay to PWM edge od the current monitor sampling point.
  * @param  currMon: handler of the instance of the current monitor data structure
  * @retval uint16_t value in ns.
  */
static inline uint16_t RCM_GetCurrentMonitorDist2Edge(CurrMonitor_t *currMon)
{
#ifdef NULL_PTR_CHECK_CUR_MON
  return ((MC_NULL == pHandle) ? 0U : currMon->samplingDistance2Edge);
#else
  return (currMon->samplingDistance2Edge);
#endif
}

/**
  * @brief  Set the delay to PWM edge od the current monitor sampling point.
  * @param  currMon: handler of the instance of the current monitor data structure
  * @retval void.
  */
static inline void RCM_SetCurrentMonitorDist2Edge(CurrMonitor_t *currMon, uint16_t value)
{
#ifdef NULL_PTR_CHECK_CUR_MON
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    currMon->samplingDistance2Edge = value;
#ifdef NULL_PTR_CHECK_CUR_MON
  }
#endif
}
</#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* REGULAR_CONVERSION_MANAGER_H */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
