<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    hf_registers.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the handling 
  * of registers access for the DAC and ASYNC protocol used in the High Frequency
  * Task.
  *
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

#include "stdint.h"
#include "register_interface.h"
#include "mc_config.h"

uint8_t HF_GetIDSize(uint16_t dataID)
{
  uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;
  uint8_t result;

  switch (typeID)
  {
    case TYPE_DATA_8BIT:
    {
      result = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
      result = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {
      result = 4;
      break;
    }

    default:
    {
      result=0;
      break;
    }
  }
  
  return (result);
}

<#macro GetPtrReg MOTOR_NUM>

  uint8_t retVal = HF_CMD_OK;
  static uint16_t nullData16=0;

#ifdef NULL_PTR_CHECK_REG_INT  
  if (MC_NULL == dataPtr)
  {
    retVal = HF_CMD_NOK;
  }
  else
  {
#endif

    MCI_Handle_t *pMCIN = &Mci[${MOTOR_NUM-1}];
    uint16_t regID = dataID & REG_MASK;
    uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;

    switch (typeID)
    {
      case TYPE_DATA_16BIT:
      {
        switch (regID)
        {
<#if FOC || ACIM>
  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_I_A:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.a);
             break;
          }

          case MC_REG_I_B:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.b);
            break;
          }

          case MC_REG_I_ALPHA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.alpha);
            break;
          }

          case MC_REG_I_BETA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.beta);
            break;
          }

          case MC_REG_I_Q_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.q);
            break;
          }

          case MC_REG_I_D_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.d);
            break;
          }

          case MC_REG_I_Q_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.q);
            break;
          }

          case MC_REG_I_D_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.d);
            break;
          }
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" && 
      (.vars["M"+ MOTOR_NUM?c + "_IS_SENSORLESS"] || .vars["M"+ MOTOR_NUM?c + "_IS_ENCODER"] || .vars.MC["M"+ MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"]) >	  
          case MC_REG_OPENLOOP_EL_ANGLE:
          {
            *dataPtr = &((&VirtualSpeedSensorM${MOTOR_NUM})->_Super.hElAngle);
            break;
          }	  
    </#if> <#-- Mx_DRIVE_TYPE == "FOC" && (Mx_IS_SENSORLESS || Mx_IS_ENCODER || Mx_DBG_OPEN_LOOP_ENABLE) -->  
  </#if><#-- MEMORY_FOOTPRINT_REG -->
  
          case MC_REG_V_Q:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.q);
            break;
          }

          case MC_REG_V_D:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.d);
            break;
          }

          case MC_REG_V_ALPHA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.alpha);
            break;
          }

          case MC_REG_V_BETA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.beta);
            break;
          }
</#if><#-- FOC || ACIM -->

<#if SIX_STEP>
          case MC_REG_PULSE_VALUE:
          {
            *dataPtr = &(pMCIN->pSixStepVars->DutyCycleRef);
            break;
          }
</#if><#-- SIX_STEP -->

<#if !SIX_STEP>
  <#if MEMORY_FOOTPRINT_REG>
  <#if .vars["M"+ MOTOR_NUM?c + "_IS_HALL_SENSOR"]>
          case MC_REG_HALL_SPEED:
          {
            *dataPtr = &((&HALL_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_HALL_EL_ANGLE:
          {
            *dataPtr = &((&HALL_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
  </#if><#-- IS_HALL_SENSOR -->
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if><#-- !SIX_STEP -->

<#if .vars["M"+ MOTOR_NUM?c + "_IS_ENCODER"]>
          case MC_REG_ENCODER_SPEED:
          {
            *dataPtr = &((&ENCODER_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_ENCODER_EL_ANGLE:
          {
            *dataPtr = &((&ENCODER_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
</#if><#-- IS_ENCODER -->

<#if .vars["M"+ MOTOR_NUM?c + "_IS_STO_PLL"]>
          case MC_REG_STOPLL_ROT_SPEED:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_STOPLL_EL_ANGLE:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
          
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
          case MC_REG_STOPLL_I_ALPHA:
          case MC_REG_STOPLL_I_BETA:
            break;
#endif

          case MC_REG_STOPLL_BEMF_ALPHA:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->hBemf_alfa_est);
            break;
          }

          case MC_REG_STOPLL_BEMF_BETA:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->hBemf_beta_est);
            break;
          }
</#if><#-- IS_STO_PLL -->

<#if .vars["M"+ MOTOR_NUM?c + "_IS_STO_CORDIC"]>
          case MC_REG_STOCORDIC_ROT_SPEED:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_STOCORDIC_EL_ANGLE:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
          case MC_REG_STOCORDIC_I_ALPHA:
          case MC_REG_STOCORDIC_I_BETA:
            break;
#endif
          case MC_REG_STOCORDIC_BEMF_ALPHA:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->hBemf_alfa_est);
            break;
          }

          case MC_REG_STOCORDIC_BEMF_BETA:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->hBemf_beta_est);
            break;
          }
</#if><#-- IS_STO_CORDIC -->

          default:
          {
            *dataPtr = &nullData16;
            retVal = HF_ERROR_UNKNOWN_REG;
            break;
          }
        }
        break;
      }

      default:
      {
        *dataPtr = &nullData16;
        retVal = HF_ERROR_UNKNOWN_REG;
        break;
      }
    }
#ifdef NULL_PTR_CHECK_REG_INT
  }
#endif
  return (retVal);
</#macro>

<#if (MC.DRIVE_NUMBER?number) gt 1>
__weak uint8_t HF_GetPtrReg(uint16_t dataID, void **dataPtr)
{
  uint8_t retVal;
  
  uint8_t motorID = (uint8_t)((dataID & MOTOR_MASK) - 1U);
  uint8_t (*GetPtrRegFcts[NBR_OF_MOTORS])(uint16_t, void**) = {&HF_GetPtrRegMotor1<#list 2..(MC.DRIVE_NUMBER?number) as NUM>, &HF_GetPtrRegMotor${NUM}</#list>};

  retVal = GetPtrRegFcts[motorID](dataID, dataPtr);

  return(retVal); 
}
<#list 1..(MC.DRIVE_NUMBER?number) as NUM>
__weak uint8_t HF_GetPtrRegMotor${NUM}(uint16_t dataID, void **dataPtr)
{
   <@GetPtrReg MOTOR_NUM=NUM />
}
</#list>

<#else><#-- MC.DRIVE_NUMBER <= 1 -->
__weak uint8_t HF_GetPtrReg(uint16_t dataID, void **dataPtr)
{
   <@GetPtrReg MOTOR_NUM=1 />
}
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/