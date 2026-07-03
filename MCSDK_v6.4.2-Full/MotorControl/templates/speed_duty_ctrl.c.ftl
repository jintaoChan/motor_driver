<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/*
  ******************************************************************************
  * @file    speed_duty_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Speed Control component of the Motor Control SDK.
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
#include "speed_duty_ctrl.h"
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
#include "mc_config.h"
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
#include "mc_type.h"
<#if MC.M1_SPEED_SENSOR == "HALL_SENSOR">
#include "drive_parameters.h"
#include "mc_config.h"
</#if><#-- MC.M1_SPEED_SENSOR == "HALL_SENSOR -->

/* Global Definitions --------------------------------------------------------*/
#define CHECK_BOUNDARY

/* Local Functions -----------------------------------------------------------*/
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  <#if MC.DRIVE_MODE == "VM">
static uint16_t SDC_CalcOpenLoopDutyCycleVM(uint16_t PWMperiod, OpenLoopSixstepCtrl_Handle_t *pHandle);
  <#else><#-- MC.DRIVE_MODE == "CM" -->
static uint16_t SDC_CalcOpenLoopDutyCycleCM(uint16_t PWMperiod, OpenLoopSixstepCtrl_Handle_t *pHandle);
  </#if><#-- MC.DRIVE_MODE == "VM" -->
static uint16_t SDC_CalcOpenLoopDutyCycle(uint32_t HTargetCntPh, OpenLoopSixstepCtrl_Handle_t *pHandle);
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup SpeednDutyCtrl Speed Control
  * @brief Speed Control component of the Motor Control SDK
  *
  * @todo Document the Speed Control "module".
  *
  * @{
  */

<#if MC.M1_SPEED_SENSOR == "HALL_SENSOR">
/**
  * @brief  It should be called before each motor restart. If SDC is set in
            speed mode, this method sets Current mechanical rotor speed reference.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @param  HFinalSpeed: targeted final speed command value.
  * @retval none.
  */
__weak void SDC_Init(SpeednDutyCtrl_Handle_t *pHandle, int16_t HFinalSpeed)
{
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t startSpeed = (((MAX_APPLICATION_SPEED_RPM / 2U) < DEFAULT_TARGET_SPEED_RPM) ? (((MAX_APPLICATION_SPEED_RPM / 2U) * SPEED_UNIT) / U_RPM) : DEFAULT_TARGET_SPEED_UNIT);
    
    if (1U != HALL_M1.Direction)
    {
      startSpeed = (-startSpeed);
      startSpeed = ((startSpeed < HFinalSpeed) ? HFinalSpeed : startSpeed);  
    }
    else
    {
      startSpeed = ((startSpeed > HFinalSpeed) ? HFinalSpeed : startSpeed);
    }
    pHandle->SpeedRefUnitExt = (int32_t)(startSpeed) * 65536; /* Mechanical rotor speed reference. */
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}
</#if><#-- MC.M1_SPEED_SENSOR == "HALL_SENSOR" -->

/**
  * @brief  It should be called before each motor restart. If SDC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval none.
  */
__weak void SDC_Clear(SpeednDutyCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    
    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      PID_SetIntegralTerm(pHandle->PISpeed, 0);
    }
    else
    {
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
      if ((SDC_GetOpenLoopFlag(pOLS[M1])) && (!SDC_GetRevUpFlag(pOLS[M1])))
      {
        pHandle->SpeedRefUnitExt = 0; /* For Open Loop case with no revup. */
      }
      else
      {
        /* nothing to do. */
      }
<#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE = false -->
      /* Nothing to do. */
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
    }
    pHandle->DutyCycleRef = ((uint32_t)pHandle->DutyCycleRefDefault) * 65536U;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
  * @brief  Starts the execution of a ramp using new target and duration. This
  *         command interrupts the execution of any previous ramp command.
  *         The generated ramp will be in the modality previously set by
  *         SDC_SetControlMode method.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @param  hTargetFinal final value of command. This is different accordingly
  *         the STC modality.
  *         hTargetFinal is the value of mechanical rotor speed reference at the end 
  *         of the ramp.Expressed in the unit defined by SPEED_UNIT
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It return false if the absolute value of hTargetFinal is out of
  *         the boundary of the application (Above max application speed or below min  
  *         application speed in this case the command is ignored and the
  *         previous ramp is not interrupted, otherwise it returns true.
  */
__weak bool SDC_ExecRamp(SpeednDutyCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms)
{
  bool allowedRange = true;

#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    allowedRange = false;
  }
  else
  {
#endif
    uint32_t wAux;
    int32_t wAux1;
    int16_t hCurrentReference;

    /* Check if the hTargetFinal is out of the bound of application. */
    if (MCM_DUTY_MODE == pHandle->Mode)
    {
      hCurrentReference = (int16_t)SDC_GetDutyCycleRef(pHandle);
#ifdef CHECK_BOUNDARY
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxPositiveDutyCycle)
      {
        allowedRange = false;
      }
      else
      {
        /* Nothing to do. */
      }
#endif
    }
    else
    {
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt >> 16);
#else
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt / 65536);
#endif

#ifdef CHECK_BOUNDARY
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxAppPositiveMecSpeedUnit)
      {
        allowedRange = false;
      }
      else if (hTargetFinal < pHandle->MinAppNegativeMecSpeedUnit)
      {
        allowedRange = false;
      }
      else if ((int32_t)hTargetFinal < (int32_t)pHandle->MinAppPositiveMecSpeedUnit)
      {
        if (hTargetFinal > pHandle->MaxAppNegativeMecSpeedUnit)
        {
          allowedRange = false;
        }
      }
      else
      {
        /* Nothing to do */
      }
#endif
    }

    if (true == allowedRange)
    {
      /* Interrupts the execution of any previous ramp command */
      if (0U == hDurationms)
      {
        if (MCM_SPEED_MODE == pHandle->Mode)
        {
          pHandle->SpeedRefUnitExt = ((int32_t)hTargetFinal) * 65536;
        }
        else
        {
          pHandle->DutyCycleRef = ((uint32_t)hTargetFinal) * 65536U;
        }
        pHandle->RampRemainingStep = 0U;
        pHandle->IncDecAmount = 0;
      }
      else
      {
        /* Store the hTargetFinal to be applied in the last step */
        pHandle->TargetFinal = hTargetFinal;

        /* Compute the (wRampRemainingStep) number of steps remaining to complete
        the ramp. */
        wAux = ((uint32_t)hDurationms) * ((uint32_t)pHandle->SDCFrequencyHz);
        wAux /= 1000U;
        pHandle->RampRemainingStep = wAux;
        pHandle->RampRemainingStep++;

        /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
        the reference value at each CalcSpeedReference. */
        wAux1 = (((int32_t)hTargetFinal) - ((int32_t)hCurrentReference)) * 65536;
        wAux1 /= ((int32_t)pHandle->RampRemainingStep);
        pHandle->IncDecAmount = wAux1;
      }
    }
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return (allowedRange);
}

/**
  * @brief  It is used to compute the new value of motor speed reference. It
  *         must be called at fixed time equal to hSDCFrequencyHz. It is called
  *         passing as parameter the speed sensor used to perform the speed
  *         regulation.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval int16_t motor dutycycle reference. This value represents actually the
  *         dutycycle expressed in digit.
  */
__weak uint16_t SDC_CalcSpeedReference(SpeednDutyCtrl_Handle_t *pHandle)
{
  uint16_t hDutyCycleReference = 0U;

#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    int32_t wCurrentReference;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (MCM_DUTY_MODE == pHandle->Mode)
    {
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
      if (SDC_GetOpenLoopFlag(pOLS[M1]))
      {
  <#if MC.DRIVE_MODE == "VM">
      wCurrentReference = (int32_t)SDC_CalcOpenLoopDutyCycleVM(pwmcHandle[M1]->PWMperiod,pOLS[M1]);
  <#else><#-- MC.DRIVE_MODE == "CM" -->
      wCurrentReference = (int32_t) SDC_CalcOpenLoopDutyCycleCM(pwmcHandle[M1]->PWMperiod, pOLS[M1]);
  </#if><#-- MC.DRIVE_MODE == "VM" -->
      pHandle->SpeedRefUnitExt = SPD_GetAvrgMecSpeedUnit(pHandle->SPD) * 65536;
      pHandle->RampRemainingStep = 0U;
      }
      else
      {
        wCurrentReference = (int32_t)pHandle->DutyCycleRef;
      }
<#else>
      wCurrentReference = (int32_t)pHandle->DutyCycleRef;
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
    }
    else
    {
      wCurrentReference = pHandle->SpeedRefUnitExt;
    }

    /* Update the speed reference or the torque reference according to the mode
       and terminates the ramp if needed. */
    if (pHandle->RampRemainingStep > 1U)
    {
      /* Increment/decrement the reference value. */
      wCurrentReference += pHandle->IncDecAmount;

      /* Decrement the number of remaining steps */
      pHandle->RampRemainingStep--;
    }
    else if (1U == pHandle->RampRemainingStep)
    {
      /* Set the backup value of hTargetFinal. */
      wCurrentReference = ((int32_t)pHandle->TargetFinal) * 65536;
      pHandle->RampRemainingStep = 0U;
    }
    else
    {
      /* Do nothing. */
    }

    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      /* Run the speed control loop */

      /* Compute speed error */
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hTargetSpeed = (int16_t)(wCurrentReference >> 16);
#else
      hTargetSpeed = (int16_t)(wCurrentReference / 65536);
#endif
      hMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pHandle->SPD);
      if (hTargetSpeed < 0)
      {
        hError = hMeasuredSpeed - hTargetSpeed;
      }
      else
      {
        hError = hTargetSpeed - hMeasuredSpeed;
      }
      hDutyCycleReference = (uint16_t)PI_Controller(pHandle->PISpeed, (int32_t)hError);

      pHandle->SpeedRefUnitExt = wCurrentReference;
      pHandle->DutyCycleRef = ((uint32_t)hDutyCycleReference) * 65536U;
    }
    else
    {
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == false>
      pHandle->DutyCycleRef = (uint32_t)wCurrentReference;
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hDutyCycleReference = (uint16_t)((int16_t)(wCurrentReference >> 16));
#else
      hDutyCycleReference = (uint16_t)((int16_t)(wCurrentReference / 65536));
#endif
<#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE = true -->
      if (!SDC_GetOpenLoopFlag(pOLS[M1]))
      {
        pHandle->DutyCycleRef = (uint32_t)wCurrentReference;
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
        //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        hDutyCycleReference = (uint16_t)((int16_t)(wCurrentReference >> 16));
#else
        hDutyCycleReference = (uint16_t)((int16_t)(wCurrentReference / 65536));
#endif
      }
      else
      {
        hDutyCycleReference = (uint16_t)wCurrentReference;
      }
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == false -->
    }
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return (hDutyCycleReference);
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval bool It returns true if the command is executed, false otherwise.
  */
__weak bool SDC_StopSpeedRamp(SpeednDutyCtrl_Handle_t *pHandle)
{
  bool retVal = false;

#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      pHandle->RampRemainingStep = 0u;
      retVal = true;
    }
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return (retVal);
}

<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  <#if MC.DRIVE_MODE == "VM">
/**
 * @brief  It is used to compute the new value of motor PWM duty Cycle reference in openLoop Voltage mode.
 *         It must be called at fixed time equal to hSTCFrequencyHz. It is called
 *         passing as parameter the PWM period, open loop parameters used to set up the duty cycle.
 * @param  PWMperiod: PWM Period counter value.
 * @param  pHandle: handler of the current instance of the SpeednTorqCtrl and component.
 * @retval int16_t: Motor duty cycle reference. This value represents actually the
 *         dutycycle expressed in digit for PWM counter.
 */
uint16_t SDC_CalcOpenLoopDutyCycleVM(uint16_t PWMperiod, OpenLoopSixstepCtrl_Handle_t *pHandle)
{
  uint32_t tempTargetCntPh = 0U;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    uint32_t highLimit = (pHandle->VoltageFactor * (uint32_t)PWMperiod) / 100U; /* Compute high limitation in PWM counter value */
    tempTargetCntPh = ((uint32_t)pHandle->DutyCycleRef * (uint32_t)highLimit) / 100U; /* Convert from percentage to PWM counter value. */
    tempTargetCntPh = SDC_CalcOpenLoopDutyCycle(tempTargetCntPh, pHandle); /* Compute mean duty cyle value */
  }
  return ((uint16_t)tempTargetCntPh);
}
  <#else><#-- MC.DRIVE_MODE == "CM" -->
/**
 * @brief  It is used to compute the new value of PWM current duty Cycle reference in current mode.
 *         It must be called at fixed time equal to hSTCFrequencyHz. It is called
 *         passing as parameter the PWM period, the open loop parameters used to set up the duty cycle.
 * @param  PWMperiod: PWM Period counter value of current refernce.
 * @param  pHandle: handler of the current instance of the SpeednTorqCtrl and component.
 * @retval int16_t:  PWM curretn dutycycle reference. This value represents actually the
 *         dutycycle expressed in digit for PWM counter.
 */
uint16_t SDC_CalcOpenLoopDutyCycleCM(uint16_t PWMperiod, OpenLoopSixstepCtrl_Handle_t *pHandle)
{
  uint32_t hTargetCntPh = 0U;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    uint32_t highLimit = (pHandle->CurrentFactor * (uint32_t)PWMperiod) / 100U; /* Compute high limitation in PWM counter value */
    hTargetCntPh = ((uint32_t)pHandle->DutyCycleRef * (uint32_t)highLimit) / 100U;  /* Convert from percentage to CM counter value. */
    hTargetCntPh = SDC_CalcOpenLoopDutyCycle(hTargetCntPh, pHandle); /* Compute mean duty cyle value */
  }
  return ((uint16_t)hTargetCntPh);
}
  </#if><#-- MC.DRIVE_MODE == "VM" -->
/**
 * @brief  It is used to compute the mean value of duty Cycle reference in Current or Voltage mode.
 *         1 order digital filter of the following type: output += (input - output) >> 8.
 *         It is called passing as parameter the current HTargetCntPh and the open loop parameters.
 * @param  HTargetCntPh: PWM Period counter value of current refernce.
 * @param  pHandle: handler of the current instance of the SpeednTorqCtrl and component.
 * @retval uint16_t motor dutycycle reference. This value represents actually the
 *         mean dutycycle expressed in digit for PWM counter.
 */
uint16_t SDC_CalcOpenLoopDutyCycle(uint32_t HTargetCntPh, OpenLoopSixstepCtrl_Handle_t *pHandle)
{
  uint32_t tempTargetCntPh = HTargetCntPh;

  /* pHandle->DutyCycleRefMean += (hTargetCntPh - pHandle->DutyCycleRefMean) >> pHandle->DutyCycleRefFilter */
  int32_t tempvalue = (int32_t)tempTargetCntPh - (int32_t)pHandle->DutyCycleRefMean;

  if (tempvalue < 0)
  {
    tempTargetCntPh = ((uint32_t)(-tempvalue) >> pHandle->DutyCycleRefFilter);
    tempTargetCntPh = pHandle->DutyCycleRefMean - tempTargetCntPh;
  }
  else
  {
    tempTargetCntPh = ((uint32_t)tempvalue >> pHandle->DutyCycleRefFilter);
    tempTargetCntPh += pHandle->DutyCycleRefMean;
  }

  pHandle->DutyCycleRefMean = tempTargetCntPh;

  return ((uint16_t)tempTargetCntPh);
}

/**
 * @brief  It is used to set the new value of duty cycle reference coming from the potentiometer (ADC).
 *         It must be called at fixed time equal to hSTCFrequencyHz. It is called
 *         passing as parameter the aw value comming from ADC IP..
 * @param  pHandle: handler of the current instance of the SpeednTorqCtrl and component.
 * @param  RawValue: ADC value coming from the potentiometer.
 * @retval none.
 */
void SDC_Potentiometer_Run(OpenLoopSixstepCtrl_Handle_t *pHandle, uint16_t RawValue)
{
  uint32_t dutyCycleRef = (120U * (uint32_t)RawValue) / 65535U; /* Convert from 16bits to percentage value. 
                                                                 * 120 to compensate the ADC max value < 65535*/
  dutyCycleRef = ((dutyCycleRef < 1U) ? 0U : dutyCycleRef); /* Lower value limitation */
  dutyCycleRef = ((dutyCycleRef > 100U) ? 100U : dutyCycleRef); /* higher value limitation */
  SDC_SetDutyCycleRefOl(pHandle, (uint8_t)dutyCycleRef); /* Set the dutyCycleRef */
}

</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
