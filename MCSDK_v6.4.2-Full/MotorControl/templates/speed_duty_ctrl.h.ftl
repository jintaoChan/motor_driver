<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    speed_duty_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Speed & Duty Control component of the Motor Control SDK.
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
  * @ingroup SpeednTorqCtrl
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPEEDDUTYCTRLCLASS_H
#define SPEEDDUTYCTRLCLASS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "speed_pos_fdbk_sixstep.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeedDutyCtrl
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Speed and Duty cycle Control parameters definition
  */
typedef struct
{
  MC_ControlMode_t Mode;               /*!< Modality of SDC. It can be one of these two
                                            settings: MCM_DUTY_MODE to enable the
                                            open loop mode or MCM_SPEED_MODE to enable the
                                            Speed loop mode.*/
  int16_t TargetFinal;                 /*!< Backup of hTargetFinal to be applied in the
                                            last step.*/
  int32_t SpeedRefUnitExt;             /*!< Current mechanical rotor speed reference
                                            expressed in SPEED_UNIT multiplied by
                                            65536.*/
  uint32_t DutyCycleRef;               /*!< Current pulse reference.*/
  uint32_t RampRemainingStep;          /*!< Number of steps remaining to complete the
                                            ramp.*/
  PID_Handle_t *PISpeed;               /*!< The regulator used to perform the speed
                                            control loop.*/
  SpeednPosFdbk_6S_Handle_t *SPD;      /*!< The speed sensor used to perform the speed
                                            regulation.*/
  int32_t IncDecAmount;                /*!< Increment/decrement amount to be applied to
                                            the reference value at each
                                            CalcSpeedReference.*/

  uint16_t SDCFrequencyHz;             /*!< Frequency on which the user updates
                                            the duty reference calling
                                            SDC_CalcSpeedReference method
                                            expressed in Hz */
  uint16_t MaxAppPositiveMecSpeedUnit; /*!< Application maximum positive value
                                            of the rotor mechanical speed. Expressed in
                                            the unit defined by #SPEED_UNIT.*/
  uint16_t MinAppPositiveMecSpeedUnit; /*!< Application minimum positive value
                                            of the rotor mechanical speed. Expressed in
                                            the unit defined by #SPEED_UNIT.*/
  int16_t MaxAppNegativeMecSpeedUnit;  /*!< Application maximum negative value
                                            of the rotor mechanical speed. Expressed in
                                            the unit defined by #SPEED_UNIT.*/
  int16_t MinAppNegativeMecSpeedUnit;  /*!< Application minimum negative value
                                            of the rotor mechanical speed. Expressed in
                                            the unit defined by #SPEED_UNIT.*/
  uint16_t MaxPositiveDutyCycle;       /*!< Maximum positive value of motor
                                            torque. This value represents
                                            actually the maximum Iq current
                                            expressed in digit.*/
  MC_ControlMode_t ModeDefault;        /*!< Default STC modality.*/
  int16_t MecSpeedRefUnitDefault;      /*!< Default mechanical rotor speed
                                            reference expressed in the unit
                                            defined by #SPEED_UNIT.*/
  uint16_t DutyCycleRefDefault;        /*!< Default pulse reference.*/
} SpeednDutyCtrl_Handle_t;

<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
typedef struct
{
  uint32_t DutyCycleRefMean;                                             /*!< Computed DutyCycleRef mean value. */
  uint8_t VoltageFactor;                                                 /*!< Range of duty cycle for voltage mode (percnetage of max period). */
  uint8_t CurrentFactor;                                                 /*!< percentage Range of duty cycle for current mode (percnetage of max period). */
  uint8_t DutyCycleRef;                                                  /*!< Duty cycle =  (DutyCycleRef) value to set into timers. */
  uint8_t DutyCycleRefFilter;                                            /*!< filter coeff of the mean computation */
  bool Openloop;                                                         /*!< OpenLopp flag to enable disable the feature, true = enabled, false = disabled. */
  bool RevUp;                                                            /*!< RevUp flag to enable disable the startup feature, true = enabled, false = disabled. */
  bool OnSensing;                                                        /*!< On sensing flag to enable disable the PWM ON Sensing feature, true = enabled, false = disabled. */

} OpenLoopSixstepCtrl_Handle_t;
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.M1_SPEED_SENSOR == "HALL_SENSOR">
/* It sets Current mechanical rotor speed reference with expected speed. */
void SDC_Init(SpeednDutyCtrl_Handle_t *pHandle, int16_t HFinalSpeed);

</#if><#-- MC.M1_SPEED_SENSOR == "HALL_SENSOR" -->
/* It resets the integral term of speed regulator */
void SDC_Clear( SpeednDutyCtrl_Handle_t * pHandle );

/* Starts the execution of a ramp using new target and duration. */
bool SDC_ExecRamp( SpeednDutyCtrl_Handle_t * pHandle, int16_t hTargetFinal, uint32_t hDurationms );

/* It computes the new value of motor speed reference */
uint16_t SDC_CalcSpeedReference(SpeednDutyCtrl_Handle_t *pHandle);

/* Stop the execution of speed ramp. */
bool SDC_StopSpeedRamp( SpeednDutyCtrl_Handle_t * pHandle );

/**
  * @brief  Set the modality of the speed controller. Two modality
  *         are available Open loop mode and Speed mode.
  *         In open loop mode is possible to set directly the dutycycle 
  *         reference or execute a motor dutycycle ramp. This value represents
  *         actually the reference expressed in digit.
  *         In Speed mode is possible to set the mechanical rotor speed
  *         reference or execute a speed ramp. The required motor dutycycle is
  *         automatically calculated by the SDC.
  *         This command interrupts the execution of any previous ramp command
  *         maintaining the last value of dutycycle.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @param  bMode modality of SDC. It can be one of these two settings:
  *         MCM_DUTY_MODE to enable the open loop mode or MCM_SPEED_MODE to
  *         enable the Speed loop mode.
  * @retval none
  */
static inline void SDC_SetControlMode(SpeednDutyCtrl_Handle_t *pHandle, MC_ControlMode_t bMode)
{
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Mode = bMode;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
  * @brief  Get the current mechanical rotor speed reference.
  *         Expressed in the unit defined by SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval int16_t current mechanical rotor speed reference.
  *         Expressed in the unit defined by SPEED_UNIT.
  */
static inline int16_t SDC_GetMecSpeedRefUnit(const SpeednDutyCtrl_Handle_t *pHandle)
{
int16_t tempMecSpeedRefUnit = 0;
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    tempMecSpeedRefUnit = (int16_t)(pHandle->SpeedRefUnitExt >> 16);
#else
    tempMecSpeedRefUnit = (int16_t)(pHandle->SpeedRefUnitExt / 65536);
#endif
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return(tempMecSpeedRefUnit);
}

/**
  * @brief  Get the current motor dutycycle reference. This value represents
  *         actually the dutycycle reference expressed in digit.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval uint16_t current dutycycle reference. This value is actually expressed in digit.
  */
static inline uint16_t SDC_GetDutyCycleRef(const SpeednDutyCtrl_Handle_t *pHandle)
{
uint16_t tempDutyCycleRef = 0U;
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    tempDutyCycleRef = (uint16_t)(pHandle->DutyCycleRef >> 16U);
#else
    tempDutyCycleRef = (uint16_t)(pHandle->DutyCycleRef / 65536U);
#endif
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return(tempDutyCycleRef);
}

/**
  * @brief  Get the modality of the speed controller.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval MC_ControlMode_t It returns the modality of SDC. It can be one of
  *         these two values: MCM_DUTY_MODE or MCM_SPEED_MODE.
  */
static inline MC_ControlMode_t SDC_GetControlMode(const SpeednDutyCtrl_Handle_t *pHandle )
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? MCM_DUTY_MODE : pHandle->Mode);
#else
  return (pHandle->Mode);
#endif
}

/**
  * @brief  This command interrupts the execution of any previous ramp command.
  *         The last value of mechanical rotor speed reference is maintained.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval none
  */
static inline void SDC_StopRamp(SpeednDutyCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->RampRemainingStep = 0U;
    pHandle->IncDecAmount = 0;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
  * @brief  Check if the settled speed ramp has been completed.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
static inline bool SDC_RampCompleted(const SpeednDutyCtrl_Handle_t *pHandle)
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
    if (0U == pHandle->RampRemainingStep)
    {
      retVal = true;
    }
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
  return (retVal);
}

/**
  * @brief It returns the used speed sensor.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval SpeednPosFdbk_6S_Handle_t speed sensor utilized by the FOC.
  */
static inline SpeednPosFdbk_6S_Handle_t *SDC_GetSpeedSensor(const SpeednDutyCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL ==  pHandle) ? MC_NULL : pHandle->SPD);
#else
  return (pHandle->SPD);
#endif
}

/**
  * @brief It sets in real time the speed sensor utilized by the SDC.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @param SPD_Handle Speed sensor component to be set.
  * @retval none
  */
static inline void SDC_SetSpeedSensor( SpeednDutyCtrl_Handle_t * pHandle, SpeednPosFdbk_6S_Handle_t * SPD_Handle )
{
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->SPD = SPD_Handle;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
  * @brief  Get the Default mechanical rotor speed reference.
  *         Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval int16_t It returns the Default mechanical rotor speed. 
  *         Expressed in the unit defined by #SPEED_UNIT
  */
static inline int16_t SDC_GetMecSpeedRefUnitDefault(const SpeednDutyCtrl_Handle_t *pHandle)
{
  return ((MC_NULL == pHandle) ? 0 : pHandle->MecSpeedRefUnitDefault);
}

/**
  * @brief  Returns the Application maximum positive value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  */
static inline uint16_t SDC_GetMaxAppPositiveMecSpeedUnit(const SpeednDutyCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? 0U : pHandle->MaxAppPositiveMecSpeedUnit);
#else
  return (pHandle->MaxAppPositiveMecSpeedUnit);
#endif
}

/**
  * @brief  Returns the Application minimum negative value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  */
static inline int16_t SDC_GetMinAppNegativeMecSpeedUnit(const SpeednDutyCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? 0 : pHandle->MinAppNegativeMecSpeedUnit);
#else
  return (pHandle->MinAppNegativeMecSpeedUnit);
#endif
}

/**
  * @brief  Force the speed reference to the curren speed. It is used
  *         at the START_RUN state to initialize the speed reference.
  * @param  pHandle: handler of the current instance of the SpeednDutyCtrl component
  * @retval none
  */
static inline void SDC_ForceSpeedReferenceToCurrentSpeed(SpeednDutyCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->SpeedRefUnitExt = ((int32_t)SPD_GetAvrgMecSpeedUnit(pHandle->SPD)) * (int32_t)65536;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>

/* It is used to set the new value of duty cycle reference coming from the potentiometer (ADC). */
void SDC_Potentiometer_Run(OpenLoopSixstepCtrl_Handle_t *pHandle, uint16_t RawValue);

/**
 * @brief  It clear paramters.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @param  DutyCycleRefFilter: filtering of the mean computation of the DutyCycle [1..12]
 * @retval none.
 */

static inline void SDC_ClearDutyCycleMean(OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->DutyCycleRefMean = 0U;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}


/**
 * @brief  It sets the Duty cycle Filter in real time.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @param  DutyCycleRefFilter: filtering of the mean computation of the DutyCycle [1..12]
 * @retval none.
 */

static inline void SDC_SetDutyCycleRefFilter(OpenLoopSixstepCtrl_Handle_t *pHandle, uint8_t DutyCycleRefFilter)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->DutyCycleRefFilter = DutyCycleRefFilter;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
 * @brief  Returns the Duty cycle reference filter value to compute a mean of the requested speed.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component
 * @retval uint8_t: filtering of the mean computation of the DutyCycle [1..12].
 */
static inline uint8_t SDC_GetDutyCycleRefFilter(const OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? 0U : pHandle->DutyCycleRefFilter);
#else
  return (pHandle->DutyCycleRefFilter);
#endif
}

/**
 * @brief  It sets openloop BEMF Sensing ON command.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @param  bool: BEMF Sensing ON, true = enabled , false = disabled.
 * @retval none.
 */
static inline void SDC_SetOnSensing(OpenLoopSixstepCtrl_Handle_t *pHandle, bool OnSensing)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
  }
  else
  {
#endif
    pHandle->OnSensing = OnSensing;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
 * @brief  Returns the openloop BEMF Sensing ON command.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval bool: BEMF Sensing ON, true = enabled , false = disabled or NULL pointer detected when check enabled.
 */
static inline bool SDC_GetOnSensing(const OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? false : pHandle->OnSensing);
#else
  return (pHandle->OnSensing);
#endif
}

/**
 * @brief  Returns the openloop status flag.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval bool: openloop status flag to set, true = enable  or false = disable or NULL pointer detected when check enabled.
 */
static inline bool SDC_GetOpenLoopFlag(const OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? false : pHandle->Openloop);
#else
  return (pHandle->Openloop);
#endif
}

/**
 * @brief  It sets the openloop status flag in real time.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @param  flag: openloop status flag to set, true = enable  or false=disable.
 * @retval none.
 */
static inline void SDC_SetOpenLoopFlag(OpenLoopSixstepCtrl_Handle_t *pHandle, bool Flag)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Openloop = Flag;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
 * @brief  It sets the Duty cycle reference.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval none.
 */
static inline void SDC_SetDutyCycleRefOl(OpenLoopSixstepCtrl_Handle_t *pHandle, uint8_t DutyCycleRef)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->DutyCycleRef = DutyCycleRef;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
 * @brief  Returns the Duty cycle reference value.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval uint8_t: DutyCycleRef Duty cycle reference value in percentage unit 2 to 100.
 *                  0xFF when Null pointer is detected if check enabled.
 */
static inline uint8_t SDC_GetDutyCycleRefOl(const OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? 0xFFU : pHandle->DutyCycleRef);
#else
  return (pHandle->DutyCycleRef);
#endif
}

/**
 * @brief  It sets Current factor in percentage unit.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @param  uint8_t: Current factor in percentage unit.
 * @retval none.
 */
static inline void SDC_SetCurrentFactor(OpenLoopSixstepCtrl_Handle_t *pHandle, uint8_t CurrentFactor)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->CurrentFactor = CurrentFactor;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
 * @brief  Returns Current factor in percentage unit.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval int16_t: Current factor in percentage unit.
 */
static inline uint16_t SDC_GetCurrentFactor(const OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? 0U : pHandle->CurrentFactor);
#else
  return (pHandle->CurrentFactor);
#endif
}

/**
 * @brief  It sets Voltage factor in percentage unit.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @param int16_t: Voltage factor in percentage unit.
 * @retval none.
 */
static inline void SDC_SetVoltageFactor(OpenLoopSixstepCtrl_Handle_t *pHandle, uint8_t VoltageFactor)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->VoltageFactor = VoltageFactor;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
 * @brief  Returns Voltage Factor value for max PWM duty cycle.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval uint8_t: Voltage factor in percentage unit.
 */
static inline uint8_t SDC_GetVoltageFactor(const OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? 0U : pHandle->VoltageFactor);
#else
  return (pHandle->VoltageFactor);
#endif
}

/**
 * @brief  It sets the Rev-up phase enable flag.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval none.
 */
static inline void SDC_SetRevUpFlag(OpenLoopSixstepCtrl_Handle_t *pHandle, bool Flag)
{
#ifdef NULL_PTR_CHECK_SDC
  if ( MC_NULL == pHandle )
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->RevUp = Flag;
#ifdef NULL_PTR_CHECK_SDC
  }
#endif
}

/**
 * @brief  Returns the Rev-up phase enable flag.
 * @param  pHandle: handler of the current instance of the OpenloopSixstepCtrl component.
 * @retval bool: RevUp enable flag true = enabled, false = disabled or NULL pointer when check is enabled.
 */
static inline bool SDC_GetRevUpFlag(const OpenLoopSixstepCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SDC
  return ((MC_NULL == pHandle) ? false : pHandle->RevUp);
#else
  return (pHandle->RevUp);
#endif
}

</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->


/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* SPEEDDUTYCTRLCLASS_H */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/

