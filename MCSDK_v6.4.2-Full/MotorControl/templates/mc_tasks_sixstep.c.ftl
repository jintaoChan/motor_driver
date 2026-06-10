<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
<#include "*/ftl/sixstep_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_tasks_sixstep.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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
  * @ingroup MCTasksSixStep
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1 
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
<#if MC.RTOS == "FREERTOS">
#include "cmsis_os.h"
</#if><#-- MC.RTOS == "FREERTOS" -->
#include "mc_interface.h"
#include "digital_output.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
<#if MC.MCP_EN == true>
#include "mcp_config.h"
</#if><#--  MC.MCP_EN == true -->
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
#include "dac_ui.h"
</#if><#--  MC.DEBUG_DAC_FUNCTIONALITY_EN -->
#include "mc_app_hooks.h"
<#if MC.TESTENV == true>
#include "mc_testenv_6step.h"
</#if>




/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SixStep
  * @{
  */


/** @addtogroup	MCCockpitSixStep
  * @{
  */



  /** @defgroup MCCockpitSixStep MC Cockpit 
  * 
  * @brief   
  *
  * @{
  */



/** @addtogroup	MCTasksSixStep
  * @{
  */


    /** @defgroup MCTasksSixStep Motor Control Tasks
  * 
  * @brief  	Motor Control subsystem configuration and operation routines for SixStep applications. 
  *
  * @{
  */

/** @defgroup MCTasksSixStep Motor Control Tasks for Six Step algorithm
  * 
  * @brief SixStep Motor Control subsystem configuration and operation routines.  
  *
  * @{
  */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
static volatile uint16_t hBootCapDelayCounterM1   = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);
<#if MC.DRIVE_NUMBER != "1">
static volatile uint16_t hMFTaskCounterM2         = ((uint16_t)0);
static volatile uint16_t hBootCapDelayCounterM2   = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM2 = ((uint16_t)0);
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if (MC.M1_ICL_ENABLED == true)>
static volatile bool ICLFaultTreatedM1 = true;
</#if><#-- (MC.M1_ICL_ENABLED == true) -->

<#if CHARGE_BOOT_CAP_ENABLING == true>
#define M1_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)${MC.M1_PWM_CHARGE_BOOT_CAP_MS}) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)${MC.M1_PWM_CHARGE_BOOT_CAP_DUTY_CYCLES}\
                                     * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
</#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->
<#if CHARGE_BOOT_CAP_ENABLING2 == true>
#define M2_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)${MC.M2_PWM_CHARGE_BOOT_CAP_MS}) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)${MC.M2_PWM_CHARGE_BOOT_CAP_DUTY_CYCLES}\
                                      *((uint32_t)PWM_PERIOD_CYCLES2 / 2U))
</#if><#-- CHARGE_BOOT_CAP_ENABLING2 == true -->
<#if MC.M1_OTF_STARTUP>
#define M1_BRAKE_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)${MC.M1_OTF_6S_BRAKE_MS}) / 1000U)
#define M1_OTF_DETECTION_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)${MC.M1_OTF_6S_DETECTION_MS}) / 1000U)
#define M1_OTF_RAMP_DURATION            (uint16_t)${MC.M1_OTF_RAMP_DURATION_MS}

</#if><#-- MC.M1_OTF_STARTUP -->
<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  <#if  CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32F4>
#define SPEED_TIMER_IDLE_RATE_TICKS    (uint32_t) (REGULAR_CONVERSION_RATE_MS * APB1TIM_FREQ / (1000U * (LF_TIMER_PSC + 1U)))
  <#else>
#define SPEED_TIMER_IDLE_RATE_TICKS    (uint32_t) (REGULAR_CONVERSION_RATE_MS * SYSCLK_FREQ / (1000U * (LF_TIMER_PSC + 1U)))
  </#if><#-- CondFamily_STM32F3 || CondFamily_STM32G4 -->
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC"  -->

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);
<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
static void SixStep_StepCommution(void);
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC"  -->

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  None
  */
__weak void SIX_STEP_Init(void)
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */
  
    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1;
    ${PWM_Init}(&PWM_Handle_M1);
<#if MC.DRIVE_MODE == "CM">
  <#if MC.CURRENT_LIMITER_OFFSET>
    LL_TIM_OC_SetCompareCH1(${_last_word(MC.CURR_REF_TIMER_SELECTION)},PWM_PERIOD_CYCLES_REF); /* Set initial compare value for current control. */
  <#else><#-- !MC.CURRENT_LIMITER_OFFSET  -->
    LL_TIM_OC_SetCompareCH1(${_last_word(MC.CURR_REF_TIMER_SELECTION)}, 0); /* Set initial 0 compare value for current control. */
  </#if><#-- MC.CURRENT_LIMITER_OFFSET -->
    LL_TIM_EnableCounter(${_last_word(MC.CURR_REF_TIMER_SELECTION)}); /* Enable the timer counter. */
    LL_TIM_CC_EnableChannel(${_last_word(MC.CURR_REF_TIMER_SELECTION)},LL_TIM_CHANNEL_CH1); /* Select the channel. */
    LL_TIM_EnableAllOutputs(${_last_word(MC.CURR_REF_TIMER_SELECTION)}); /* Enable the PWM output. */
</#if><#-- SIX_STEP && MC.DRIVE_MODE == "CM -->

    /* USER CODE BEGIN MCboot 1 */
  
    /* USER CODE END MCboot 1 */
  
    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);
    
    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    ${SPD_init_M1}(${SPD_M1});

    /******************************************************/
    /*   Speed & duty cycle component initialization          */
    /******************************************************/
    SIX_STEP_Clear(M1);
    MCI_ExecSpeedRamp(&Mci[M1],
    SDC_GetMecSpeedRefUnitDefault(pSDC[M1]),0); /* First command to SDC */
    
<#if MC.M1_ICL_ENABLED == true>
    ICL_Init(&ICL_M1, &ICLDOUTParamsM1);
    Mci[M1].State = ICLWAIT;
</#if><#-- MC.M1_ICL_ENABLED == true -->

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function 
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
<#if MC.M1_COMPLEX_GATE_DRIVER_INTERFACE != "NONE">
    ${MC.M1_PWM_DRIVER_PN?upper_case}_StopManagement(&${MC.M1_PWM_DRIVER_PN?upper_case}_Handle_M1);
</#if>  
  SIX_STEP_Clear(motor);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */
  
  bool IsSpeedReliable = ${SPD_calcAvrgMecSpeedUnit_M1}(${SPD_M1});
<#if MC.M1_ICL_ENABLED == true>
  uint16_t Vbus_M1 = VBS_GetAvBusVoltage_V(&(BusVoltageSensor_M1._Super));
  ICL_State_t ICLstate = ICL_Exec(&ICL_M1, Vbus_M1);

  if ( !ICLFaultTreatedM1 && (ICLstate == ICL_ACTIVE))
  {
    ICLFaultTreatedM1 = true;
  }
  else
  {
    /* Nothing to do */
  }
</#if><#-- MC.M1_ICL_ENABLED == true -->

<#if MC.M1_IPD_STARTUP == true>
  if ((IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && IPD_6S_GetRunStateFlag(&IPD_M1))
  {
    IsSpeedReliable = true;
    Bemf_ADC_M1._Super.bSpeedErrorNumber = 0;
  }
  else
  {
    
    if (Bemf_ADC_M1._Super.bMaximumSpeedErrorsNumber > M1_SS_MEAS_ERRORS_BEFORE_FAULTS)
    {
      Bemf_ADC_M1._Super.bMaximumSpeedErrorsNumber = Bemf_ADC_M1._Super.bMaximumSpeedErrorsNumber - 1U;
      Bemf_ADC_M1._Super.bSpeedErrorNumber = 0;
      IsSpeedReliable = true;
    }
    else
    {
      /* Nothing to do end of starup IPD phase. */
    }
  }
</#if><#-- MC.M1_IPD_STARTUP == true -->

<#if MC.M1_ICL_ENABLED == true>
  if ((MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS) && ICLFaultTreatedM1)
<#else><#-- MC.M1_ICL_ENABLED == false -->
  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
</#if><#-- MC.M1_ICL_ENABLED == true -->
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {
<#if MC.M1_ICL_ENABLED == true>
        case ICLWAIT:
        {
          if (ICL_INACTIVE == ICLstate)
          {
            /* If ICL is Inactive, move to IDLE */
            Mci[M1].State = IDLE;
          }
          break;
        }
</#if><#-- MC.M1_ICL_ENABLED == true -->

        case IDLE:
        {
          if (MCI_START == Mci[M1].DirectCommand)
          {
<#if MC.M1_COMPLEX_GATE_DRIVER_INTERFACE != "NONE">
            ${MC.M1_PWM_DRIVER_PN?upper_case}_StartManagement(&${MC.M1_PWM_DRIVER_PN?upper_case}_Handle_M1);
</#if>  
<#if  (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") && (MC.M1_OTF_STARTUP == false) >
            RUC_6S_UpdatePulse(&RevUpControlM1, &BusVoltageSensor_M1._Super);
</#if><#-- (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") && (MC.M1_OTF_STARTUP == false)  -->
<#if MC.TESTENV == true >
            mc_testenv_init();
</#if><#-- MC.TESTENV == true -->
<#if CHARGE_BOOT_CAP_ENABLING == true>
  <#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE")>
    <#if MC.M1_DP_DESTINATION == "TIM_BKIN">
            LL_TIM_DisableBRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    <#elseif MC.M1_DP_DESTINATION == "TIM_BKIN2">
            LL_TIM_DisableBRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    </#if><#-- MC.M1_DP_DESTINATION == "TIM_BKIN" -->
  </#if><#-- (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") -->
            ${PWM_TurnOnLowSides}(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
            TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
            Mci[M1].State = CHARGE_BOOT_CAP;
</#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->
<#if MC.M1_OTF_STARTUP == true>
            OTF_6S_Init(&OTF_M1, &BusVoltageSensor_M1._Super);
            Mci[M1].State = OTF_DETECTION;
            TSK_SetOTFDetectionDelayM1(M1_OTF_DETECTION_TICKS);
</#if><#-- MC.M1_OTF_STARTUP == true-->
          }
          else
          {
<#if MC.TESTENV == true>
            mc_testenv_clear();
<#else><#-- MC.TESTENV = false -->
            /* Nothing to be done, FW stays in IDLE state. */
</#if><#-- MC.TESTENV == true -->
          }
          break;
        }
    
<#if (CHARGE_BOOT_CAP_ENABLING == true)>
        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              ${PWM_SwitchOff}(pwmcHandle[M1]);
  <#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE")>
    <#if MC.M1_DP_DESTINATION == "TIM_BKIN">
              LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
              LL_TIM_EnableBRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    <#elseif MC.M1_DP_DESTINATION == "TIM_BKIN2">
              LL_TIM_ClearFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
              LL_TIM_EnableBRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    </#if><#-- MC.M1_DP_DESTINATION == "TIM_BKIN" -->
  </#if><#-- (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") -->
  <#if (MC.M1_OTF_STARTUP == true)>
              ${PWM_SwitchOn}(pwmcHandle[M1]);
  </#if><#-- MC.M1_OTF_STARTUP == true -->
  <#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
              BADC_SetDirection(&Bemf_ADC_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));
  <#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE = false -->
              HALL_SetDirection(&HALL_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
              SIX_STEP_Clear(M1);

  <#if MC.M1_SPEED_SENSOR == "HALL_SENSOR">
    <#if MC.CURRENT_LIMITER_OFFSET == true>
#if (PID_SPEED_INTEGRAL_INIT_DIV == 0)
              PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
              PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                 (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                 / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
    </#if><#-- MC.CURRENT_LIMITER_OFFSET == true -->
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
              Mci[M1].State = RUN;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
              (void)SixStep_StepCommution();
  <#else><#-- sensorless mode only -->
              Mci[M1].State = START;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
  </#if><#-- MC.M1_SPEED_SENSOR == "HALL_SENSOR" -->
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }
</#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->
<#if MC.M1_OTF_STARTUP == true>

      case OTF_DETECTION:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {       
            if (TSK_OTFDetectionDelayHasElapsedM1())
            {
              OTF_6S_Clear(&OTF_M1 );
              TSK_SetBrakeDelayM1(M1_BRAKE_TICKS);
              Mci[M1].State = OTF_BRAKE;		  
            }
            else
            {
              if (true == OTF_6S_IsOngoing(&OTF_M1))
              {
                /* Nothing to do */              
              }
              else
              {
                if (true == OTF_6S_IsAborted(&OTF_M1))
                {
                  OTF_6S_Clear(&OTF_M1 );
                  TSK_SetBrakeDelayM1(M1_BRAKE_TICKS);
                  Mci[M1].State = OTF_BRAKE;			  
                }
                else
                {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                  PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
                  PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                      (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                       / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
                  /* USER CODE BEGIN MediumFrequencyTask M1 1 */
                  
                  /* USER CODE END MediumFrequencyTask M1 1 */
                  
                  SDC_SetSpeedSensor(pSDC[M1], &Bemf_ADC_M1._Super);
                  int16_t tTargetSpeed = SDC_GetMecSpeedRefUnit(pSDC[M1]);
                  SDC_ForceSpeedReferenceToCurrentSpeed(pSDC[M1]); /* Init the reference speed to current speed */
                  MCI_ExecSpeedRamp(&Mci[M1],
                                    tTargetSpeed,M1_OTF_RAMP_DURATION); /* First command to SDC */  
                  MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                  Mci[M1].State = RUN;
                }
              }
            }
          }
          break;
        }

      case OTF_BRAKE:
        {
          if (TSK_BrakeDelayHasElapsedM1())
          {
            RUC_6S_UpdatePulse(&RevUpControlM1, &BusVoltageSensor_M1._Super);
            BADC_SetDirection(&Bemf_ADC_M1, MCI_GetImposedMotorDirection( &Mci[M1]));
            
            PWMC_SwitchOffPWM(pwmcHandle[M1]);
            SIX_STEP_Clear( M1 );
            
            Mci[M1].State = START;
            PWMC_SwitchOnPWM(pwmcHandle[M1]);
            
          }
          else
          {
            /* Nothing to be done */
          }
          break;
        }
</#if><#-- MC.M1_OTF_STARTUP == true-->
<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            if ((!SDC_GetOpenLoopFlag(pOLS[M1])) || ((SDC_GetOpenLoopFlag(pOLS[M1]) && SDC_GetRevUpFlag(pOLS[M1]))))
            {
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
  <#if MC.M1_IPD_STARTUP == true>
            if (IPD_6S_GetIPDStartUpFlag(&IPD_M1))
            {
              if (IPD_6S_Task(&IPD_M1,&PWM_Handle_M1))
              {
                /* Stay in START state. */
              }
              else
              {
                Mci[M1].State = RUN;
              }
            }
            else
            {
  </#if><#-- MC.M1_IPD_STARTUP == true -->
              /* Execute the IPD procedure. */
              /* Execute the Rev Up procedure. */
              if(! RUC_6S_Exec(&RevUpControlM1))
              {
                /* The time allowed for the startup sequence has expired */
                MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
              }
              else
              {
              /* Rotor alignment */
                if (true == RUC_6S_IsAlignStageNow(&RevUpControlM1)) 
                {
                  PWMC_SetPhaseVoltage(pwmcHandle[M1], SixStepVars[M1].DutyCycleRef);
                  PWMC_LoadNextStep(&PWM_Handle_M1);
                }
                else
                {
                  /* Nothing to do */
                }
        
                /* Execute the open loop start-up ramp:
                 * Compute the duty cycle reference as configured in the Rev Up sequence */
                (void) BADC_CalcRevUpDemagTime (&Bemf_ADC_M1, SPD_GetAvrgMecSpeedUnit(&RevUpControlM1._Super));
                SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
              }
              
              /* Check that startup speed has reached the validation threshold*/
              if (true == RUC_6S_ObserverSpeedReached(&RevUpControlM1))
              {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                     / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
                /* USER CODE BEGIN MediumFrequencyTask M1 1 */
                
                /* USER CODE END MediumFrequencyTask M1 1 */
                        
                SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
                BADC_SetLoopClosed(&Bemf_ADC_M1);
                SDC_SetSpeedSensor(pSDC[M1], &Bemf_ADC_M1._Super);                                 
                SDC_ForceSpeedReferenceToCurrentSpeed(pSDC[M1]); /* Init the reference speed to current speed */
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
                if (SDC_GetOpenLoopFlag(pOLS[M1]))
                {
                  SDC_SetControlMode(pSDC[M1], MCM_DUTY_MODE);
                }
                else
                {
                  MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                }
  <#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE = false -->
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
                Mci[M1].State = RUN;
              }
              else
              {
                /* Nothing to do */
              }
  <#if MC.M1_IPD_STARTUP == true>
            }
  </#if><#-- M1_IPD_STARTUP == true -->
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            }
            else
            {
              SDC_SetControlMode(pSDC[M1], MCM_DUTY_MODE);
              SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
              PWMC_SetPhaseVoltage(pwmcHandle[M1], SixStepVars[M1].DutyCycleRef);
              BADC_SetLoopClosed(&Bemf_ADC_M1);
              SDC_SetSpeedSensor(pSDC[M1], &Bemf_ADC_M1._Super);
              SDC_ForceSpeedReferenceToCurrentSpeed(pSDC[M1]); /* Init the reference speed to current speed */
              Bemf_ADC_M1.SpeedTimerState = LFTIM_DEMAGNETIZATION;
              Mci[M1].State = RUN;
            }
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
          }
          break;
        }
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */
            
            /* USER CODE END MediumFrequencyTask M1 2 */

<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            if (SDC_GetOpenLoopFlag(pOLS[M1]))
            {
              SDC_SetControlMode(pSDC[M1], MCM_DUTY_MODE);
            }
            else
            {
  <#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true>
              SDC_SetControlMode(pSDC[M1], MCM_SPEED_MODE); /* To alolw to swapp from one to another control mode in real time. */
  <#else><#-- MC.M1_POTENTIOMETER_ENABLE = false || MC.M2_POTENTIOMETER_ENABLE = false -->
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
  </#if><#-- MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true -->
            }
<#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE = false -->
            MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
            SixStepVars[M1].DutyCycleRef = SDC_CalcSpeedReference(pSDC[M1]);
            /* Update PWM at this Medium frequency rate after compution to allow a better start at low speed. */
<#if MC.DRIVE_MODE == "VM">
            PWMC_SetPhaseVoltage(pwmcHandle[M1], SixStepVars[M1].DutyCycleRef);
<#else><#-- MC.DRIVE_MODE != "VM" -->
            PWMC_SetPhaseVoltage(pwmcHandle[M1], PWM_Handle_M1.StartCntPh);
            LL_TIM_OC_SetCompareCH1(${_last_word(MC.CURR_REF_TIMER_SELECTION)},SixStepVars[M1].DutyCycleRef); /* Set counter compare for current control. */
</#if><#-- MC.DRIVE_MODE == "VM" -->

<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
            (void) BADC_CalcRunDemagTime (&Bemf_ADC_M1);
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

<#if MC.M1_OTF_STARTUP == true>
            OTF_6S_UpdateDutyConv(&OTF_M1, SixStepVars[M1].DutyCycleRef);
</#if><#-- MC.M1_OTF_STARTUP == true-->

            if(!IsSpeedReliable)
            {
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  <#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
              if((!BADC_IsLoopClosed(&Bemf_ADC_M1)) && (!SDC_GetRevUpFlag(pOLS[M1])))
              {
                Bemf_ADC_M1._Super.bSpeedErrorNumber = 0;
              }
              else
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
  <#else><#-- MC.M1_SPEED_SENSOR != "SENSORLESS_ADC" -->
              MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
  </#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
  
<#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
              MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
            }
            else
            {
              /* Nothing to do */
            }
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {
<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
            BADC_Clear(&Bemf_ADC_M1);
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            SDC_ClearDutyCycleMean(pOLS[M1]);
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.M1_IPD_STARTUP == true>
            if (IPD_6S_GetIPDStartUpFlag(&IPD_M1))
            {
              IPD_6S_Clear(&IPD_M1);
            }
            else
            {
              /* Nothing to do. */
            }
</#if><#-- MC.M1_IPD_STARTUP == true -->
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */

            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        
        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }
    
        default:
          break;
       }
    }  
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }

<#if MC.M1_SPEED_SENSOR == "HALL_SENSOR">
  /* Perform the Regular conversion. */
  RCM_ExecNextConv();
  RCM_WaitForConv();
  RCM_ReadOngoingConv();
</#if><#-- MC.M1_SPEED_SENSOR == "HALL_SENSOR" -->

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */

}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void SIX_STEP_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN SixStep_Clear 0 */

  /* USER CODE END SixStep_Clear 0 */
  SDC_Clear(pSDC[bMotor]);
  SixStepVars[bMotor].DutyCycleRef = SDC_GetDutyCycleRef(pSDC[bMotor]);
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  BADC_Stop();
  BADC_Clear(&Bemf_ADC_M1);
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if  MC.CURRENT_LIMITER_OFFSET == true >
#if (PID_SPEED_INTEGRAL_INIT_DIV == 0)
  PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
  PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                      (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                      / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
</#if><#-- MC.CURRENT_LIMITER_OFFSET == true -->
  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  RUC_6S_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
  SDC_SetSpeedSensor(pSDC[M1], &RevUpControlM1._Super);
<#if MC.M1_IPD_STARTUP == true>
  IPD_6S_Clear(&IPD_M1);
</#if><#-- MC.M1_IPD_STARTUP == true -->
  Bemf_ADC_M1.SpeedTimerState = LFTIM_IDLE;
<#else><#-- MC.M1_SPEED_SENSOR != "SENSORLESS_ADC" -->
  HALL_Clear(&HALL_M1);
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if MC.DRIVE_MODE == "CM">
  LL_TIM_SetCounter(${_last_word(MC.CURR_REF_TIMER_SELECTION)}, 0u); /* Clear the counter. */
  <#if MC.CURRENT_LIMITER_OFFSET>
  LL_TIM_OC_SetCompareCH1(${_last_word(MC.CURR_REF_TIMER_SELECTION)},PWM_PERIOD_CYCLES_REF); /* Init compare couter value. */
  <#else>
  LL_TIM_OC_SetCompareCH1(${_last_word(MC.CURR_REF_TIMER_SELECTION)}, 0); /* Init to 0 compare couter value. */
  </#if>
</#if>

<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Clear(&PerfTraces,bMotor);
  </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
</#if><#-- DWT_CYCCNT_SUPPORTED -->
  /* USER CODE BEGIN SixStep_Clear 1 */

  /* USER CODE END SixStep_Clear 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

  <#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/**
  * @brief  This is the 6step commutation task. It configures the demagnetization period and 
  * manages ADC regular conversions when motor is both running and stopped.
  */
  
__weak void TSK_SpeedTIM_task()
{

  /* USER CODE BEGIN SpeedTimerTask 0 */

  /* USER CODE END SpeedTimerTask 0 */
<#if MC.M1_IPD_STARTUP == true>  
  if ((IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && IPD_6S_GetRunStateFlag(&IPD_M1))
  {
    Bemf_ADC_M1.ZCDetectionErrors = 0;
  }
  else
  {  
    if (Bemf_ADC_M1.MaxZCDetectionErrors > BEMF_ERRORS_SCORE)
    {
      Bemf_ADC_M1.MaxZCDetectionErrors = Bemf_ADC_M1.MaxZCDetectionErrors - 1U;
      Bemf_ADC_M1.ZCDetectionErrors = 0;
    }
    else
    {
      /* Nothing to do end of starup IPD phase. */
    } 
  }
</#if><#-- MC.M1_IPD_STARTUP == true -->

  if (true == BADC_CheckDetectionErrors(&Bemf_ADC_M1))
  {
    MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
  }
  else 
  {
    /* Nothing to do */
  }
  
  switch (Bemf_ADC_M1.SpeedTimerState)
  {
    case LFTIM_COMMUTATION:
    {
      BADC_StepChangeEvent(&Bemf_ADC_M1);
      (void)SixStep_StepCommution();
      RCM_ExecNextConv();
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
    if (SDC_GetOnSensing(pOLS[M1]))
      {
        BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1, &BusVoltageSensor_M1._Super);
      }
      else
      {
        BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1, &BusVoltageSensor_M1._Super);
        PWMC_SetADCTriggerChannel(&PWM_Handle_M1, *Bemf_ADC_M1.pSensing_Point);    
      }
    <#else>
      BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1, &BusVoltageSensor_M1._Super);
      PWMC_SetADCTriggerChannel(&PWM_Handle_M1, *Bemf_ADC_M1.pSensing_Point);    
    </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
      break;
    }

    case LFTIM_DEMAGNETIZATION:
    {
      RCM_ReadOngoingConv();
      if (false == Bemf_ADC_M1.IsLoopClosed) 
      {
        BADC_SetSpeedTimer(&Bemf_ADC_M1, RevUpControlM1.ElSpeedTimerDpp - Bemf_ADC_M1.DemagCounterThreshold);
      }
      else
      {
        /* Nothing to do, step commutation is scheduled only when bemf zero crossing is detected */    
      }
      BADC_Start(&Bemf_ADC_M1, PWM_Handle_M1.Step, PWM_Handle_M1.LSModArray);
      Bemf_ADC_M1.SpeedTimerState = LFTIM_COMMUTATION;
      break;
    }

    case LFTIM_IDLE:
    default:
    {
  <#if MC.M1_IPD_STARTUP == true>
      if((START == Mci[M1].State) && (IPD_6S_GetIPDStartUpFlag(&IPD_M1)) && (IPD_6S_GetIPDPulseRunStateFlag(&IPD_M1)))
      {
        /* No Regular conversion during IPD. */
      }
      else
      {
   </#if><#-- MC.M1_IPD_STARTUP == true -->
      RCM_ExecNextConv();
      RCM_WaitForConv();
      RCM_ReadOngoingConv();
    <#if MC.M1_IPD_STARTUP == true>
      }
    </#if><#-- MC.M1_IPD_STARTUP == true -->
      BADC_SetSpeedTimer(&Bemf_ADC_M1, SPEED_TIMER_IDLE_RATE_TICKS);
  <#if MC.M1_OTF_STARTUP == true>
      if (false == OTF_6S_IsOngoing(&OTF_M1))
      {
        if (false == RUC_6S_IsAlignStageNow(&RevUpControlM1)) 
        {
          Bemf_ADC_M1.SpeedTimerState = LFTIM_COMMUTATION;
        }
        else
        {}
      }
  <#else>
      if (false == RUC_6S_IsAlignStageNow(&RevUpControlM1))
      {
       Bemf_ADC_M1.SpeedTimerState = LFTIM_COMMUTATION;
      }
      else
      {} 
  </#if><#-- MC.M1_OTF_STARTUP == true-->
      break;
    }
  }

  /* USER CODE BEGIN SpeedTimerTask 1 */

  /* USER CODE END SpeedTimerTask 1 */ 
} 
  <#if MC.M1_OTF_STARTUP == true>
/**
  * @brief  This is the BEMF zero crossing detection task. It detects the zero crossing event and
  * configures the speed timer to schedule the next step commutation.
  */
void TSK_BEMF_ZCD_Task()
{
  if ((false == OTF_6S_IsAborted(&OTF_M1)) && (true == OTF_6S_IsOngoing(&OTF_M1))) 
  {
    if (true == OTF_6S_Task(&OTF_M1))
    {
      Bemf_ADC_M1.SpeedTimerState = LFTIM_COMMUTATION;
      BADC_SetLoopClosed(&Bemf_ADC_M1);  
      (void) BADC_CalcRunDemagTime(&Bemf_ADC_M1);
      SixStepVars[M1].DutyCycleRef = OTF_6S_CalcSpeedReference(&OTF_M1);
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
  }
}
  </#if><#-- MC.M1_OTF_STARTUP == true-->
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

<#if MC.M1_SPEED_SENSOR == "HALL_SENSOR">
/**
  * @brief  This is the Hall sensors commutation task. It configures
  * the speed timer for an immediate or delayed step commutation.
  */
void TSK_SpeedTIM_task(void)
{
  (void)HALL_TIMx_CC_IRQHandler(&HALL_M1);

  if ((RUN == Mci[M1].State) && ((0U == HALL_M1.PhaseShift) || (HALL_M1.PhaseShift >= 30U)))
  {
    (void)SixStep_StepCommution();
  }
  else
  {
    /* Nothing to do. */
  }
}

</#if><#-- MC.M1_SPEED_SENSOR == "HALL_SENSOR" -->
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
inline void SixStep_StepCommution(void)
{
<#if MC.DRIVE_MODE == "VM">
  PWMC_SetPhaseVoltage(pwmcHandle[M1], SixStepVars[M1].DutyCycleRef);
<#else><#-- MC.DRIVE_MODE != "VM" -->
  PWMC_SetPhaseVoltage(pwmcHandle[M1], PWM_Handle_M1.StartCntPh);
  LL_TIM_OC_SetCompareCH1(${_last_word(MC.CURR_REF_TIMER_SELECTION)},SixStepVars[M1].DutyCycleRef); /* Set counter compare for current control. */
</#if><#-- MC.DRIVE_MODE == "VM" -->
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  PWMC_ForceNextStep(&PWM_Handle_M1, RUC_6S_GetDirection(&RevUpControlM1), 0u);
<#elseif MC.M1_SPEED_SENSOR == "HALL_SENSOR">
  PWMC_ForceNextStep(&PWM_Handle_M1, 0, HALL_GetStep(&HALL_M1));
  LL_TIM_SetPrescaler(HALL_M1.TIMx, HALL_M1.Prescaler); /* To avoid update at CCH2 event instead of Hall event. */
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
  PWMC_LoadNextStep(&PWM_Handle_M1);
<#if MC.M1_OTF_STARTUP == true>
  if (true == OTF_6S_IsOngoing(&OTF_M1))
  { 
    OTF_6S_SwitchOver(&OTF_M1);
  }
</#if><#-- MC.M1_OTF_STARTUP == true-->
}


<#if MC.M1_IPD_STARTUP == true>
void IPD_6step_BemfStart(void)
{
  BADC_SetLoopClosed(&Bemf_ADC_M1);                                               /* Start the BEMF measurements as in Loop closed. */
  SDC_SetSpeedSensor(pSDC[M1], &Bemf_ADC_M1._Super);                              /* Speed based measure. */
  SDC_SetControlMode(Mci[M1].pSDC, MCM_DUTY_MODE);                                /* Set MCM_DUTY_MODE. */
  RevUpControlM1.hDirection = ((Mci[M1].hFinalSpeed < 0) ? 0xFF : 1);             /* Direction for SixStep_StepCommution. */
  BADC_SetDirection(&Bemf_ADC_M1, RevUpControlM1.hDirection);
  uint32_t startSpeed = (((MAX_APPLICATION_SPEED_RPM / 2U) < DEFAULT_TARGET_SPEED_RPM) ? (((MAX_APPLICATION_SPEED_RPM / 2U) * SPEED_UNIT) / U_RPM) : DEFAULT_TARGET_SPEED_UNIT);
  if (1U != RevUpControlM1.hDirection)
  {
    startSpeed = (-startSpeed);
    startSpeed = ((startSpeed < Mci[M1].hFinalSpeed) ? Mci[M1].hFinalSpeed : startSpeed);
  }
  else
  {
    startSpeed = ((startSpeed > Mci[M1].hFinalSpeed) ? Mci[M1].hFinalSpeed : startSpeed);
  }
  pSDC[M1]->SpeedRefUnitExt = (int32_t)(startSpeed) * (int32_t)65536;             /* Mechanical rotor speed reference. */
  (void)SDC_ExecRamp(Mci[M1].pSDC, (int16_t)Mci[M1].hFinalSpeed, 20);             /* Start with maximum speed set up. */
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_DUTY_CTRL
  SixStepVars[M1].DutyCycleRef = ((Mci[M1].hFinalSpeed < 0) ? (-(pSDC[M1]->DutyCycleRef >> 16)) : (pSDC[M1]->DutyCycleRef >> 16));
#else
  SixStepVars[M1].DutyCycleRef = ((Mci[M1].hFinalSpeed < 0) ? (-(pSDC[M1]->DutyCycleRef / 65536)) : (pSDC[M1]->DutyCycleRef / 65536));
#endif
  Bemf_ADC_M1.Last_TimerSpeed_Counter =  LL_TIM_GetCounter(ADC_TIMER_TRIGGER);    /* Reset the timerSpeed base. */
  Bemf_ADC_M1.SpeedTimerState = LFTIM_COMMUTATION;                                /* Start the BEMF reading process. */
  Bemf_ADC_M1._Super.bMaximumSpeedErrorsNumber = 255;                             /* Maximum value to prevent any speed measuremnt errors at startup IPd phase. */
  Bemf_ADC_M1.MaxZCDetectionErrors = 255;                                         /* Maximum value to prevent any speed measuremnt errors at startup IPd phase. */
}
</#if><#-- MC.M1_IPD_STARTUP == true -->


/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
