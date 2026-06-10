<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_macro.ftl">
/**
  ******************************************************************************
  * @file    mc_config.c 
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */ 

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */ 

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
<#if MC.DRIVE_MODE == "VM">
  .wUpperIntegralLimit = (int32_t)(PERIODMAX * SP_KIDIV),
  .wLowerIntegralLimit = 0,
  .hUpperOutputLimit   = (int16_t)PERIODMAX,
  .hLowerOutputLimit   = 0,
<#else>
  .wUpperIntegralLimit = (int32_t)(PERIODMAX_REF * SP_KIDIV),
  .wLowerIntegralLimit = 0,
  .hUpperOutputLimit   = (int16_t)PERIODMAX_REF,
  .hLowerOutputLimit   = 0,
</#if><#-- MC.DRIVE_MODE == "VM" -->
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
/**
  * @brief  Openloop sixstep Controller parameters Motor 1.
  */
OpenLoopSixstepCtrl_Handle_t OpenLoopSixstepCtrllM1 =
{
  .DutyCycleRefMean           = 0U,                               /* Computed DutyCycleRef mean value. */
  .CurrentFactor              = 10U,                              /* Curent factor for openloop speed control % of max DutyCycle. */
  .VoltageFactor              = 85U,                              /* Voltage factor for openloop speed control % of max DutyCycle. */
  .DutyCycleRef               = 20U,                              /* DutyCycleRef  duty cycle for PWM timer. */
  .DutyCycleRefFilter         = M1_OPENLOOP_DIGITAL_FILTER_SHIFT, /* Shif value of the digital filter. */
  .Openloop                   = true,                             /* Openloop flag. */
  .RevUp                      = 1,                                /* RevUp enabling flag. */
  .OnSensing                  = 0,                                /* OnSensing enabling flag. */
};
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->

static SpeednDutyCtrl_Handle_t SpeednDutyCtrlM1 =
{
  .Mode                       = DEFAULT_CONTROL_MODE,                         /* Changed during RevUp phase */
  .TargetFinal                = 0,                                            /* Will be updated with SDC_ExecRamp and SDC_CalcSpeedReference. */
  .SpeedRefUnitExt            = (int32_t)(DEFAULT_TARGET_SPEED_UNIT) * 65536, /* Mechanical rotor speed reference. */
  .DutyCycleRef               = 0U * 65536U,                                  /* Will be updated during RevUp phase. */
  .RampRemainingStep          = 0U,
  .PISpeed                    = &PIDSpeedHandle_M1,                            /* Speed PID pointeur. */
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  .SPD                        = &RevUpControlM1._Super,                        /* Speed sensor pointer. */
<#else><#-- HAL_SENSING -->
  .SPD                        = &HALL_M1._Super,                               /* Speed sensor pointer. */
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC"  --> 
  .IncDecAmount               = 0U,
  .SDCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
<#if MC.DRIVE_MODE == "VM">
  .MaxPositiveDutyCycle       = (uint16_t)PERIODMAX / 2U,
<#else><#-- MC.DRIVE_MODE != "VM" -->
  .MaxPositiveDutyCycle       = (uint16_t)PERIODMAX_REF,
</#if><#-- MC.DRIVE_MODE == "VM" -->
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
<#if MC.CURRENT_LIMITER_OFFSET>
  .DutyCycleRefDefault        = (uint16_t)PERIODMAX_REF,
<#else><#-- != MC.CURRENT_LIMITER_OFFSET -->
  .DutyCycleRefDefault        = 0U,
</#if><#-- MC.CURRENT_LIMITER_OFFSET -->
};

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC")>
RevUpCtrl_6S_Handle_t RevUpControlM1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .speedConvFactor           = SPEED_TIMER_CONV_FACTOR,
  },
  
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT,
  .pSDC                    = &SpeednDutyCtrlM1,
  
<#if MC.DRIVE_MODE == "VM">
  .ParamsData  = 
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),
    (uint16_t)PHASE1_VOLTAGE_DPP,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),
    (uint16_t)PHASE2_VOLTAGE_DPP,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),
    (uint16_t)PHASE3_VOLTAGE_DPP,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),
    (uint16_t)PHASE4_VOLTAGE_DPP,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),
    (uint16_t)PHASE5_VOLTAGE_DPP,(void*)MC_NULL},
  },
<#elseif MC.DRIVE_MODE == "CM">
  .ParamsData =
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),
    PHASE1_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),
    PHASE2_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),
    PHASE3_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),
    PHASE4_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),
    PHASE5_FINAL_CURRENT_REF,(void*)MC_NULL},
  },
<#else><#-- MC.DRIVE_MODE != "VM" ||  MC.DRIVE_MODE != "CM" -->
  .ParamsData =
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),(uint16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),(uint16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),(uint16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),(uint16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),(uint16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
  },
</#if><#-- MC.M1_DRIVE_TYPE == "SIX_STEP" &&  MC.DRIVE_MODE == "VM" -->
};
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") -->

<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
Bemf_ADC_Handle_t Bemf_ADC_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .speedConvFactor           = SPEED_TIMER_CONV_FACTOR,
  },

  .Pwm_H_L =
  {
    .AdcThresholdPwmPerc       = 10 * BEMF_THRESHOLD_PWM_PERC,
    .AdcThresholdHighPerc      = 10 * BEMF_THRESHOLD_HIGH_PERC,
    .AdcThresholdLowPerc       = 10 * BEMF_THRESHOLD_LOW_PERC,
    .Bus2ThresholdConvFactor   = BEMF_BUS2THRES_FACTOR,
    .ThresholdCorrectFactor    = BEMF_CORRECT_FACTOR,
    .SamplingPointOff          = BEMF_ADC_TRIG_TIME,
  <#if MC.DRIVE_MODE == "VM">
    .SamplingPointOn           = BEMF_ADC_TRIG_TIME_ON,
  </#if><#-- MC.DRIVE_MODE == "VM" -->	
  <#if  CondFamily_STM32G4>
    .AWDfiltering              = ADC_AWD_FILTER_NUMBER + 1,
  </#if><#-- CondFamily_STM32G4 -->	
  },
  <#if  MC.DRIVE_MODE == "VM">
  .OnSensingEnThres            = BEMF_PWM_ON_ENABLE_THRES,
  .OnSensingDisThres           = BEMF_PWM_ON_DISABLE_THRES,
  <#else><#-- MC.DRIVE_MODE != "VM" -->
  .OnSensingEnThres            = PWM_PERIOD_CYCLES,
  .OnSensingDisThres           = PWM_PERIOD_CYCLES,
  </#if><#-- MC.DRIVE_MODE == "VM" -->
  .ComputationDelay            = (uint8_t) (- COMPUTATION_DELAY),
  .ZcRising2CommDelay          = ZCD_RISING_TO_COMM_9BIT,
  .ZcFalling2CommDelay         = ZCD_FALLING_TO_COMM_9BIT,
  .SpeedBufferSize             = BEMF_AVERAGING_FIFO_DEPTH,
  .StartUpConsistThreshold     = NB_CONSECUTIVE_TESTS,
  .DemagParams =
  {
    .DemagMinimumSpeedUnit     = DEMAG_MINIMUM_SPEED,
    .RevUpDemagSpeedConv       = DEMAG_REVUP_CONV_FACTOR,
    .RunDemagSpeedConv         = DEMAG_RUN_CONV_FACTOR,
    .DemagMinimumThreshold     = MIN_DEMAG_COUNTER_TIME,
  },
  .MaxZCDetectionErrors        = BEMF_ERRORS_SCORE,
  .DriveMode                   = DEFAULT_DRIVE_MODE,
};
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

PWMC_Handle_t PWM_Handle_M1 =
{
  <#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" && MC.DRIVE_MODE == "CM">
  .StartCntPh           = (uint16_t) (0.8*BEMF_ADC_TRIG_TIME),
  <#else><#-- MC.DRIVE_MODE == "CM" -->
  .StartCntPh           = PWM_PERIOD_CYCLES,
  </#if><#-- MC.DRIVE_MODE == "CM" -->
  .PWMperiod            = PWM_PERIOD_CYCLES,
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  .AlignStep            = ALIGN_STEP,
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
  .OverCurrentFlag      = false,
  .OverVoltageFlag      = false,
  .driverProtectionFlag = false,
  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
  .TimerCfg             = &ThreePwm_TimerCfgM1,
  .QuasiSynchDecay      = false,
  <#else>
  .TimerCfg             = &SixPwm_TimerCfgM1,
    <#if  MC.QUASI_SYNC>
  .QuasiSynchDecay      = true,
    <#else>
  .QuasiSynchDecay      = false,
    </#if><#-- MC.QUASI_SYNC -->
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" --> 
    <#if  MC.FAST_DEMAG>
  .LSModArray           = {1,0,1,0,1,0},
    <#else>
  .LSModArray           = {0,0,0,0,0,0},
    </#if><#-- MC.QUASI_SYNC -->
  .pParams_str          = &PWMC_ParamsM1,
  .LowSideOutputs       = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
  .TurnOnLowSidesAction = false,
};

<#if (MC.M1_SPEED_SENSOR == "HALL_SENSOR")>
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL.
  */
HALL_6S_Handle_t HALL_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = 0,
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .speedConvFactor           = SPEED_TIMER_CONV_FACTOR,
  },

  .SensorPlacement             = HALL_SENSORS_PLACEMENT,
  .PhaseShift                  = PHASE_SHIFT_DEG,
  .StepShift                   = STEP_SHIFT,
  .SpeedBufferSize             = HALL_AVERAGING_FIFO_DEPTH,
  .TIMClockFreq                = HALL_TIM_CLK,
  .TIMx                        = ${_last_word(MC.M1_HALL_TIMER_SELECTION)},
  .ICx_Filter                  = M1_HALL_IC_FILTER_LL,
  .H1Port                      = M1_HALL_H1_GPIO_Port,
  .H1Pin                       = M1_HALL_H1_Pin,
  .H2Port                      = M1_HALL_H2_GPIO_Port,
  .H2Pin                       = M1_HALL_H2_Pin,
  .H3Port                      = M1_HALL_H3_GPIO_Port,
  .H3Pin                       = M1_HALL_H3_Pin,
  .Direction                   = 1,
};

</#if><#-- (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->
<#if MC.M1_OTF_STARTUP> 
OTF_6S_Handle_t OTF_M1 =
{
  .pRevUp                = &RevUpControlM1,
  .pPwmc                 = &PWM_Handle_M1,
  .pBADC                 = &Bemf_ADC_M1,
  .maxConsecutiveBemfTransitions = NB_CONSECUTIVE_TESTS,
  .maxBemfErrors         = M1_OTF_MAX_BEMF_ERRORS,
  .OTFabort              = true,
  .speedTimerPsc         = LF_TIMER_PSC,
  .pSensing_ThresholdPerc= M1_OTF_BEMF_THRESHOLD_PERC,
  .LSDetectCnt           = ((uint32_t) (M1_OTF_LS_DETECT_PERC * PWM_PERIOD_CYCLES / 100)),
  .LSBrakeCnt            = ((uint32_t) (M1_OTF_LS_BRAKE_PERC * PWM_PERIOD_CYCLES / 100)),
  .speedDutyConvFactor   = SPEED_DUTY_FACTOR_DEFAULT,
  .TimerCfg              = &OTFTimerCfgM1,  
  .LowPassFilterBW       = M1_OTF_SPEED_DUTY_CYCLE_RATIO,
};
</#if><#-- MC.M1_OTF_STARTUP == true-->
<#if MC.M1_IPD_STARTUP == true>
IPD_6S_Handle_t IPD_M1 =
{
  .IPDStartUpFlag                 = true,
  .OnePulsePwmTimerDutyCycle      = IPD_PWM_DUTY_CYCLE,
  .OnePulsePwmTimerFrequency      = M1_IPD_PWM_FREQ_HZ,
  .IpdPwmPeriodCycle              = IPD_PWM_PERIOD_CYCLES,
  .ValideBemfDetectedThreshold    = M1_IPD_BEMF_RUN_THREHOLD,
  .NumZeroSpeedSamples            = 0U,
  .NumZeroSpeedStepValidThreshold = 1U,
  .NumZeroSpeedStepValid          = 0U,
  .previous_angle                 = 0U,
  .PreviousStep                   = 0,
  .IPDRunning                     = false,
  .IPDBEMFMeasured                = true,
  .IPDPulseRunState               = false,
  .IPDDebug                       = false,
  .ADC_JDR_Currents               = {0,0,0,0,0,0},
  .Flux_6_Steps                   = {0,0,0,0,0,0,0,0,0,0,0,0}
};
</#if><#-- MC.M1_IPD_STARTUP == true-->
SixStepVars_t SixStepVars[NBR_OF_MOTORS];
<#if MC.DRIVE_NUMBER != "1">
SpeednDutyCtrl_Handle_t *pSDC[NBR_OF_MOTORS]    = {&SpeednDutyCtrlM1};
PID_Handle_t *pPIDIq[NBR_OF_MOTORS]             = {&PIDIqHandle_M1 ,&PIDIqHandle_M2};
PID_Handle_t *pPIDId[NBR_OF_MOTORS]             = {&PIDIdHandle_M1 ,&PIDIdHandle_M2};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1 ,&TempSensor_M2};
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS]  = {&PQD_MotorPowMeasM1,&PQD_MotorPowMeasM2};   
  <#if MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true>
PosCtrl_Handle_t *pPosCtrl[NBR_OF_MOTORS]       = {<#if MC.M1_POSITION_CTRL_ENABLING >&PosCtrlM1<#else>MC_NULL</#if>,
                                                   <#if MC.M2_POSITION_CTRL_ENABLING>&PosCtrlM2<#else>MC_NULL</#if>};
  </#if><#-- MC.M1_POSITION_CTRL_ENABLING == false && MC.M2_POSITION_CTRL_ENABLING == false -->
  <#if MC.M1_FLUX_WEAKENING_ENABLING == true || MC.M2_FLUX_WEAKENING_ENABLING == true>
FW_Handle_t *pFW[NBR_OF_MOTORS]                 = {<#if MC.M1_FLUX_WEAKENING_ENABLING>&FW_M1<#else>MC_NULL</#if>,
                                                   <#if MC.M2_FLUX_WEAKENING_ENABLING>&FW_M2<#else>MC_NULL </#if>};
  </#if><#-- MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true -->
  <#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
FF_Handle_t *pFF[NBR_OF_MOTORS]                 = {<#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING>&FF_M1<#else>MC_NULL</#if>,
                                                <#if MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING>&FF_M2<#else>MC_NULL </#if>};
  </#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
<#else><#-- MC.DRIVE_NUMBER == 1 -->
SpeednDutyCtrl_Handle_t *pSDC[NBR_OF_MOTORS]    = {&SpeednDutyCtrlM1};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
OpenLoopSixstepCtrl_Handle_t *pOLS[NBR_OF_MOTORS] = {&OpenLoopSixstepCtrllM1};
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

MCI_Handle_t Mci[NBR_OF_MOTORS] =
{
  {
    .pSDC = &SpeednDutyCtrlM1,
    .pSixStepVars = &SixStepVars[0],
<#if MC.M1_POSITION_CTRL_ENABLING == true>
    .pPosCtrl = &PosCtrlM1,
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
    .pPWM = &PWM_Handle_M1, 
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M1,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY, 
  },

<#if MC.DRIVE_NUMBER != "1">
  {
    .pSTC = &SpeednTorqCtrlM2, 	
    .pSixStepVars = &SixStepVars[1],
<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
    .pVSS = &VirtualSpeedSensorM2,
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.M2_POSITION_CTRL_ENABLING == true>
    .pPosCtrl = &PosCtrlM2,
</#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
    .pPWM = &PWM_Handle_M2,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },
</#if><#-- MC.DRIVE_NUMBER == true -->
};

<#if MC.M1_CURRENT_MONITOR_READING == true>
/**
  * Current monitor parameters Motor 1.
  */
CurrMonitor_t CurrMonitor_M1 =
{
  .regADC                   = ${MC.M1_CUR_MON_ADC},
  .channel                  = MC_${MC.M1_CUR_MON_CHANNEL},
  .samplingTime             = M1_CUR_MON_ADC_SAMPLING_TIME,
  .currentConvFactor        = CURRENT_MONITOR_CONV_FACTOR,
  .samplingPointConvFact    = M1_CUR_MON_SAMPLING_CONV,
  .samplingDistance2Edge    = M1_CUR_MON_ADC_SAMPLING_TIME_NS,
  .hLowPassFilterBW         = 8,
};

</#if><#-- MC.M1_CURRENT_MONITOR_READING == true -->

<#if MC.M1_ICL_ENABLED == true>
ICL_Handle_t ICL_M1 =
{
  .ICLstate                  = ICL_ACTIVE,
  .hICLTicksCounter          = M1_ICL_CAPS_CHARGING_DELAY_TICKS,
  .hICLSwitchDelayTicks      = M1_ICL_RELAY_SWITCHING_DELAY_TICKS,
  .hICLChargingDelayTicks    = M1_ICL_CAPS_CHARGING_DELAY_TICKS,
  .hICLVoltageThreshold      = M1_ICL_VOLTAGE_THRESHOLD_VOLT,
  .hICLUnderVoltageThreshold = UD_VOLTAGE_THRESHOLD_V,
};

DOUT_handle_t ICLDOUTParamsM1 =
{
  .OutputState      = INACTIVE,
  .hDOutputPort     = M1_ICL_SHUT_OUT_GPIO_Port,
  .hDOutputPin      = M1_ICL_SHUT_OUT_Pin,
  .bDOutputPolarity = ${MC.M1_ICL_DIGITAL_OUTPUT_POLARITY}    
};
</#if><#-- MC.M1_ICL_ENABLED == true -->
/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */ 

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/

