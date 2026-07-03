/**
  ******************************************************************************
  * @file    r3_1_f30x_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used.
  *          
  *          It is specifically designed for STM32F30X
  *          microcontrollers and implements the successive sampling of two motor
  *          current using only one ADC.
  *           + MCU peripheral and handle initialization function
  *           + three shunt current sensing
  *           + space vector modulation function
  *           + ADC sampling function
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
  *
  * @ingroup R3_1_F30X_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r3_1_f30x_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */



/** @addtogroup ConvFOC
  * @{
  */

/** @addtogroup MCLLAPI
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup R3_1_pwm_curr_fdbk R3 1 ADC PWM & Current Feedback
 *
 * @brief 3-Shunt, 1 ADC, PWM & Current Feedback implementation for F0XX, F30X, F4XX, F7XX, G0XX, G4XX, H5XX, C0XX and L4XX MCUs.
 *
 * This component is used in applications based on F0XX, F30X, F4XX, F7XX, G0XX, G4XX, H5XX, C0XX and L4XX MCUs, using a three
 * shunt resistors current sensing topology and 1 ADC peripheral to acquire the current
 * values.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))
#define CCMR2_CH4_DISABLE 0x8FFFu   //

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void R3_1_TIMxInit( TIM_TypeDef * TIMx, PWMC_Handle_t * pHdl );
static void R3_1_ADCxInit( ADC_TypeDef * ADCx );
__STATIC_INLINE uint16_t R3_1_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg  );
static void R3_1_HFCurrentsPolarizationAB( PWMC_Handle_t * pHdl,ab_t * Iab );
static void R3_1_HFCurrentsPolarizationC( PWMC_Handle_t * pHdl, ab_t * Iab );
static void R3_1_SetAOReferenceVoltage( uint32_t DAC_Channel, uint16_t hDACVref );
uint16_t R3_1_SetADCSampPointPolarization( PWMC_Handle_t * pHdl) ;
static void R3_1_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
static void R3_1_RLTurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );
static void R3_1_RLSwitchOnPWM( PWMC_Handle_t * pHdl );


/**
  * @brief  Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  *         in three shunt topology one ADC.
  * 
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void R3_1_Init( PWMC_R3_1_Handle_t * pHandle )
{
  COMP_TypeDef * COMP_OCPAx = pHandle->pParams_str->CompOCPASelection;
  COMP_TypeDef * COMP_OCPBx = pHandle->pParams_str->CompOCPBSelection;
  COMP_TypeDef * COMP_OCPCx = pHandle->pParams_str->CompOCPCSelection;
  COMP_TypeDef * COMP_OVPx = pHandle->pParams_str->CompOVPSelection;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  /*Check that _Super is the first member of the structure PWMC_R3_1_Handle_t */
  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {
    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADCx );
    LL_ADC_ClearFlag_EOC( ADCx );
    LL_ADC_DisableIT_JEOC( ADCx );
    LL_ADC_ClearFlag_JEOC( ADCx );

    if ( TIMx == TIM1 )
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
    }
#if defined(TIM8)
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
    }
#endif
    /* Over current protection phase A */
    if ( COMP_OCPAx != NULL )
    {
      /* Inverting input*/
      if ( pHandle->pParams_str->CompOCPAInvInput_MODE != EXT_MODE )
      {
        if ( LL_COMP_GetInputMinus( COMP_OCPAx ) == LL_COMP_INPUT_MINUS_DAC1_CH1 )
        {
          R3_1_SetAOReferenceVoltage( LL_DAC_CHANNEL_1, ( uint16_t )( pHandle->pParams_str->DAC_OCP_Threshold ) );
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if ( LL_COMP_GetInputMinus( COMP_OCPAx ) == LL_COMP_INPUT_MINUS_DAC1_CH2 )
        {
          R3_1_SetAOReferenceVoltage( LL_DAC_CHANNEL_2, ( uint16_t )( pHandle->pParams_str->DAC_OCP_Threshold ) );
        }
#endif
        else
        {
        }
      }
      /* Output */
      LL_COMP_Enable ( COMP_OCPAx );
      LL_COMP_Lock( COMP_OCPAx );
    }

    /* Over current protection phase B */
    if ( COMP_OCPBx != NULL )
    {
      LL_COMP_Enable ( COMP_OCPBx );
      LL_COMP_Lock( COMP_OCPBx );
    }

    /* Over current protection phase C */
    if ( COMP_OCPCx != NULL )
    {
      LL_COMP_Enable ( COMP_OCPCx );
      LL_COMP_Lock( COMP_OCPCx );
    }

    /* Over voltage protection */
    if ( COMP_OVPx != NULL )
    {
      /* Inverting input*/
      if ( pHandle->pParams_str->CompOVPInvInput_MODE != EXT_MODE )
      {
        if ( LL_COMP_GetInputMinus( COMP_OVPx ) == LL_COMP_INPUT_MINUS_DAC1_CH1 )
        {
          R3_1_SetAOReferenceVoltage( LL_DAC_CHANNEL_1, ( uint16_t )( pHandle->pParams_str->DAC_OVP_Threshold ) );
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if ( LL_COMP_GetInputMinus( COMP_OVPx ) == LL_COMP_INPUT_MINUS_DAC1_CH2 )
        {
          R3_1_SetAOReferenceVoltage( LL_DAC_CHANNEL_2, ( uint16_t )( pHandle->pParams_str->DAC_OVP_Threshold ) );
        }
#endif
        else
        {
        }
      }
      /* Output */
      LL_COMP_Enable ( COMP_OVPx );
      LL_COMP_Lock( COMP_OVPx );
    }
    
    if (LL_ADC_IsEnabled (ADCx) == 0)
    {
      R3_1_ADCxInit (ADCx);
    }
    else 
    {
      /* Nothing to do ADCx_1 already configured */
    }
    R3_1_TIMxInit( TIMx, &pHandle->_Super );
    pHandle->ADCTriggerEdge = (uint16_t) LL_ADC_INJ_TRIG_EXT_RISING;
  }
}



/**
  * @brief Initializes @p ADCx peripheral for current sensing.
  * 
  * Specific to F30X and G4XX.
  * 
  * @param  ADCx: ADC instance peripheral
  */
static void R3_1_ADCxInit( ADC_TypeDef * ADCx )
{
     
  if ( LL_ADC_IsInternalRegulatorEnabled(ADCx) == 0u)
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADCx);
  
    /* Wait for Regulator Startup time */
    /* Note: Variable divided by 2 to compensate partially              */
    /*       CPU processing cycles, scaling in us split to not          */
    /*       exceed 32 bits register capacity and handle low frequency. */
    volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));      
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }
  
  LL_ADC_StartCalibration( ADCx, LL_ADC_SINGLE_ENDED );
  while ( LL_ADC_IsCalibrationOnGoing( ADCx) == 1u) 
  {}
  /* ADC Enable (must be done after calibration) */
  /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0 
  * following a calibration phase, could have no effect on ADC 
  * within certain AHB/ADC clock ratio.
  */
  while (  LL_ADC_IsActiveFlag_ADRDY( ADCx ) == 0u)  
  { 
    LL_ADC_Enable( ADCx );
  }

  /* Start and immediately stop ADC conversion to completely flush the JSQR queue of context */
  LL_ADC_INJ_StartConversion( ADCx );
  LL_ADC_INJ_StopConversion( ADCx );

  while( LL_ADC_INJ_IsStopConversionOngoing( ADCx ) )
  {
    /* Nothing to do */
  }
  
  /* Set trigger mode to hardware to avoid instant conversion */
  LL_ADC_INJ_SetTriggerEdge( ADCx, LL_ADC_INJ_TRIG_EXT_RISING );

  /* Start injected conversion */
  LL_ADC_INJ_StartConversion( ADCx );

  /* TODO: check if not already done by MX */
  LL_ADC_INJ_SetQueueMode( ADCx, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY );
 }

/**
  * @brief  It initializes TIMx peripheral for PWM generation.
  * 
  * Specific to F30X and G4XX.
  * 
  * @param TIMx: Timer to be initialized.
  * @param pHdl: Handler of the current instance of the PWM component.
  */
static void R3_1_TIMxInit( TIM_TypeDef * TIMx, PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  uint32_t Brk2Timeout = 1000;

  /* disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter( TIMx );
  
  /* set TRGO to update, this will flush the ADC JSQR
     when update will be sent during TIM init */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );
  /* Enables the TIMx Preload on CC4 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );
  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE( TIMx );
  if ( pHandle->pParams_str->FreqRatio == 2u )
  {
    if ( pHandle->pParams_str->IsHigherFreqTim == HIGHER_FREQ )
    {
      if ( pHandle->pParams_str->RepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter( TIMx, 1 );
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter( TIMx, 3 );
      }
    }
    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
  }
  else /* FreqRatio equal to 1 or 3 */
  {
    if ( pHandle->_Super.Motor == M1 )
    {
      if ( pHandle->pParams_str->RepetitionCounter == 1u )
      {
        LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
      }
      else if ( pHandle->pParams_str->RepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter( TIMx, 1 );
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter( TIMx, 3 );
      }
      else
      {
      }
    }
    else
    {
    }
  }

  /* set TRGO back to CH4 to trig ADC on CH4 match */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
  
  LL_TIM_ClearFlag_BRK( TIMx );
  while ((LL_TIM_IsActiveFlag_BRK2 (TIMx) == 1u) && (Brk2Timeout != 0u) )
  {
    LL_TIM_ClearFlag_BRK2( TIMx );
    Brk2Timeout--;
  }   
  LL_TIM_EnableIT_BRK( TIMx );
  /* Enable PWM channel */
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  /* clear Update flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE(TIMx);

}


/**
  * @brief  Stores in @p pHdl handler the calibrated @p offsets.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  offsets: Phase offset.
 */
__weak void R3_1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  pHandle->PhaseAOffset = offsets->phaseAOffset;
  pHandle->PhaseBOffset = offsets->phaseBOffset;
  pHandle->PhaseCOffset = offsets->phaseCOffset;
  pHdl->offsetCalibStatus = true;
}

/**
  * @brief Reads the calibrated @p offsets stored in @p pHdl.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  offsets: Phase offset.
  */
__weak void R3_1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  offsets->phaseAOffset = pHandle->PhaseAOffset;
  offsets->phaseBOffset = pHandle->PhaseBOffset;
  offsets->phaseCOffset = pHandle->PhaseCOffset;
}

/**
  * @brief  Stores into the @p pHdl the voltage present on Ia and Ib current 
  *         feedback analog channels when no current is flowing into the motor.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void R3_1_CurrentReadingPolarization( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  volatile PWMC_GetPhaseCurr_Cb_t GetPhaseCurrCbSave;
  volatile PWMC_SetSampPointSectX_Cb_t SetSampPointSectXCbSave;

  if (false == pHandle->_Super.offsetCalibStatus)
  {
    /* Save callback routines */
    GetPhaseCurrCbSave = pHandle->_Super.pFctGetPhaseCurrents;
    SetSampPointSectXCbSave = pHandle->_Super.pFctSetADCSampPointSectX;

    pHandle->PhaseAOffset = 0u;
    pHandle->PhaseBOffset = 0u;
    pHandle->PhaseCOffset = 0u;

    pHandle->PolarizationCounter = 0u;

    /* It forces inactive level on TIMx CHy and CHyN */
    LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);

    /* Offset calibration for all phases */
    /* Change function to be executed in ADCx_ISR */
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = &R3_1_HFCurrentsPolarizationAB;
    pHandle->_Super.pFctSetADCSampPointSectX = &R3_1_SetADCSampPointPolarization;
 
    pHandle->ADCTriggerEdge = (uint16_t) LL_ADC_INJ_TRIG_EXT_RISING;

    /* We want to polarize calibration Phase A and Phase B, so we select SECTOR_5 */
    pHandle->PolarizationSector=SECTOR_5;
    /* Required to force first polarization conversion on SECTOR_5*/
    pHandle->_Super.Sector = SECTOR_5;
    __enable_irq();
    R3_1_SwitchOnPWM( &pHandle->_Super );

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd( TIMx,
                            &pHandle->_Super.SWerror,
                            pHandle->pParams_str->RepetitionCounter,
                            &pHandle->PolarizationCounter );

    R3_1_SwitchOffPWM( &pHandle->_Super );

    /* Offset calibration for C phase */
    pHandle->PolarizationCounter = 0u;

    /* Change function to be executed in ADCx_ISR */
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = &R3_1_HFCurrentsPolarizationC;
 
    /* We want to polarize Phase C, so we select SECTOR_1 */
    pHandle->PolarizationSector=SECTOR_1;
    /* Required to force first polarization conversion on SECTOR_1*/
    pHandle->_Super.Sector = SECTOR_1;
    __enable_irq();
    R3_1_SwitchOnPWM( &pHandle->_Super );

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd( TIMx,
                            &pHandle->_Super.SWerror,
                            pHandle->pParams_str->RepetitionCounter,
                            &pHandle->PolarizationCounter );

    R3_1_SwitchOffPWM( &pHandle->_Super );
    pHandle->PhaseAOffset /= NB_CONVERSIONS;
    pHandle->PhaseBOffset /= NB_CONVERSIONS;
    pHandle->PhaseCOffset /= NB_CONVERSIONS;
    if (0U == pHandle->_Super.SWerror)
    {
      pHandle->_Super.offsetCalibStatus = true;
    }
    else
    {
      /* nothing to do */
    }

    /* restore function to be executed in ADCx_ISR */
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = GetPhaseCurrCbSave;
    pHandle->_Super.pFctSetADCSampPointSectX = SetSampPointSectXCbSave;
    __enable_irq();
  }

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
  LL_TIM_OC_SetCompareCH1 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH2 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH3 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

  /* At the end of calibration, all phases are at 50% we will sample A&B */
  pHandle->_Super.Sector=SECTOR_5;
  pHandle->_Super.BrakeActionLock = false;

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p Iab ab_t format.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
__weak void R3_1_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;  
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  uint8_t Sector;
  int32_t Aux;
  uint32_t ADCDataReg1;
  uint32_t ADCDataReg2;
  
  Sector = ( uint8_t )pHandle->_Super.Sector;
  ADCDataReg1 =  ADCx->JDR1;
  ADCDataReg2 =  ADCx->JDR2;
  
  /* disable ADC trigger source */
  //LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);  
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
  
  switch ( Sector )
  {
    case SECTOR_4:
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg2 );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    case SECTOR_6:
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }

      /* Ia = -Ic -Ib */
      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux -= ( int32_t )Iab->b;             /* Ia  */

      /* Saturation of Ia */
      if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }
      break;

    case SECTOR_2:
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = -Ic -Ia */
      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux -= ( int32_t )Iab->a;             /* Ib */

      /* Saturation of Ib */
      if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    default:
      break;
  }

  pHandle->_Super.Ia = Iab->a;
  pHandle->_Super.Ib = Iab->b;
  pHandle->_Super.Ic = -Iab->a - Iab->b;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p Iab ab_t format. Specific to overmodulation.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
__weak void R3_1_GetPhaseCurrents_OVM( PWMC_Handle_t * pHdl, ab_t * Iab )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  uint8_t Sector;
  int32_t Aux;
  uint32_t ADCDataReg1;
  uint32_t ADCDataReg2;

  /* disable ADC trigger source */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  Sector = ( uint8_t )pHandle->_Super.Sector;
  ADCDataReg1 = ADCx->JDR1;
  ADCDataReg2 = ADCx->JDR2;

switch ( Sector )
  {
    case SECTOR_4:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      if (pHandle->_Super.useEstCurrent == true)
      {
        // Ib not available, use estimated Ib
        Aux = ( int32_t )( pHandle->_Super.IbEst );
      }
      else
      {
        /* Ib = PhaseBOffset - ADC converted value) */
        Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg2 );
      }

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;
      
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      if (pHandle->_Super.useEstCurrent == true)
      {
        // Ia not available, use estimated Ia
        Aux = ( int32_t )( pHandle->_Super.IaEst );
      }
      else
      {
        Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );
      }

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg2 );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    case SECTOR_6:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }

      if (pHandle->_Super.useEstCurrent == true)
      {
        Aux =  ( int32_t ) pHandle->_Super.IcEst ; /* -Ic */
        Aux -= ( int32_t )Iab->b; 
      }
      else
      {
      /* Ia = -Ic -Ib */
        Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
        Aux -= ( int32_t )Iab->b;             /* Ia  */
      }
      /* Saturation of Ia */
      if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }
      break;
      
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      if (pHandle->_Super.useEstCurrent == true)
      {
        Aux = ( int32_t ) pHandle->_Super.IbEst;
      }
      else
      {
        Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg1 );
      }
      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }

      /* Ia = -Ic -Ib */
      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux -= ( int32_t )Iab->b;             /* Ia  */

      /* Saturation of Ia */
      if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }
      break;

    case SECTOR_2:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      if (pHandle->_Super.useEstCurrent == true)
      {
        Aux = ( int32_t ) pHandle->_Super.IaEst;
      }
      else
      {
        Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );
      }
      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = -Ic -Ia */
      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux -= ( int32_t )Iab->a;             /* Ib */

      /* Saturation of Ib */
      if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      if (pHandle->_Super.useEstCurrent == true)
      {
        /* Ib = -Ic -Ia */
        Aux = ( int32_t ) pHandle->_Super.IcEst; /* -Ic */
        Aux -= ( int32_t )Iab->a;             /* Ib */
      }
      else
      {
        /* Ib = -Ic -Ia */
        Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
        Aux -= ( int32_t )Iab->a;             /* Ib */
      }
      
      /* Saturation of Ib */
      if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    default:
      break;
  }

    pHandle->_Super.Ia = Iab->a;
    pHandle->_Super.Ib = Iab->b;
    pHandle->_Super.Ic = -Iab->a - Iab->b;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Configures the ADC for the current sampling during calibration.
  * 
  * Sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * The WriteTIMRegisters method is then called.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns the return value of R3_1_WriteTIMRegisters.
  */
uint16_t R3_1_SetADCSampPointPolarization( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  pHandle->_Super.Sector = pHandle->PolarizationSector;

  return R3_1_WriteTIMRegisters( &pHandle->_Super, ( pHandle->Half_PWMPeriod - (uint16_t) 1 ) );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  * 
  * Sets the ADC sequence length and channels and sets the sampling point via TIMx_Ch4 value and polarity.
  * The WriteTIMRegisters method is then called.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns the value of R3_1_WriteTIMRegisters.
  */
uint16_t R3_1_SetADCSampPointSectX( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

  uint16_t SamplingPoint;
  uint16_t DeltaDuty;

  /* Check if sampling the AB phases in the middle of the PWM period is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHdl->lowDuty ) > pHandle->pParams_str->Tafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are
     * differences between offsets */

    /* Sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds to 
     * sector 4 or 5 */
    pHandle->_Super.Sector = SECTOR_5;

    /* Set the sampling point in the middle of the PWM period */
    SamplingPoint = pHandle->Half_PWMPeriod - 1u;
  }
  else
  {
    /* In this case it is necessary to sample phases with maximum and variable complementary duty cycle */

    /* In every sector there is always one phase with maximum complementary duty, one with minimum 
     * complementary duty and one with variable complementary duty. In this case, phases with variable 
     * and maximum complementary duty are converted, always starting with the one with variable 
     * complementary duty cycle */

    DeltaDuty = ( uint16_t )( pHdl->lowDuty - pHdl->midDuty );

    /* Definition of crossing point */
    if ( DeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHdl->lowDuty ) * 2u )
    {
      SamplingPoint = pHdl->lowDuty - pHandle->pParams_str->Tbefore;
    }
    else
    {
      SamplingPoint = pHdl->lowDuty + pHandle->pParams_str->Tafter;

      if ( SamplingPoint >= pHandle->Half_PWMPeriod )
      {
         /* ADC trigger edge must be changed from positive to negative */
        pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_FALLING;
        SamplingPoint = ( 2u * pHandle->Half_PWMPeriod ) - SamplingPoint - (uint16_t) 1;
      }
    }
  }
  return R3_1_WriteTIMRegisters( &pHandle->_Super, SamplingPoint );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Configures the ADC for the current sampling related to sector X (X = [1..6] ) in case of overmodulation.
  * 
  * Sets the ADC sequence length and channels and sets the sampling point via TIMx_Ch4 value and polarity.
  * The WriteTIMRegisters method is then called.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns the value of R3_1_WriteTIMRegisters.
  */
uint16_t R3_1_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

  uint16_t SamplingPoint;
  uint16_t DeltaDuty;

  pHandle->_Super.useEstCurrent = false;
  DeltaDuty = ( uint16_t )( pHdl->lowDuty - pHdl->midDuty );

  /* Check if sampling the AB phases in the middle of the PWM period is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHdl->lowDuty ) > pHandle->pParams_str->Tafter )
  {
	  /* When it is possible to sample in the middle of the PWM period, always sample the same phases
	   * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
	   * between offsets */

	  /* Sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
	   * to sector 4 or 5  */
    pHandle->_Super.Sector = SECTOR_5;

    /* Set the sampling point in the middle of the PWM period */
    SamplingPoint =  pHandle->Half_PWMPeriod - 1u;
  }
  else
  {
    /* In this case it is necessary to sample phases with maximum and variable complementary duty cycle.*/

    /* In every sector there is always one phase with maximum complementary duty, one with minimum 
     * complementary duty and one with variable complementary duty. In this case, phases with variable
     * complementary duty and with maximum duty are converted and the first will be always the phase
     * with variable complementary duty cycle */

    if ( DeltaDuty >= pHandle->pParams_str->Tcase3 )
    {
      SamplingPoint = pHdl->lowDuty - pHandle->pParams_str->Tbefore;
    }
    else
    {
      /* case 2 (cf user manual) */
      if ( ( pHandle->Half_PWMPeriod - pHdl->lowDuty ) > pHandle->pParams_str->Tcase2 )
      {
        /* ADC trigger edge must be changed from positive to negative */
        pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_FALLING;
        SamplingPoint = pHdl->lowDuty + pHandle->pParams_str->Tbefore;
      }
      else
      {
        /* No suitable sampling window has been found, sampling is executed in the middle of the PWM
         * Period. Sampled currents will be disregarded and estimated currents will be used instead */
        SamplingPoint = pHandle->Half_PWMPeriod - 1u;
        pHandle->_Super.useEstCurrent = true;

      }
    }
  }
  return R3_1_WriteTIMRegisters( &pHandle->_Super, SamplingPoint );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Writes into peripheral registers the new duty cycles and sampling point.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  SamplingPoint: New capture/compare register value, written in timer clock counts.
  * @retval uint16_t Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__STATIC_INLINE uint16_t R3_1_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t SamplingPoint )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  uint16_t Aux;


  LL_TIM_OC_SetCompareCH1 ( TIMx, (uint32_t) pHandle->_Super.CntPhA );
  LL_TIM_OC_SetCompareCH2 ( TIMx, (uint32_t) pHandle->_Super.CntPhB );
  LL_TIM_OC_SetCompareCH3 ( TIMx, (uint32_t) pHandle->_Super.CntPhC );
  LL_TIM_OC_SetCompareCH4( TIMx, (uint32_t) SamplingPoint );

  /* Limit for update event */

//  if ( LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4) == 1u )
  if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET )
  {
    Aux = MC_DURATION;
  }
  else
  {
    Aux = MC_NO_ERROR;
  }
  return Aux;
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during polarization.
  * 
  * It sums up injected conversion data into PhaseAOffset and
  * PhaseBOffset to compute the offset introduced in the current feedback
  * network. It is required to properly configure ADC inputs before in order to enable
  * the offset computation.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
static void R3_1_HFCurrentsPolarizationAB( PWMC_Handle_t * pHdl, ab_t * Iab )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  uint32_t ADCDataReg1 = ADCx->JDR1;
  uint32_t ADCDataReg2 = ADCx->JDR2;
   
  /* disable ADC trigger source */
  //LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle-> PhaseAOffset += ADCDataReg1;
    pHandle-> PhaseBOffset += ADCDataReg2;
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  Iab->a = 0;
  Iab->b = 0;
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during polarization.
  * 
  * It sums up injected conversion data into PhaseCOffset to compute the offset
  * introduced in the current feedback network. It is required to properly configure 
  * ADC inputs before in order to enable offset computation.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
static void R3_1_HFCurrentsPolarizationC( PWMC_Handle_t * pHdl, ab_t * Iab )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  uint32_t ADCDataReg2 = ADCx->JDR2;

  /* disable ADC trigger source */
  //LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    /* Phase C is read from SECTOR_1, second value */
    pHandle-> PhaseCOffset += ADCDataReg2;    
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  Iab->a = 0;
  Iab->b = 0;
}

/**
  * @brief  Turns on low side switches.
  * 
  * This function is intended to be used for charging boot capacitors of driving section. It has to be
  * called on each motor start-up when using high voltage drivers.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  ticks: Timer ticks value to be applied
  *                Min value: 0 (low sides ON)
  *                Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)
  */
__weak void R3_1_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1( TIMx, ticks );
  LL_TIM_OC_SetCompareCH2( TIMx, ticks );
  LL_TIM_OC_SetCompareCH3( TIMx, ticks );
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }
  return;
}


/**
  * @brief  Enables PWM generation on the proper Timer peripheral.
  * 
  * This function is specific for RL detection phase.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_1_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, ((uint32_t) pHandle->Half_PWMPeriod / (uint32_t) 2));
  LL_TIM_OC_SetCompareCH2(TIMx, ((uint32_t) pHandle->Half_PWMPeriod / (uint32_t) 2));
  LL_TIM_OC_SetCompareCH3(TIMx, ((uint32_t) pHandle->Half_PWMPeriod / (uint32_t) 2));
  LL_TIM_OC_SetCompareCH4(TIMx, ((uint32_t) pHandle->Half_PWMPeriod - (uint32_t) 5));
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs ( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
  pHandle->_Super.PWMState = true;
}


/**
  * @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_1_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  pHandle->_Super.PWMState = false;
  pHandle->_Super.TurnOnLowSidesAction = false;
  
  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs( TIMx );
  if ( pHandle->_Super.BrakeActionLock == true )
  {
  }
  else
  {
    if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
}


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Contains the TIMx Update event interrupt.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void * R3_1_TIMx_UP_IRQHandler( PWMC_R3_1_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;
  
  ADCx->JSQR = pHandle->pParams_str->ADCConfig[pHandle->_Super.Sector] | ((uint32_t) pHandle->ADCTriggerEdge);
  
  /* enable ADC trigger source */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
    
  pHandle->ADCTriggerEdge = (uint16_t)LL_ADC_INJ_TRIG_EXT_RISING;

  return &( pHandle->_Super.Motor );
}

/**
  * @brief  Configures the analog output used for protection thresholds.
  * 
  * Specific to F30X and G4XX.
  *
  * @param  DAC_Channel: the selected DAC channel.
  *          This parameter can be:
  *            @arg DAC_Channel_1: DAC Channel1 selected.
  *            @arg DAC_Channel_2: DAC Channel2 selected.
  * @param  hDACVref: Value of DAC reference expressed as 16bit unsigned integer.
  *         Ex. 0 = 0V 65536 = VDD_DAC.
  */
static void R3_1_SetAOReferenceVoltage( uint32_t DAC_Channel, uint16_t hDACVref )
{
  LL_DAC_ConvertData12LeftAligned ( DAC1, DAC_Channel, hDACVref );

  /* Enable DAC Channel */
  LL_DAC_TrigSWConversion ( DAC1, DAC_Channel );
  
  if (LL_DAC_IsEnabled ( DAC1, DAC_Channel ) == 1u ) 
  { /* If DAC is already enable, we wait LL_DAC_DELAY_VOLTAGE_SETTLING_US*/
    uint32_t wait_loop_index = ((LL_DAC_DELAY_VOLTAGE_SETTLING_US) * (SystemCoreClock / (1000000UL * 2UL)));      
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }
  else
  {
    /* If DAC is not enabled, we must wait LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US*/
    LL_DAC_Enable ( DAC1, DAC_Channel );
    uint32_t wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US) * (SystemCoreClock / (1000000UL * 2UL)));      
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }    
  }
}

/**
  * @brief  Sets the PWM mode for R/L detection.
  * 
  * Specific to F30X, F4XX, L4XX and G4XX.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void R3_1_RLDetectionModeEnable( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  
  if ( pHandle->_Super.RLDetectionMode == false )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    LL_TIM_OC_SetCompareCH1( TIMx, 0u );

    /*  Channel2 configuration */
    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
   
  }

  __disable_irq();
  pHandle->_Super.pFctGetPhaseCurrents = &R3_1_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_1_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_1_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_1_SwitchOffPWM;
  __enable_irq();

  pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  Disables the PWM mode for R/L detection.
  * 
  * Specific to F30X, F4XX, L4XX and G4XX.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void R3_1_RLDetectionModeDisable( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == true )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH1( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel2 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH2( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH3( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );
    
    /* ADCx Injected discontinuous mode disable */
    LL_ADC_INJ_SetSequencerDiscont( pHandle->pParams_str->ADCx,
                                    LL_ADC_INJ_SEQ_DISCONT_DISABLE );
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = &R3_1_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_1_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_1_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_1_SwitchOffPWM;
    __enable_irq();

    pHandle->_Super.RLDetectionMode = false;
  }
}

/**
  * @brief  Sets the PWM dutycycle for R/L detection.
  *
  * Specific to F30X, F4XX, L4XX and G4XX.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  hDuty: Duty cycle to apply, written in uint16_t.
  * @retval uint16_t Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
uint16_t R3_1_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  uint32_t val;
  uint16_t hAux;


  val = ( ( uint32_t )( pHandle->Half_PWMPeriod ) * ( uint32_t )( hDuty ) ) >> 16;
  pHandle->_Super.CntPhA = ( uint16_t )( val );
  
  /* set sector in order to sample phase B */
  pHandle->_Super.Sector = SECTOR_4;
  
  /* TIM1 Channel 1 Duty Cycle configuration.
   * In RL Detection mode only the Up-side device of Phase A are controlled*/
  LL_TIM_OC_SetCompareCH1(TIMx, ( uint32_t )pHandle->_Super.CntPhA);


  /* Limit for update event */
  /*  If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET )
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Computes and stores into @p pHandle latest converted motor phase currents
  *         during RL detection phase.
  * 
  * Specific to F30X, F4XX, L4XX and G4XX.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
static void R3_1_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;
  int32_t wAux;
  
  /* disable ADC trigger source */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  wAux = (int32_t)( pHandle->PhaseBOffset ) - ADCx->JDR2;

  /* Check saturation */
  if ( wAux > -INT16_MAX )
  {
    if ( wAux < INT16_MAX )
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  pStator_Currents->a = (int16_t)wAux;
  pStator_Currents->b = (int16_t)wAux;
}

/**
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors
  * of driving section. It has to be called at each motor start-up when
  * using high voltage drivers.
  * This function is specific for RL detection phase. Specific to F30X, F4XX, L4XX and G4XX.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  ticks: Duty cycle of the boot capacitors charge, specific to motor.
  */
static void R3_1_RLTurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks )
{
  (void)ticks; /* parameter not used */
  
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1 ( TIMx, 0u );
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  
  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }
  return;
}


/**
  * @brief  Enables PWM generation on the proper Timer peripheral.
  * 
  * This function is specific for RL detection phase. Specific to F30X, F4XX, L4XX and G4XX.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
static void R3_1_RLSwitchOnPWM( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Set channel 1 Compare/Capture register to 1 */
  LL_TIM_OC_SetCompareCH1(TIMx, 1u);
  /* Set channel 4 Compare/Capture register to trig ADC in the middle 
     of the PWM period */
  LL_TIM_OC_SetCompareCH4(TIMx,(( uint32_t )( pHandle->Half_PWMPeriod ) - 5u));
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs(TIMx);

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }

  /* set the sector that correspond to Phase A and B sampling
   * B will be sampled by ADCx_1 */
  pHdl->Sector = SECTOR_4;
  pHandle->_Super.PWMState = true;
}

/**
 * @brief  Turns on low sides switches and start ADC triggering.
 * 
 * This function is specific for MP phase. Specific to F30X, F4XX, L4XX and G4XX.
 *
 * @param  pHdl: Handler of the current instance of the PWM component.
 */
void R3_1_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  LL_TIM_OC_SetCompareCH1( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH2( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH3( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod - 5u));
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs ( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }

  pHdl->Sector = SECTOR_5;
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  return;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
