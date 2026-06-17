
/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
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
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"
#include "mc_api.h"

typedef enum
{
  MC_APP_BRINGUP_DISABLED = 0,
  MC_APP_BRINGUP_START_REQUESTED,
  MC_APP_BRINGUP_RUNNING,
  MC_APP_BRINGUP_FAULTED
} MC_APP_BringUpState_t;

#define MC_APP_STANDALONE_BRINGUP_ENABLE         0
#define MC_APP_STANDALONE_TARGET_SPEED_RPM       300
#define MC_APP_STANDALONE_SPEED_RAMP_DURATION_MS 1000U

static MC_APP_BringUpState_t mc_app_bringup_state = MC_APP_BRINGUP_DISABLED;

static bool MC_APP_IsFaultState(MCI_State_t motor_state)
{
  return ((motor_state == FAULT_NOW) || (motor_state == FAULT_OVER));
}

static void MC_APP_RunStandaloneBringUp(void)
{
  MCI_State_t motor_state;
  uint16_t current_faults;
  uint16_t occurred_faults;

  motor_state = MC_GetSTMStateMotor1();
  current_faults = MC_GetCurrentFaultsMotor1();
  occurred_faults = MC_GetOccurredFaultsMotor1();

  if ((current_faults != MC_NO_FAULTS) || (occurred_faults != MC_NO_FAULTS) || MC_APP_IsFaultState(motor_state))
  {
    mc_app_bringup_state = MC_APP_BRINGUP_FAULTED;
    return;
  }

  switch (mc_app_bringup_state)
  {
    case MC_APP_BRINGUP_DISABLED:
      if ((motor_state == IDLE) && MC_StartMotor1())
      {
        MC_ProgramSpeedRampMotor1((int16_t)(MC_APP_STANDALONE_TARGET_SPEED_RPM * SPEED_UNIT / U_RPM),
                                  MC_APP_STANDALONE_SPEED_RAMP_DURATION_MS);
        mc_app_bringup_state = MC_APP_BRINGUP_START_REQUESTED;
      }
      break;

    case MC_APP_BRINGUP_START_REQUESTED:
      if (motor_state == RUN)
      {
        mc_app_bringup_state = MC_APP_BRINGUP_RUNNING;
      }
      break;

    case MC_APP_BRINGUP_RUNNING:
    case MC_APP_BRINGUP_FAULTED:
    default:
      break;
  }
}

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup COMMON_MC
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */

/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
__weak void MC_APP_BootHook(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

/* USER CODE BEGIN BootHook */

#if (MC_APP_STANDALONE_BRINGUP_ENABLE != 0)
  mc_app_bringup_state = MC_APP_BRINGUP_DISABLED;
#else
  mc_app_bringup_state = MC_APP_BRINGUP_FAULTED;
#endif

/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

/* USER SECTION BEGIN PostMediumFrequencyHookM1 */

#if (MC_APP_STANDALONE_BRINGUP_ENABLE != 0)
  MC_APP_RunStandaloneBringUp();
#endif

/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
