#include "ethercat/cia402/cia402.h"

#include "esc.h"

/* CiA402 control word bits */
#define CW_SWITCH_ON        0x0001U
#define CW_ENABLE_VOLTAGE   0x0002U
#define CW_QUICK_STOP       0x0004U
#define CW_ENABLE_OPERATION 0x0008U
#define CW_FAULT_RESET      0x0080U

/* CiA402 status word masks/states */
#define SW_MASK_STATE                0x006FU
#define SW_SWITCH_ON_DISABLED        0x0040U
#define SW_READY_TO_SWITCH_ON        0x0021U
#define SW_SWITCHED_ON               0x0023U
#define SW_OPERATION_ENABLED         0x0027U
#define SW_QUICK_STOP_ACTIVE         0x0007U
#define SW_FAULT                     0x0008U

/* Additional status bits commonly expected by masters. */
#define SW_REMOTE                    0x0200U

/* CiA402 command decoding masks. */
#define CW_CMD_MASK                  0x008FU
#define CW_CMD_DISABLE_VOLTAGE       0x0000U
#define CW_CMD_SHUTDOWN              0x0006U
#define CW_CMD_SWITCH_ON             0x0007U
#define CW_CMD_ENABLE_OPERATION      0x000FU
#define CW_CMD_QUICK_STOP            0x0002U

typedef enum
{
    CIA402_STATE_SWITCH_ON_DISABLED = 0,
    CIA402_STATE_READY_TO_SWITCH_ON,
    CIA402_STATE_SWITCHED_ON,
    CIA402_STATE_OPERATION_ENABLED,
    CIA402_STATE_QUICK_STOP_ACTIVE,
    CIA402_STATE_FAULT
} cia402_state_t;

static cia402_state_t cia402_state_from_status(uint16_t status_word)
{
    uint16_t state_code = status_word & SW_MASK_STATE;

    switch (state_code)
    {
        case SW_READY_TO_SWITCH_ON:
            return CIA402_STATE_READY_TO_SWITCH_ON;
        case SW_SWITCHED_ON:
            return CIA402_STATE_SWITCHED_ON;
        case SW_OPERATION_ENABLED:
            return CIA402_STATE_OPERATION_ENABLED;
        case SW_QUICK_STOP_ACTIVE:
            return CIA402_STATE_QUICK_STOP_ACTIVE;
        case SW_FAULT:
            return CIA402_STATE_FAULT;
        case SW_SWITCH_ON_DISABLED:
        default:
            return CIA402_STATE_SWITCH_ON_DISABLED;
    }
}

static uint16_t cia402_status_from_state(cia402_state_t state)
{
    switch (state)
    {
        case CIA402_STATE_READY_TO_SWITCH_ON:
            return SW_READY_TO_SWITCH_ON | SW_REMOTE;
        case CIA402_STATE_SWITCHED_ON:
            return SW_SWITCHED_ON | SW_REMOTE;
        case CIA402_STATE_OPERATION_ENABLED:
            return SW_OPERATION_ENABLED | SW_REMOTE;
        case CIA402_STATE_QUICK_STOP_ACTIVE:
            return SW_QUICK_STOP_ACTIVE | SW_REMOTE;
        case CIA402_STATE_FAULT:
            return SW_FAULT | SW_REMOTE;
        case CIA402_STATE_SWITCH_ON_DISABLED:
        default:
            return SW_SWITCH_ON_DISABLED | SW_REMOTE;
    }
}

void CIA402_Init(uint16_t *status_word, int8_t *mode_display)
{
    if (status_word != 0)
    {
        *status_word = cia402_status_from_state(CIA402_STATE_SWITCH_ON_DISABLED);
    }

    if (mode_display != 0)
    {
        *mode_display = 0;
    }
}

void CIA402_Update(uint8_t al_state, uint16_t control_word, int8_t mode_request,
                   uint16_t *status_word, int8_t *mode_display)
{
    cia402_state_t state;
    uint16_t command;
    uint8_t al_state_masked;

    if (status_word == 0)
    {
        return;
    }

    state = cia402_state_from_status(*status_word);
    command = control_word & CW_CMD_MASK;
    al_state_masked = (uint8_t)(al_state & ESCREG_AL_STATEMASK);

    /*
     * Bind CiA402 to EtherCAT AL state:
     * - Below SAFE-OP: only Switch On Disabled is valid.
     * - SAFE-OP: run CiA402 transitions but do not allow Operation Enabled.
     * - OP: full CiA402 state machine allowed.
     */
    if ((al_state_masked != ESCsafeop) && (al_state_masked != ESCop))
    {
        *status_word = cia402_status_from_state(CIA402_STATE_SWITCH_ON_DISABLED);
        if (mode_display != 0)
        {
            *mode_display = mode_request;
        }
        return;
    }

    if ((control_word & CW_FAULT_RESET) != 0U)
    {
        if (state == CIA402_STATE_FAULT)
        {
            state = CIA402_STATE_SWITCH_ON_DISABLED;
        }
    }

    switch (state)
    {
        case CIA402_STATE_SWITCH_ON_DISABLED:
            if (command == CW_CMD_SHUTDOWN)
            {
                state = CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;

        case CIA402_STATE_READY_TO_SWITCH_ON:
            if (command == CW_CMD_DISABLE_VOLTAGE)
            {
                state = CIA402_STATE_SWITCH_ON_DISABLED;
            }
            else if (command == CW_CMD_SWITCH_ON)
            {
                state = CIA402_STATE_SWITCHED_ON;
            }
            break;

        case CIA402_STATE_SWITCHED_ON:
            if (command == CW_CMD_DISABLE_VOLTAGE)
            {
                state = CIA402_STATE_SWITCH_ON_DISABLED;
            }
            else if (command == CW_CMD_SHUTDOWN)
            {
                state = CIA402_STATE_READY_TO_SWITCH_ON;
            }
            else if (command == CW_CMD_ENABLE_OPERATION)
            {
                state = CIA402_STATE_OPERATION_ENABLED;
            }
            break;

        case CIA402_STATE_OPERATION_ENABLED:
            if (command == CW_CMD_DISABLE_VOLTAGE)
            {
                state = CIA402_STATE_SWITCH_ON_DISABLED;
            }
            else if (command == CW_CMD_SHUTDOWN)
            {
                state = CIA402_STATE_READY_TO_SWITCH_ON;
            }
            else if (command == CW_CMD_SWITCH_ON)
            {
                state = CIA402_STATE_SWITCHED_ON;
            }
            else if (command == CW_CMD_QUICK_STOP || (control_word & CW_QUICK_STOP) == 0U)
            {
                state = CIA402_STATE_QUICK_STOP_ACTIVE;
            }
            break;

        case CIA402_STATE_QUICK_STOP_ACTIVE:
            if (command == CW_CMD_DISABLE_VOLTAGE)
            {
                state = CIA402_STATE_SWITCH_ON_DISABLED;
            }
            else if ((control_word & CW_QUICK_STOP) != 0U)
            {
                if (command == CW_CMD_ENABLE_OPERATION)
                {
                    state = CIA402_STATE_OPERATION_ENABLED;
                }
                else if (command == CW_CMD_SWITCH_ON)
                {
                    state = CIA402_STATE_SWITCHED_ON;
                }
                else if (command == CW_CMD_SHUTDOWN)
                {
                    state = CIA402_STATE_READY_TO_SWITCH_ON;
                }
            }
            break;

        case CIA402_STATE_FAULT:
        default:
            break;
    }

    if ((control_word & CW_ENABLE_VOLTAGE) == 0U && state != CIA402_STATE_FAULT)
    {
        state = CIA402_STATE_SWITCH_ON_DISABLED;
    }

    if ((al_state_masked == ESCsafeop) && (state == CIA402_STATE_OPERATION_ENABLED))
    {
        state = CIA402_STATE_SWITCHED_ON;
    }

    *status_word = cia402_status_from_state(state);

    if (mode_display != 0)
    {
        *mode_display = mode_request;
    }
}
