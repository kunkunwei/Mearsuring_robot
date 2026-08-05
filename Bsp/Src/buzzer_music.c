#include "buzzer_music.h"
#include "bsp_tim.h"
#include <stddef.h>

#define BUZZER_BOOT_NOTE_DURATION_MS 250U
#define BUZZER_ALARM_STARTUP_GRACE_MS 2000U
#define BUZZER_ALARM_FREQUENCY_HZ 1000U
#define BUZZER_ALARM_TONE_MS 120U
#define BUZZER_ALARM_TONE_GAP_MS 120U
#define BUZZER_ALARM_MOTOR_GAP_MS 500U
#define BUZZER_ALARM_REPEAT_GAP_MS 2000U

typedef enum
{
    BUZZER_PHASE_BOOT = 0,
    BUZZER_PHASE_IDLE,
    BUZZER_PHASE_ALARM_TONE,
    BUZZER_PHASE_ALARM_TONE_GAP,
    BUZZER_PHASE_ALARM_MOTOR_GAP,
    BUZZER_PHASE_ALARM_REPEAT_GAP,
} BuzzerPhase_t;

typedef struct
{
    BuzzerPhase_t phase;
    uint32_t deadline_ms;
    uint32_t alarm_enable_tick_ms;
    uint8_t boot_note_index;
    uint8_t active_offline_mask;
    uint8_t motor_index;
    uint8_t beeps_remaining;
} BuzzerAlertState_t;

static const uint16_t boot_notes_hz[] = {1046U, 1174U, 1568U};
static BuzzerAlertState_t buzzer_state;

static uint8_t Buzzer_DeadlineReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static uint8_t Buzzer_FindOfflineMotor(uint8_t mask, uint8_t start_index, uint8_t *motor_index)
{
    if (motor_index == NULL)
    {
        return 0U;
    }

    for (uint8_t i = start_index; i < 4U; i++)
    {
        if ((mask & (uint8_t)(1U << i)) != 0U)
        {
            *motor_index = i;
            return 1U;
        }
    }
    return 0U;
}

static void Buzzer_StartAlarmTone(uint32_t now_ms)
{
    Buzzer_SetFrequency(BUZZER_ALARM_FREQUENCY_HZ);
    buzzer_state.phase = BUZZER_PHASE_ALARM_TONE;
    buzzer_state.deadline_ms = now_ms + BUZZER_ALARM_TONE_MS;
}

static void Buzzer_StartMotorReport(uint32_t now_ms, uint8_t motor_index)
{
    buzzer_state.motor_index = motor_index;
    buzzer_state.beeps_remaining = motor_index + 1U;
    Buzzer_StartAlarmTone(now_ms);
}

void Buzzer_AlertInit(uint32_t now_ms)
{
    buzzer_state.phase = BUZZER_PHASE_BOOT;
    buzzer_state.deadline_ms = now_ms + BUZZER_BOOT_NOTE_DURATION_MS;
    buzzer_state.alarm_enable_tick_ms = 0U;
    buzzer_state.boot_note_index = 0U;
    buzzer_state.active_offline_mask = 0U;
    buzzer_state.motor_index = 0U;
    buzzer_state.beeps_remaining = 0U;
    Buzzer_SetFrequency(boot_notes_hz[0]);
}

void Buzzer_AlertUpdate(uint32_t now_ms, uint8_t motor_offline_mask)
{
    if (buzzer_state.phase == BUZZER_PHASE_BOOT)
    {
        if (Buzzer_DeadlineReached(now_ms, buzzer_state.deadline_ms) == 0U)
        {
            return;
        }

        buzzer_state.boot_note_index++;
        if (buzzer_state.boot_note_index < (uint8_t)(sizeof(boot_notes_hz) / sizeof(boot_notes_hz[0])))
        {
            Buzzer_SetFrequency(boot_notes_hz[buzzer_state.boot_note_index]);
            buzzer_state.deadline_ms = now_ms + BUZZER_BOOT_NOTE_DURATION_MS;
        }
        else
        {
            buzzer_off();
            buzzer_state.phase = BUZZER_PHASE_IDLE;
            buzzer_state.alarm_enable_tick_ms = now_ms + BUZZER_ALARM_STARTUP_GRACE_MS;
        }
        return;
    }

    if (Buzzer_DeadlineReached(now_ms, buzzer_state.alarm_enable_tick_ms) == 0U)
    {
        return;
    }

    motor_offline_mask &= 0x0FU;
    if (motor_offline_mask != buzzer_state.active_offline_mask)
    {
        buzzer_state.active_offline_mask = motor_offline_mask;
        buzzer_state.phase = BUZZER_PHASE_IDLE;
        buzzer_off();
    }

    if (motor_offline_mask == 0U)
    {
        return;
    }

    if (buzzer_state.phase == BUZZER_PHASE_IDLE)
    {
        uint8_t motor_index = 0U;
        if (Buzzer_FindOfflineMotor(motor_offline_mask, 0U, &motor_index) != 0U)
        {
            Buzzer_StartMotorReport(now_ms, motor_index);
        }
        return;
    }

    if (Buzzer_DeadlineReached(now_ms, buzzer_state.deadline_ms) == 0U)
    {
        return;
    }

    if (buzzer_state.phase == BUZZER_PHASE_ALARM_TONE)
    {
        buzzer_off();
        buzzer_state.beeps_remaining--;
        if (buzzer_state.beeps_remaining > 0U)
        {
            buzzer_state.phase = BUZZER_PHASE_ALARM_TONE_GAP;
            buzzer_state.deadline_ms = now_ms + BUZZER_ALARM_TONE_GAP_MS;
            return;
        }

        uint8_t next_motor_index = 0U;
        if (Buzzer_FindOfflineMotor(motor_offline_mask,
                                    buzzer_state.motor_index + 1U,
                                    &next_motor_index) != 0U)
        {
            buzzer_state.motor_index = next_motor_index;
            buzzer_state.beeps_remaining = next_motor_index + 1U;
            buzzer_state.phase = BUZZER_PHASE_ALARM_MOTOR_GAP;
            buzzer_state.deadline_ms = now_ms + BUZZER_ALARM_MOTOR_GAP_MS;
        }
        else
        {
            buzzer_state.phase = BUZZER_PHASE_ALARM_REPEAT_GAP;
            buzzer_state.deadline_ms = now_ms + BUZZER_ALARM_REPEAT_GAP_MS;
        }
        return;
    }

    if (buzzer_state.phase == BUZZER_PHASE_ALARM_TONE_GAP ||
        buzzer_state.phase == BUZZER_PHASE_ALARM_MOTOR_GAP)
    {
        Buzzer_StartAlarmTone(now_ms);
        return;
    }

    if (buzzer_state.phase == BUZZER_PHASE_ALARM_REPEAT_GAP)
    {
        uint8_t motor_index = 0U;
        if (Buzzer_FindOfflineMotor(motor_offline_mask, 0U, &motor_index) != 0U)
        {
            Buzzer_StartMotorReport(now_ms, motor_index);
        }
    }
}
