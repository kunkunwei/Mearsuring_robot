#include "buzzer_music.h"
#include <assert.h>
#include <stdint.h>

static uint32_t last_frequency;
static uint32_t frequency_set_count;
static uint32_t off_count;

void Buzzer_SetFrequency(uint32_t frequency_hz)
{
    last_frequency = frequency_hz;
    frequency_set_count++;
}

void buzzer_off(void)
{
    last_frequency = 0U;
    off_count++;
}

static void test_boot_sequence(void)
{
    Buzzer_AlertInit(0U);
    assert(last_frequency == 1046U);

    Buzzer_AlertUpdate(249U, 0U);
    assert(last_frequency == 1046U);
    Buzzer_AlertUpdate(250U, 0U);
    assert(last_frequency == 1174U);
    Buzzer_AlertUpdate(500U, 0U);
    assert(last_frequency == 1568U);
    Buzzer_AlertUpdate(750U, 0U);
    assert(last_frequency == 0U);
}

static void test_motor_two_is_reported_with_two_beeps(void)
{
    const uint32_t set_count_before = frequency_set_count;

    Buzzer_AlertUpdate(2749U, 1U << 1U);
    assert(last_frequency == 0U);
    Buzzer_AlertUpdate(2750U, 1U << 1U);
    assert(last_frequency == 1000U);
    assert(frequency_set_count == set_count_before + 1U);

    Buzzer_AlertUpdate(2870U, 1U << 1U);
    assert(last_frequency == 0U);
    Buzzer_AlertUpdate(2990U, 1U << 1U);
    assert(last_frequency == 1000U);
    assert(frequency_set_count == set_count_before + 2U);
    Buzzer_AlertUpdate(3110U, 1U << 1U);
    assert(last_frequency == 0U);
}

static void test_recovery_stops_alert_immediately(void)
{
    const uint32_t off_count_before = off_count;
    Buzzer_AlertUpdate(3200U, 0U);
    assert(last_frequency == 0U);
    assert(off_count > off_count_before);
}

int main(void)
{
    test_boot_sequence();
    test_motor_two_is_reported_with_two_beeps();
    test_recovery_stops_alert_immediately();
    return 0;
}
