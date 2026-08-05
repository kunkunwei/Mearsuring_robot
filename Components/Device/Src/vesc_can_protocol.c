#include "vesc_can_protocol.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define VESC_CAN_PACKET_SET_RPM 3U

int32_t VescCan_MechanicalRpmToErpm(float mechanical_rpm, float pole_pairs)
{
    if (!isfinite(mechanical_rpm) || !isfinite(pole_pairs) || pole_pairs <= 0.0f)
    {
        return 0;
    }

    const float erpm = mechanical_rpm * pole_pairs;
    if (erpm >= (float)INT32_MAX)
    {
        return INT32_MAX;
    }
    if (erpm <= (float)INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)erpm;
}

bool VescCan_BuildSetErpmFrame(uint8_t vesc_id,
                               int32_t erpm,
                               uint32_t *ext_id,
                               uint8_t data[4])
{
    if (ext_id == NULL || data == NULL)
    {
        return false;
    }

    *ext_id = ((uint32_t)VESC_CAN_PACKET_SET_RPM << 8U) | vesc_id;
    data[0] = (uint8_t)((uint32_t)erpm >> 24U);
    data[1] = (uint8_t)((uint32_t)erpm >> 16U);
    data[2] = (uint8_t)((uint32_t)erpm >> 8U);
    data[3] = (uint8_t)erpm;
    return true;
}
