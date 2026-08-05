#ifndef VESC_CAN_PROTOCOL_H
#define VESC_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

int32_t VescCan_MechanicalRpmToErpm(float mechanical_rpm, float pole_pairs);
bool VescCan_BuildSetErpmFrame(uint8_t vesc_id,
                               int32_t erpm,
                               uint32_t *ext_id,
                               uint8_t data[4]);

#endif
