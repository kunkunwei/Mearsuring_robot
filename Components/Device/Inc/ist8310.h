//
// Created by kun on 25-7-17.
//

#ifndef IST8310_H
#define IST8310_H
#include "main.h"
#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

/*********************/
#define mag_max_x 76.5000f
#define mag_max_y 76.5000f
#define mag_max_z 76.5000f

#define mag_min_x -76.8000f
#define mag_min_y -76.8000f
#define mag_min_z -76.8000f

/*********************/
typedef struct ist8310_real_data_t
{
    uint8_t status;
    float raw_mag[3];
    float calibrated_mag[3];

    float mag_max[3];
    float mag_min[3];

    float mag_bias[3];
    float mag_scale[3];
} ist8310_real_data_t;

extern ist8310_real_data_t ist8310_Info;
extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data);
extern void ist8310_read_mag(fp32 mag[3]);
extern void mag_calibration(ist8310_real_data_t *ist8310_Info);
extern void simple_mag_calibration_messure(ist8310_real_data_t *ist8310_Info);
extern void IST8310_Info_Update(ist8310_real_data_t *ist8310_Info);
#endif //IST8310_H
