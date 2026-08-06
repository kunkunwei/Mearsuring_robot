#ifndef CHASSIS_CONTROL_MANAGER_H
#define CHASSIS_CONTROL_MANAGER_H

#include "chassis_brake.h"
#include "chassis_hold_ctrl.h"
#include "chassis_mit_ctrl.h"

#include <stdint.h>

#define CHASSIS_CONTROL_MOTOR_COUNT 4U

typedef enum
{
    CHASSIS_CTRL_DISABLED = 0,
    CHASSIS_CTRL_DRIVE,
    CHASSIS_CTRL_BRAKE,
    CHASSIS_CTRL_HOLD,
    CHASSIS_CTRL_FAULT,
} Chassis_Control_State_e;

typedef enum
{
    CHASSIS_CTRL_FAULT_NONE = 0,
    CHASSIS_CTRL_FAULT_FEEDBACK,
    CHASSIS_CTRL_FAULT_OVERSPEED,
    CHASSIS_CTRL_FAULT_OSCILLATION,
    CHASSIS_CTRL_FAULT_SATURATION,
    CHASSIS_CTRL_FAULT_TIMING,
} Chassis_Control_Fault_e;

typedef struct
{
    Chassis_Mit_Config_t drive;
    Chassis_Brake_Config_t brake;
    Chassis_Hold_Config_t hold;
    float brake_position_comp_off_pitch_rad;
    float brake_position_comp_full_pitch_rad;
    float command_deadband_rpm;
    float turn_breakaway_current_a;
    float turn_breakaway_target_rpm;
    float turn_breakaway_enter_rpm;
    float turn_breakaway_release_ratio;
    float turn_breakaway_hold_time_s;
    float turn_breakaway_taper_time_s;
    uint8_t turn_sync_enabled;
    float turn_sync_min_ratio;
    float turn_sync_time_s;
    float drive_slip_error_ratio;
    float drive_slip_recover_ratio;
    float drive_slip_confirm_time_s;
    float hold_enter_speed_rpm;
    float hold_enter_time_s;
    float current_rise_a_per_s;
    float current_release_a_per_s;
    float hold_overspeed_rpm;
    float hold_overspeed_time_s;
    float oscillation_window_s;
    float oscillation_min_current_a;
    uint8_t oscillation_reversal_limit;
    float saturation_time_s;
    float dt_min_s;
    float dt_max_s;
    uint8_t timing_fault_count_limit;
} Chassis_Control_Config_t;

typedef struct
{
    uint8_t enabled;
    float target_rpm[CHASSIS_CONTROL_MOTOR_COUNT];
    float speed_rpm[CHASSIS_CONTROL_MOTOR_COUNT];
    float position_deg[CHASSIS_CONTROL_MOTOR_COUNT];
    uint8_t speed_valid[CHASSIS_CONTROL_MOTOR_COUNT];
    uint8_t position_valid[CHASSIS_CONTROL_MOTOR_COUNT];
    float pitch_rad;
} Chassis_Control_Input_t;

typedef struct
{
    Chassis_Control_State_e state;
    Chassis_Control_Fault_e fault;
    float current_a[CHASSIS_CONTROL_MOTOR_COUNT];
    float raw_current_a[CHASSIS_CONTROL_MOTOR_COUNT];
    float position_current_a[CHASSIS_CONTROL_MOTOR_COUNT];
    float speed_current_a[CHASSIS_CONTROL_MOTOR_COUNT];
    float feedforward_current_a[CHASSIS_CONTROL_MOTOR_COUNT];
} Chassis_Control_Output_t;

typedef struct
{
    Chassis_Control_Config_t config;
    Chassis_Control_State_e state;
    Chassis_Control_Fault_e fault;
    Chassis_Mit_State_t drive_state[CHASSIS_CONTROL_MOTOR_COUNT];
    Chassis_Hold_State_t hold_state;
    float hold_wheel_position_ref_deg[CHASSIS_CONTROL_MOTOR_COUNT];
    float brake_position_comp_scale;
    float last_current_a[CHASSIS_CONTROL_MOTOR_COUNT];
    float turn_sync_ratio;
    float breakaway_engaged_time_s[CHASSIS_CONTROL_MOTOR_COUNT];
    float drive_slip_elapsed_s[CHASSIS_CONTROL_MOTOR_COUNT];
    uint8_t breakaway_engaged[CHASSIS_CONTROL_MOTOR_COUNT];
    uint8_t drive_slip_limited[CHASSIS_CONTROL_MOTOR_COUNT];
    float hold_still_time_s;
    float hold_overspeed_elapsed_s;
    float reversal_window_elapsed_s;
    float saturation_elapsed_s;
    int8_t last_current_sign[CHASSIS_CONTROL_MOTOR_COUNT];
    uint8_t reversal_count[CHASSIS_CONTROL_MOTOR_COUNT];
    uint8_t timing_fault_count;
} Chassis_Control_Manager_t;

void Chassis_ControlManager_DefaultConfig(Chassis_Control_Config_t *config);
void Chassis_ControlManager_Init(Chassis_Control_Manager_t *manager,
                                 const Chassis_Control_Config_t *config);
void Chassis_ControlManager_Disable(Chassis_Control_Manager_t *manager,
                                    Chassis_Control_Output_t *output);
void Chassis_ControlManager_Update(Chassis_Control_Manager_t *manager,
                                   const Chassis_Control_Input_t *input,
                                   float dt_s,
                                   Chassis_Control_Output_t *output);
void Chassis_ControlManager_SetDriveGains(Chassis_Control_Manager_t *manager,
                                          float position_kp_a_per_deg,
                                          float speed_kd_a_per_rpm,
                                          float friction_current_a);

#endif
