/**
	****************************(C) COPYRIGHT 2016 DJI****************************
	* @file       chassis_behaviour.c/h
	* @brief      完成底盘行为任务�?	* @note
	* @history
	*  Version    Date            Author          Modification
	*  V1.0.0     Dec-26-2018     RM              1. 完成
	*  v2.0.0     Nov-05-2023     pxx             刚起�?	*
	@verbatim
	==============================================================================

	==============================================================================
	@endverbatim
	****************************(C) COPYRIGHT 2016 DJI****************************
	*/
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "main.h"
#include "remote_control.h"
#include "old_pid.h"
// #include "CAN_Receive.h"


#include "user_lib.h"
// #include "observe_task.h"

// 前向声明
//  struct JumpController_t;
/*底盘CAN_ID
			 �?		1     2
	�?            �?		4     3
			 �?*/

// 任务开始空闲一段时�?#define CHASSIS_TASK_INIT_TIME 357
/////////////////////////////////////////////////////////////////
/* 遥控器开关定�?*/
#define RIGHT_SWITCH 0  // 右拨�?#define LEFT_SWITCH 1   // 左拨�?/* 遥控器通道定义 */
#define RC_RIGHT_X_CH 0 // 右摇杆X轴通道
#define RC_RIGHT_Y_CH 1 // 右摇杆Y轴通道
#define RC_LEFT_X_CH 2  // 左摇杆X轴通道
#define RC_LEFT_Y_CH 3  // 左摇杆Y轴通道
// 功能开关通道�?#define MODE_CHANNEL 0
#define MODE_CHANNEL 0
#define FUNCTION_CHANNEL 1
///////////////////////////////////////////////////////////////
// 遥控遥感死区限制
#define CHASSIS_RC_DEADLINE 10

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.003f
//手动模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比�?#define CHASSIS_ANGLE_Z_RC_SEN 0.020f

////////////////////////////////////////////////////////////////
#define CHASSIS_ANGLE_Z_RC_SEN 0.020f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f



// 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
// 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
// 底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

// // 底盘电机最大速度
// #define MAX_WHEEL_SPEED 1.0f
// // 底盘运动过程最大前进速度
// #define NORMAL_MAX_CHASSIS_SPEED_X 1.3f
// // 底盘设置旋转速度
// #define CHASSIS_WZ_SET_SCALE 0.0f

//////////////////////////////////////////////////////////
#define WHEEL_R 0.025f // Wheel radius, m.
#define CHASSIS_WHEEL_BASE 0.085f // Front-rear wheelbase measured at tire centers, m.
#define CHASSIS_WHEEL_TRACK 0.081f // Left-right track measured at tire centers, m.

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.5f
#define MOTOR_DISTANCE_TO_CENTER (CHASSIS_WHEEL_TRACK / 2.0f)  // 轮距的一半，单位 m
#define CHASSIS_MOTOR_1_FORWARD_SIGN (-1.0f)
#define CHASSIS_MOTOR_2_FORWARD_SIGN ( 1.0f)
#define CHASSIS_MOTOR_3_FORWARD_SIGN ( 1.0f)
#define CHASSIS_MOTOR_4_FORWARD_SIGN (-1.0f)
//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比�?#define M3508_MOTOR_RPM_TO_VECTOR ((2.0f * PI * WHEEL_R) / 60.0f)
#define M3508_MOTOR_RPM_TO_VECTOR ((2.0f * PI * WHEEL_R) / 60.0f)
// #define M3508_MOTOR_RPM_TO_VECTOR 0.0006237146233552417758135
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN (M3508_MOTOR_RPM_TO_VECTOR)
#define CHASSIS_MOTOR_VECTOR_TO_RPM_SEN (1.0f / CHASSIS_MOTOR_RPM_TO_VECTOR_SEN)

//底盘电机最大速度
#define MAX_WHEEL_SPEED 3.6f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
#define NORMAL_MAX_CHASSIS_SPEED_WZ 15.0f

//////////////////////////////////////////////////////////

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 25.5f
#define M3505_MOTOR_SPEED_PID_KI 15.5f
#define M3505_MOTOR_SPEED_PID_KD 700.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 10000.0f	//16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 500.0f

//底盘旋转跟随PID
#define YAW_SPEED_PID_KP 2.0f
#define YAW_SPEED_PID_KI 0.20f
#define YAW_SPEED_PID_KD 0.288f
#define YAW_SPEED_PID_MAX_OUT 10.0f
#define YAW_SPEED_PID_MAX_IOUT 2.0f

// MIT-style wheel current control. KP=position stiffness, KI=velocity damping, KD=current feedforward.
#define CHASSIS_MIT_ACTIVE_RPM_THRESHOLD 3.0f
#define CHASSIS_MIT_BREAKAWAY_ENTER_RPM 30.0f
#define CHASSIS_MIT_BREAKAWAY_EXIT_RPM 80.0f
#define CHASSIS_MIT_RUNNING_FF_RATIO 0.30f
#define CHASSIS_STOP_BRAKE_DONE_RPM 5.0f
#define CHASSIS_STOP_BRAKE_MAX_CURRENT 10000.0f
#define CHASSIS_TURN_MIN_RPM 120.0f
#define CHASSIS_TURN_MIN_VX_THRESHOLD 0.03f
#define CHASSIS_TURN_MIN_WZ_THRESHOLD 0.10f
#define CHASSIS_MOTOR_STATUS1_TIMEOUT_MS 20U
#define CHASSIS_MOTOR_STATUS4_TIMEOUT_MS 80U
#define CHASSIS_MIT_POS_ERROR_MAX_DEG 90.0f
#define CHASSIS_HOLD_POS_ERROR_MAX_DEG 120.0f
#define CHASSIS_HOLD_STATIC_DEADBAND_DEG 0.2f
#define CHASSIS_HOLD_KP_CURRENT_PER_DEG 80.0f
#define CHASSIS_HOLD_DAMP_CURRENT_PER_RPM 8.0f
#define CHASSIS_HOLD_STATIC_CURRENT 2500.0f
#define CHASSIS_HOLD_MAX_CURRENT 8000.0f

//////////////////////////////////////////////////////////

// 获取姿态角指针地址后，对应姿态角的地址偏移�?fp32类型
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 1
#define INS_GYRO_Y_ADDRESS_OFFSET 0
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 1
#define INS_ACCEL_Y_ADDRESS_OFFSET 0
#define INS_ACCEL_Z_ADDRESS_OFFSET 2
typedef enum
{
	CHASSIS_FORCE_RAW = 0,
	CHASSIS_MANL_CTRL = 1,
	CHASSIS_ROS_CTRL = 2,
	CHASSIS_HOLD_TEST = 3,
} chassis_mode_e;
typedef struct
{
	const dji_motor_measure_t *chassis_motor_measure;
	float accel;
	float speed;         // wheel feedback speed, m/s
	float speed_set;     // wheel target speed, m/s
	float speed_rpm;     // signed motor/wheel feedback speed, rpm
	float speed_set_rpm; // signed motor/wheel target speed, rpm
	float pos_deg;       // signed continuous wheel feedback position, degree
	float pos_set_deg;   // signed continuous wheel target position, degree
	float last_pos_raw_deg;
		float hold_pos_ref_deg;
	float hold_pos_error_deg;
uint32_t last_status4_tick;
	uint8_t ff_breakaway_active;
	int8_t ff_last_sign;
		uint8_t hold_pos_ready;
uint8_t pos_ready;
	int16_t target_current;
} Chassis_Motor_t;
typedef struct
{
	PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
	PidTypeDef chassis_yaw_gyro_pid;              //底盘旋转pid
} Chassis_Pid_t;
typedef struct
{
	float vx;                     //底盘设定速度 前进方向 前为正，单位 m/s
	float wz;                     //底盘设定旋转角速度，逆时针为�?单位 rad/s
    float chassis_yaw_set;		      // 设置底盘yaw转向期望绝对角度

	first_order_filter_type_t chassis_cmd_slow_set_vx;
	first_order_filter_type_t chassis_cmd_slow_set_wz;

} Chassis_set_t;
typedef struct
{
	float vx;                     //底盘速度 前进方向 前为正，单位 m/s
	float wz;                     //底盘旋转角速度，逆时针为�?单位 rad/s
} Chassis_ref_t;
typedef struct
{
	chassis_mode_e chassis_mode;			// 底盘控制状态机
	chassis_mode_e last_chassis_mode;		// 底盘上次控制状态机
	int8_t last_normol_channel;                //上一次遥控器开关所在的位置
} Chassis_mode_t;

typedef struct
{
	const Remote_Info_Typedef *chassis_RC;
	const float *chassis_INS_angle;
	const float *chassis_imu_gyro;
	const float *chassis_imu_accel;

	Chassis_mode_t mode;
	Chassis_Motor_t chassis_motor[4];
	Chassis_Pid_t chassis_pid;
	Chassis_set_t state_set;
	Chassis_ref_t state_ref;
} chassis_move_t;

//
// 获取底盘结构体指�?const chassis_move_t *get_chassis_control_point(void);
const chassis_move_t *get_chassis_control_point(void);
const Chassis_ref_t *get_chassis_ref_point(void);
void chassis_set_motor_speed_pid(fp32 kp, fp32 ki, fp32 kd);
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
bool is_chassis_init_done(void);

#endif
