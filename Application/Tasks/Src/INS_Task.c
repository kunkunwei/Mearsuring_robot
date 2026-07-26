/* USER CODE BEGIN Header */
#include <stdio.h>
/**
 ******************************************************************************
 * @file           : INS_Task.c
 * @brief          : 惯性导航系统任务源文件
 * @details        : 实现基于BMI088 IMU的惯性导航功能，包括：
 *                   - 扩展卡尔曼滤波器姿态解算
 *                   - 二阶低通滤波器加速度滤波
 *                   - IMU温度控制
 *                   - 多圈角度计算
 *                   - 数据获取接口函数
 * @author         : Yan Yuanbin
 * @date           : 2023/04/27
 * @version        : v1.0
 ******************************************************************************
 * @attention      : 1ms周期任务，提供高精度姿态估计
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bsp_tim.h"
#include "bmi088.h"
#include "api_quaternion.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "main.h"
#include "user_lib.h"
/**
 * @brief 惯性导航系统全局信息结构体实例
 * @details static: 只允许本 TU（INS_Task）内写入，外部通过 get_ins_info_point() 等接口只读访问
 */
static INS_Info_Typedef INS_Info;

/* Private variables ---------------------------------------------------------*/
/**
 * @brief 二阶低通滤波器系数数组
 * @details 用于加速度计数据滤波，降低高频噪声影响
 *          系数计算基于巴特沃斯滤波器设计
 */
static float INS_LPF2p_Alpha[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
 * @brief 加速度计二阶低通滤波器信息结构体数组
 * @details 分别对应X、Y、Z三轴加速度的滤波器
 */
LowPassFilter2p_Info_TypeDef INS_AcceLPF2p[3];

/**
 * @brief 四元数扩展卡尔曼滤波器状态转移矩阵初始化数据
 * @details 6x6单位矩阵，用于EKF的状态预测步骤
 *          状态向量包括四元数(q0,q1,q2,q3)和陀螺仪偏差(bx,by,bz)
 */
static float QuaternionEKF_A_Data[36] = {1, 0, 0, 0, 0, 0,
                                         0, 1, 0, 0, 0, 0,
                                         0, 0, 1, 0, 0, 0,
                                         0, 0, 0, 1, 0, 0,
                                         0, 0, 0, 0, 1, 0,
                                         0, 0, 0, 0, 0, 1};

/**
 * @brief 四元数扩展卡尔曼滤波器后验协方差矩阵初始化数据
 * @details 6x6矩阵，定义了状态估计的初始不确定性
 *          对角线元素较大表示初始估计不确定性较高
 */
static float QuaternionEKF_P_Data[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                         0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                         0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                         0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                         0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                         0.1, 0.1, 0.1, 0.1, 0.1, 100};
/**
 * @brief IMU温度控制PID参数初始化数据
 * @details PID参数: [Kp=1600, Ki=20, Kd=0, 输出下限=0, 输出上限=0, 最大输出=2000]
 *          用于将BMI088温度稳定在40°C，提高测量精度
 */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM] = {1600, 20, 0, 0, 0, 2000};

/**
 * @brief IMU温度控制PID信息结构体
 * @details 维持BMI088工作在恒定温度，减少温度漂移对测量精度的影响
 */
PID_Info_TypeDef TempCtrl_PID;

/**
 * @brief 四元数姿态估计信息结构体
 * @details 包含四元数、欧拉角等姿态解算结果
 */
Quaternion_Info_Typedef Quaternion_Info;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief 初始化惯性导航系统任务
 * @details 初始化滤波器、PID控制器、四元数EKF等组件
 */
static void INSTask_Init(void);

/**
 * @brief 控制BMI088 IMU的温度
 * @param temp BMI088当前温度值
 * @details 使用PID控制器将温度稳定在40°C，提高传感器精度和稳定性
 */
static void BMI088_Temp_Control(float temp);
first_order_filter_type_t yaw_gyro_low_filter={
    .frame_period = 1,
    .num = 0.2f,
};
first_order_filter_type_t yaw_low_filter={
    .frame_period = 1,
    .num = 0.2f,
};
/* USER CODE BEGIN Header_INS_Task */
/**
 * @brief 惯性导航系统任务主函数
 * @param argument FreeRTOS任务参数（未使用）
 * @details 1ms周期任务，执行以下操作：
 *          1. 更新BMI088传感器数据
 *          2. 对加速度数据进行低通滤波
 *          3. 使用EKF进行姿态解算
 *          4. 计算欧拉角和多圈角度
 *          5. 控制IMU温度（每5ms执行一次）
 */
/* USER CODE END Header_INS_Task */
void INS_Task(void const *argument)
{
    /* USER CODE BEGIN INS_Task */
    TickType_t systick = 0;

    /* 初始化惯性导航系统任务 */
    INSTask_Init();

    /* 无限循环 */
    for (;;) {
        systick = osKernelSysTick();

        /* 更新BMI088传感器测量数据 */
        BMI088_Update();

        /* 获取BMI088只读数据指针 */
        const BMI088_Info_Typedef *bmi = BMI088_GetInfo();

        /* 对加速度测量値进行二阶低通滤波处理 */
        INS_Info.accel[IMU_ACCEL_INDEX_PITCH] = LowPassFilter2p_Update(&INS_AcceLPF2p[0], bmi->accel[IMU_ACCEL_INDEX_PITCH]);
        INS_Info.accel[IMU_ACCEL_INDEX_YAW]   = LowPassFilter2p_Update(&INS_AcceLPF2p[2], bmi->accel[IMU_ACCEL_INDEX_YAW]);
        INS_Info.accel[IMU_ACCEL_INDEX_ROLL]  = LowPassFilter2p_Update(&INS_AcceLPF2p[1], bmi->accel[IMU_ACCEL_INDEX_ROLL]);

        /* 更新陀螺仪角速度数据（弧度制） */
        memcpy(INS_Info.gyro, bmi->gyro, sizeof(INS_Info.gyro));

        /* 更新四元数扩展卡尔曼滤波器，融合陀螺仪和加速度计数据 */
        QuaternionEKF_Update(&Quaternion_Info, INS_Info.gyro, INS_Info.accel, 0.001f);

        /* 复制欧拉角结果到INS信息结构体 */
        memcpy(INS_Info.angle, Quaternion_Info.EulerAngle, sizeof(INS_Info.angle));

        /* 更新欧拉角数据（弧度制） */
        INS_Info.pit_angle = Quaternion_Info.EulerAngle[2]; // 俯仰角
        // INS_Info.yaw_angle = -Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW];   // 偏航角
        INS_Info.rol_angle = Quaternion_Info.EulerAngle[1];  // 横滚角

        first_order_filter_cali(&yaw_low_filter,-Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]);
        INS_Info.yaw_angle=yaw_low_filter.out;

        // INS_Info.pit_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]; // 俯仰角
        // INS_Info.yaw_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW];   // 偏航角
        // INS_Info.rol_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL];  // 横滚角

        /* 更新YAW轴累积总角度（处理多圈旋转） */
        if (INS_Info.yaw_angle - INS_Info.last_yawangle < -3.141593f) {
            INS_Info.YawRoundCount++; // 正向跨越±180°边界
        } else if (INS_Info.yaw_angle - INS_Info.last_yawangle > 3.141593f) {
            INS_Info.YawRoundCount--; // 反向跨越±180°边界
        }
        if (fabs(INS_Info.yaw_gyro)<0.039f)
        {
            INS_Info.yaw_angle=INS_Info.last_yawangle;
        }
        INS_Info.last_yawangle = INS_Info.yaw_angle; // 更新上次角度值

        /* 计算包含多圈信息的总角度 */
        INS_Info.yaw_tolangle = INS_Info.yaw_angle + INS_Info.YawRoundCount * 2 * 3.141593f;

        /* 更新各轴角速度数据（弧度/秒） */
        INS_Info.pit_gyro = INS_Info.gyro[0]; // 俯仰轴角速度
        // INS_Info.yaw_gyro = -INS_Info.gyro[IMU_GYRO_INDEX_YAW];   // 偏航轴角速度
        INS_Info.rol_gyro = INS_Info.gyro[1];  // 横滚轴角速度

        first_order_filter_cali(&yaw_gyro_low_filter,-INS_Info.gyro[IMU_GYRO_INDEX_YAW]);
         INS_Info.yaw_gyro=yaw_gyro_low_filter.out;

        /* 每隔5ms执行一次温度控制（降低控制频率，避免过于频繁的温控操作） */
        if (systick % 5 == 0) {
            BMI088_Temp_Control(BMI088_GetInfo()->temperature);
        }



        /* 等待下一个1ms周期 */
        osDelayUntil(&systick, 1);
    }
    /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief 初始化惯性导航系统任务
 * @details 初始化所有必要的组件：滤波器、PID控制器、卡尔曼滤波器
 */
static void INSTask_Init(void)
{
    /* 初始化三轴加速度计的二阶低通滤波器 */
    LowPassFilter2p_Init(&INS_AcceLPF2p[0], INS_LPF2p_Alpha); // X轴(PITCH)
    LowPassFilter2p_Init(&INS_AcceLPF2p[1], INS_LPF2p_Alpha); // Y轴(ROLL)
    LowPassFilter2p_Init(&INS_AcceLPF2p[2], INS_LPF2p_Alpha); // Z轴(YAW)

    /* 初始化BMI088温度控制PID */
    PID_Init(&TempCtrl_PID, PID_VELOCITY, TemCtrl_PID_Param);

    /* 初始化四元数扩展卡尔曼滤波器 */
    /* 参数: 观测噪声=10, 采样时间=0.001s, 过程噪声=1000000, 状态转移矩阵, 协方差矩阵 */
    QuaternionEKF_Init(&Quaternion_Info, 10.f, 0.001f, 1000000.f, QuaternionEKF_A_Data, QuaternionEKF_P_Data);
}
//------------------------------------------------------------------------------
/**
 * @brief 控制BMI088 IMU的工作温度
 * @param temp BMI088当前温度测量值
 * @details 使用PID控制器将BMI088温度稳定在40°C，减少温度漂移对
 *          陀螺仪和加速度计测量精度的影响
 */
static void BMI088_Temp_Control(float temp)
{
    /* 执行PID计算，目标温度40°C */
    f_PID_Calculate(&TempCtrl_PID, 40.f, temp);

    /* 限制PID输出在0-2000范围内 */
    VAL_LIMIT(TempCtrl_PID.Output, 0, 2000);

    /* 设置加热器功率 */
    Heat_Power_Control((uint16_t)TempCtrl_PID.Output);
}
//------------------------------------------------------------------------------
/**
 * @brief 获取INS姿态角数据指针
 * @return 指向三轴欧拉角数组的指针 [PITCH, ROLL, YAW]
 * @details 提供给其他模块访问姿态角信息的接口
 */
const float *get_INS_angle_point()
{
    return &INS_Info.angle[0];
}

/**
 * @brief 获取陀螺仪角速度数据指针
 * @return 指向三轴角速度数组的指针 [PITCH, ROLL, YAW] (弧度/秒)
 * @details 提供给其他模块访问角速度信息的接口
 */
const float *get_gyro_data_point()
{
    return &INS_Info.gyro[0];
}

/**
 * @brief 获取加速度计数据指针
 * @return 指向三轴加速度数组的指针 [X, Y, Z] (m/s²)
 * @details 提供给其他模块访问滤波后加速度信息的接口
 */
const float *get_accel_data_point()
{
    return &INS_Info.accel[0];
}

/**
 * @brief 获取INS完整信息结构体指针
 * @return 指向INS_Info_Typedef结构体的常量指针
 * @details 提供给其他模块访问完整惯导信息的主要接口
 *          包含姿态角、角速度、加速度、多圈角度等所有导航信息
 */
const INS_Info_Typedef *get_ins_info_point()
{
    return &INS_Info;
}
