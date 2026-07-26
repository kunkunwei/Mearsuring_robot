/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : PID 功能实现
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 待完善
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief VAL_LIMIT 宏，用于限制指定变量的取值范围。
 * @param x: 指定变量
 * @param min: 指定变量的最小值
 * @param max: 指定变量的最大值
 * @retval none
 */
#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

/**
 * @brief PID 参数数量宏定义
 */
#ifndef PID_PARAMETER_NUM
#define PID_PARAMETER_NUM 6							
#endif

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 包含 PID 控制器错误状态的枚举类型。
 */
typedef enum
{
    PID_ERROR_NONE = 0x00U,        /*!< 无错误 */
    PID_FAILED_INIT = 0x01U,        /*!< 初始化失败 */
		PID_CALC_NANINF = 0x02U,      /*!< 产生非数字(NaN)或无穷 */
    PID_Status_NUM,
}PID_Status_e;

/**
 * @brief 包含 PID 控制器类型的枚举类型。
 */
typedef enum
{
		PID_Type_None = 0x00U,         /*!< 无类型 */
		PID_POSITION = 0x01U,          /*!< 位置式 PID */
		PID_VELOCITY = 0x02U,          /*!< 增量式 PID */
    PID_TYPE_NUM,
}PID_Type_e;

/**
 * @brief 包含 PID 错误处理信息的结构体。
 */
typedef struct
{
    uint16_t ErrorCount;    /*!< 错误状态判断次数 */
    PID_Status_e Status;    /*!< 错误状态 */
}PID_ErrorHandler_Typedef;

/**
 * @brief 包含 PID 控制器参数的结构体。
 */
typedef struct
{
    float kp;             /*!< 比例增益 */
    float ki;             /*!< 积分增益 */
    float kd;             /*!< 微分增益 */

		float Deadband;       /*!< 响应死区 */
    float limitIntegral;  /*!< 积分限幅 */
    float limitOutput;    /*!< 输出限幅 */
}PID_Parameter_Typedef;

/**
 * @brief 包含 PID 控制器信息的结构体。
 */
typedef struct _PID_TypeDef
{
		PID_Type_e type;    /*!< PID 类型 */

		float target;       /*!< 目标值 */
		float measure;      /*!< 测量值 */

    float Err[3];       /*!< 当前误差/上次误差/上上次误差 */
		float Integral;     /*!< 积分项 */

    float Pout;         /*!< 比例输出 */
    float Iout;         /*!< 积分输出 */
    float Dout;         /*!< 微分输出 */
    float Output;       /*!< PID 输出 */

		PID_Parameter_Typedef param;            /*!< 参数结构体 */
    PID_ErrorHandler_Typedef ERRORHandler;  /*!< PID 错误处理结构体 */

    /**
     * @brief 指向初始化 PID 参数函数的指针。
     * @param pid: 指向 _PID_TypeDef 结构体的指针，
     *         包含 PID 控制器的信息。
     * @param para: 指向浮点数组的指针，
     *         包含 PID 控制器的参数。
     * @retval pid 错误状态
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *pid,float *para);

    /**
     * @brief 指向清除 PID 计算状态函数的指针。
     * @param pid: 指向 _PID_TypeDef 结构体的指针，
     *         包含 PID 控制器的信息。
     * @retval none
     */
		void (*PID_Calc_Clear)(struct _PID_TypeDef *pid);
				
}PID_Info_TypeDef;


/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 初始化 PID 控制器。
 */
extern void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM]);
/**
  * @brief  计算 PID 控制器
  */
extern float f_PID_Calculate(PID_Info_TypeDef *Pid, float target,float measure);

#endif //CONTROLLER_PID_H

