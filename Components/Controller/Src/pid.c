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

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/**
 * @brief 初始化 PID 参数。
 * @param pid: 指向 PID_Info_TypeDef 结构体的指针，
 *         包含 PID 控制器的信息。
 * @param para: 指向浮点数组的指针，
 *         包含 PID 控制器的参数。
 * @retval pid 错误状态
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *Pid,float para[PID_PARAMETER_NUM])
{
    /* 判断 PID 参数指针 */
    if(Pid->type == PID_Type_None || para == NULL)
    {
      return PID_FAILED_INIT;
    }

    /* 初始化 PID 参数 ------------------*/
    Pid->param.kp = para[0];
    Pid->param.ki = para[1];
    Pid->param.kd = para[2];
    Pid->param.Deadband = para[3];
    Pid->param.limitIntegral = para[4];
    Pid->param.limitOutput = para[5];

    /* 清除 PID 错误判断计数 */
    Pid->ERRORHandler.ErrorCount = 0;

    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief 清除 PID 计算状态。
 * @param pid: 指向 PID_Info_TypeDef 结构体的指针，
 *         包含 PID 控制器的信息。
 * @retval none
 */
static void PID_Calc_Clear(PID_Info_TypeDef *Pid)
{
	memset(Pid->Err,0,sizeof(Pid->Err));
	Pid->Integral = 0;
		
	Pid->Pout = 0;
	Pid->Iout = 0;
	Pid->Dout = 0;
	Pid->Output = 0;
}
//------------------------------------------------------------------------------


/**
 * @brief 初始化 PID 控制器。
 * @param pid: 指向 PID_Info_TypeDef 结构体的指针，
 *         包含 PID 控制器的信息。
 * @param type: PID 控制器类型
 * @param para: 指向浮点数组的指针，
 *         包含 PID 控制器的参数。
 * @retval pid 错误状态
 */
void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM])
{
		Pid->type = type;

		Pid->PID_Calc_Clear = PID_Calc_Clear;
    Pid->PID_Param_Init = PID_Param_Init;

		Pid->PID_Calc_Clear(Pid);
    Pid->ERRORHandler.Status = Pid->PID_Param_Init(Pid, para);
}
//------------------------------------------------------------------------------


/**
  * @brief  判断 PID 错误状态
  * @param pid: 指向 PID_Info_TypeDef 结构体的指针，
  *         包含 PID 控制器的信息。
  * @retval None
  */
static void PID_ErrorHandle(PID_Info_TypeDef *Pid)
{
		/* 判断 NAN/INF */
		if(isnan(Pid->Output) == true || isinf(Pid->Output)==true)
		{
				Pid->ERRORHandler.Status = PID_CALC_NANINF;
		}
}
//------------------------------------------------------------------------------

/**
  * @brief  计算 PID 控制器
  * @param  *pid 指向 PID_TypeDef_t 结构体的指针，
  *              包含指定 PID 的配置信息。
  * @param  Target  PID 控制器的目标值
  * @param  Measure PID 控制器的测量值
  * @retval PID 输出
  */
float f_PID_Calculate(PID_Info_TypeDef *Pid, float target,float measure)
{		
  /* 更新 PID 错误状态 */
  PID_ErrorHandle(Pid);
  if(Pid->ERRORHandler.Status != PID_ERROR_NONE)
  {
    Pid->PID_Calc_Clear(Pid);
    return 0;
  }
  
  /* 更新目标/测量值 */
  Pid->target = target;
  Pid->measure = measure;

  /* 更新误差 */
	Pid->Err[2] = Pid->Err[1];
	Pid->Err[1] = Pid->Err[0];
	Pid->Err[0] = Pid->target - Pid->measure;
		
  if(fabsf(Pid->Err[0]) >= Pid->param.Deadband)
  {
		/* 更新 PID 控制器输出 */
		if(Pid->type == PID_POSITION)
		{
      /* 更新 PID 积分项 */
      if(Pid->param.ki != 0)
        Pid->Integral += Pid->Err[0];
      else
        Pid->Integral = 0;

      /* 限制积分项 */
      VAL_LIMIT(Pid->Integral,-Pid->param.limitIntegral,Pid->param.limitIntegral);
      
      /* 更新比例/积分/微分输出 */
      Pid->Pout = Pid->param.kp * Pid->Err[0];
      Pid->Iout = Pid->param.ki * Pid->Integral;
      Pid->Dout = Pid->param.kd * (Pid->Err[0] - Pid->Err[1]);
      
      /* 更新 PID 输出 */
      Pid->Output = Pid->Pout + Pid->Iout + Pid->Dout;
      VAL_LIMIT(Pid->Output,-Pid->param.limitOutput,Pid->param.limitOutput);
		}
		else if(Pid->type == PID_VELOCITY)
		{
      /* 更新比例/积分/微分输出 */
      Pid->Pout = Pid->param.kp * (Pid->Err[0] - Pid->Err[1]);
      Pid->Iout = Pid->param.ki * (Pid->Err[0]);
      Pid->Dout = Pid->param.kd * (Pid->Err[0] - 2.f*Pid->Err[1] + Pid->Err[2]);

      /* 更新 PID 输出 */
      Pid->Output += Pid->Pout + Pid->Iout + Pid->Dout;
      VAL_LIMIT(Pid->Output,-Pid->param.limitOutput,Pid->param.limitOutput);
		}
  }

  return Pid->Output;
}
//------------------------------------------------------------------------------
