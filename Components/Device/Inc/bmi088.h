/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bmi088.h
  * @brief          : BMI088六轴惯性测量单元接口函数头文件
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : BMI088是博世公司的6轴IMU传感器，包含3轴加速度计和3轴陀螺仪
  *                  支持SPI和I2C通信接口，本实现采用SPI接口
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_BMI088_H
#define DEVICE_BMI088_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "bmi088_reg.h"

/* Exported defines -----------------------------------------------------------*/
/* 通信接口选择 */
#define BMI088_USE_SPI                          // 使用SPI通信接口

/* 温度转换参数 */
#define BMI088_TEMP_FACTOR 0.125f               // 温度转换因子(°C/LSB)
#define BMI088_TEMP_OFFSET 23.0f                // 温度偏移值(°C)

/* 寄存器配置数量 */
#define BMI088_WRITE_ACCEL_REG_NUM 6            // 加速度计初始化寄存器数量
#define BMI088_WRITE_GYRO_REG_NUM 6             // 陀螺仪初始化寄存器数量

/* 数据就绪标志位定义 */
#define BMI088_GYRO_DATA_READY_BIT 0            // 陀螺仪数据就绪位
#define BMI088_ACCEL_DATA_READY_BIT 1           // 加速度计数据就绪位  
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2      // 加速度计温度数据就绪位

/* 时序延时参数 */
#define BMI088_LONG_DELAY_TIME 80               // 长延时时间(ms)
#define BMI088_COM_WAIT_SENSOR_TIME 150         // 传感器通信等待时间(ms)

/* I2C设备地址定义 */
#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)   // 加速度计I2C地址
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)    // 陀螺仪I2C地址

/* 加速度计量程选择 */
#define BMI088_ACCEL_RANGE_3G                   // 当前使用: ±3g量程
//#define BMI088_ACCEL_RANGE_6G                 // 可选: ±6g量程
//#define BMI088_ACCEL_RANGE_12G                // 可选: ±12g量程  
//#define BMI088_ACCEL_RANGE_24G                // 可选: ±24g量程

/* 陀螺仪量程选择 */
#define BMI088_GYRO_RANGE_2000                  // 当前使用: ±2000°/s量程
//#define BMI088_GYRO_RANGE_1000                // 可选: ±1000°/s量程
//#define BMI088_GYRO_RANGE_500                 // 可选: ±500°/s量程
//#define BMI088_GYRO_RANGE_250                 // 可选: ±250°/s量程
//#define BMI088_GYRO_RANGE_125                 // 可选: ±125°/s量程

/* 加速度计灵敏度系数 (g/LSB) */
#define BMI088_ACCEL_3G_SEN 0.0008974358974f    // ±3g量程灵敏度: 0.0009g/LSB
#define BMI088_ACCEL_6G_SEN 0.00179443359375f   // ±6g量程灵敏度: 0.0018g/LSB
#define BMI088_ACCEL_12G_SEN 0.0035888671875f   // ±12g量程灵敏度: 0.0036g/LSB
#define BMI088_ACCEL_24G_SEN 0.007177734375f    // ±24g量程灵敏度: 0.0072g/LSB

/* 陀螺仪灵敏度系数 (rad/s/LSB) */
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f    // ±2000°/s灵敏度
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f // ±1000°/s灵敏度
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f  // ±500°/s灵敏度
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f  // ±250°/s灵敏度
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f // ±125°/s灵敏度

/* Exported types ------------------------------------------------------------*/
/**
 * @brief BMI088状态码枚举定义
 * @details 定义了BMI088初始化和运行过程中可能出现的各种状态和错误码
 */
typedef enum
{
    BMI088_NO_ERROR                     = 0x00,  // 无错误，正常状态
    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,  // 加速度计电源控制错误
    BMI088_ACC_PWR_CONF_ERROR           = 0x02,  // 加速度计电源配置错误
    BMI088_ACC_CONF_ERROR               = 0x03,  // 加速度计配置错误
    BMI088_ACC_SELF_TEST_ERROR          = 0x04,  // 加速度计自检错误
    BMI088_ACC_RANGE_ERROR              = 0x05,  // 加速度计量程设置错误
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,  // 中断1 IO控制错误
    BMI088_INT_MAP_DATA_ERROR           = 0x07,  // 中断映射数据错误
    BMI088_GYRO_RANGE_ERROR             = 0x08,  // 陀螺仪量程设置错误
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,  // 陀螺仪带宽设置错误
    BMI088_GYRO_LPM1_ERROR              = 0x0A,  // 陀螺仪低功耗模式1错误
    BMI088_GYRO_CTRL_ERROR              = 0x0B,  // 陀螺仪控制错误
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,  // 陀螺仪中断3/4 IO配置错误
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,  // 陀螺仪中断3/4 IO映射错误

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,  // 加速度计自检失败
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,  // 陀螺仪自检失败
    BMI088_NO_SENSOR                    = 0xFF,  // 传感器不存在或通信失败
}BMI088_Status_e;

/**
 * @brief MPU原始数据结构体定义
 * @details 存储从BMI088传感器接收到的原始16位数据
 */
typedef struct
{
	int16_t accelx;      /*!< X轴加速度原始数据 */
	int16_t accely;      /*!< Y轴加速度原始数据 */
	int16_t accelz;      /*!< Z轴加速度原始数据 */

	int16_t gyrox;       /*!< X轴角速度原始数据 */
	int16_t gyroy;       /*!< Y轴角速度原始数据 */
	int16_t gyroz;       /*!< Z轴角速度原始数据 */

	int16_t temperature; /*!< 温度原始数据 */
}MPU_Info_Typedef;

/**
 * @brief BMI088完整信息结构体定义
 * @details 包含BMI088的所有数据：原始数据、转换后数据、零点偏移等
 */
typedef struct
{
    bool offsets_init;         /*!< 零点偏移初始化标志 */

    float accel[3];            /*!< 转换后的加速度数据 [X,Y,Z] (m/s²) */
    float gyro[3];             /*!< 转换后的陀螺仪数据 [X,Y,Z] (rad/s) */
    float temperature;         /*!< 转换后的温度数据 (°C) */

    MPU_Info_Typedef mpu_info; /*!< BMI088原始数据结构体 */

    float offsets_gyrox;       /*!< X轴陀螺仪零点偏移 (rad/s) */
    float offsets_gyroy;       /*!< Y轴陀螺仪零点偏移 (rad/s) */
    float offsets_gyroz;       /*!< Z轴陀螺仪零点偏移 (rad/s) */
}BMI088_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  BMI088传感器初始化函数
 * @details 通过向传感器内部配置寄存器写入指定数据来初始化BMI088
 * @retval None
 */
extern void BMI088_Init(void);

/**
 * @brief  触发一次BMI088传感器数据读取（内部自动更新静态结构体）
 * @retval None
 */
void BMI088_Update(void);

/**
 * @brief  获取BMI088数据只读指针
 * @retval const BMI088_Info_Typedef* 指向内部静态结构体的只读指针
 */
const BMI088_Info_Typedef *BMI088_GetInfo(void);

#endif //DEVICE_BMI088_H
