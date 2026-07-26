/* USER CODE BEGIN Header */
/**
 *******************************************************************************
 * @file           : bmi088.c
 * @brief          : BMI088六轴惯性测量单元接口函数实现文件
 * @author         : Yan Yuanbin
 * @date           : 2023/04/27
 * @version        : v1.0
 ******************************************************************************
 * @attention      : 实现BMI088传感器的初始化、数据读取、校准等功能
 *                  支持SPI通信协议，提供完整的IMU数据处理接口
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tick.h"

/* Private define ------------------------------------------------------------*/
#if defined(BMI088_USE_SPI)

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  写入单个寄存器值到传感器
 * @param  reg  寄存器地址
 * @param  data 写入数据
 * @retval None
 */
static void BMI088_Write_Single_Reg(uint8_t, uint8_t);

/**
 * @brief  从传感器读取单个寄存器值
 * @param  reg  寄存器地址
 * @param  data 读取数据指针
 * @retval None
 */
static void BMI088_Read_Single_Reg(uint8_t, uint8_t *);

/**
 * @brief  从传感器读取多个寄存器值
 * @param  reg  起始寄存器地址
 * @param  data 读取数据缓冲区指针
 * @param  len  读取数据长度
 * @retval None
 */
static void BMI088_Read_Multi_Reg(uint8_t, uint8_t *, uint8_t);

/**
 * @brief 加速度计单寄存器写入宏定义
 * @param reg  指定的寄存器地址
 * @param data 写入的单字节数据
 * @note  完成片选拉低、写入、片选拉高的完整时序
 */
#define BMI088_Accel_Write_Single_Reg(reg, data) \
  {                                              \
    BMI088_ACCEL_NS_L();                         \
    BMI088_Write_Single_Reg((reg), (data));      \
    BMI088_ACCEL_NS_H();                         \
  }
//------------------------------------------------------------------------------

/**
 * @brief 加速度计单寄存器读取宏定义
 * @param reg  指定的寄存器地址
 * @param data 读取的单字节数据
 * @note  加速度计读取需要发送读命令(reg|0x80)和一个虚拟字节(0x55)
 */
#define BMI088_Accel_Read_Single_Reg(reg, data) \
  {                                             \
    BMI088_ACCEL_NS_L();                        \
    BMI088_Read_Write_Byte((reg) | 0x80);       \
    BMI088_Read_Write_Byte(0x55);               \
    (data) = BMI088_Read_Write_Byte(0x55);      \
    BMI088_ACCEL_NS_H();                        \
  }
//------------------------------------------------------------------------------

/**
 * @brief 加速度计多寄存器读取宏定义
 * @param reg  起始寄存器地址
 * @param data 读取数据缓冲区
 * @param len  读取数据长度
 * @note  用于批量读取连续寄存器数据，提高读取效率
 */
#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
  {                                                 \
    BMI088_ACCEL_NS_L();                            \
    BMI088_Read_Write_Byte((reg) | 0x80);           \
    BMI088_Read_Multi_Reg(reg, data, len);          \
    BMI088_ACCEL_NS_H();                            \
  }
//------------------------------------------------------------------------------

/**
 * @brief 陀螺仪单寄存器写入宏定义
 * @param reg  指定的寄存器地址
 * @param data 写入的单字节数据
 * @note  完成片选拉低、写入、片选拉高的完整时序
 */
#define BMI088_Gyro_Write_Single_Reg(reg, data) \
  {                                             \
    BMI088_GYRO_NS_L();                         \
    BMI088_Write_Single_Reg((reg), (data));     \
    BMI088_GYRO_NS_H();                         \
  }
//------------------------------------------------------------------------------

/**
 * @brief 陀螺仪单寄存器读取宏定义
 * @param reg  指定的寄存器地址
 * @param data 读取的单字节数据
 * @note  陀螺仪读取相对简单，直接发送寄存器地址即可
 */
#define BMI088_Gyro_Read_Single_Reg(reg, data) \
  {                                            \
    BMI088_GYRO_NS_L();                        \
    BMI088_Read_Single_Reg((reg), &(data));    \
    BMI088_GYRO_NS_H();                        \
  }
//------------------------------------------------------------------------------

/**
 * @brief 陀螺仪多寄存器读取宏定义
 * @param reg  起始寄存器地址
 * @param data 读取数据缓冲区
 * @param len  读取数据长度
 * @note  用于批量读取陀螺仪连续寄存器数据
 */
#define BMI088_Gyro_Read_Multi_Reg(reg, data, len) \
  {                                                \
    BMI088_GYRO_NS_L();                            \
    BMI088_Read_Multi_Reg((reg), (data), (len));   \
    BMI088_GYRO_NS_H();                            \
  }
//------------------------------------------------------------------------------

#endif

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 加速度计初始化寄存器配置表
 * @note  配置采样频率为3倍陀螺仪采样频率，以提供更高精度的数据
 */
static float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN; // 加速度计灵敏度(使用6g量程)

/**
 * @brief 陀螺仪灵敏度系数
 * @note  使用2000°/s量程对应的灵敏度
 */
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

/**
 * @brief BMI088全局信息结构体实例
 * @note  static: 只允许本 TU 内的函数（BMI088_Info_Update）写入，
 *        外部通过 BMI088_Update() / BMI088_GetInfo() 访问
 */
static BMI088_Info_Typedef BMI088_Info;

/**
 * @brief BMI088加速度计配置数据和错误状态表
 * @note  每行格式: {寄存器地址, 配置数据, 错误状态码}
 */
static uint8_t Accel_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        /* 开启加速度计电源 */
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},

        /* 设置为活跃模式(非暂停模式) */
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},

        /* 加速度计配置: 正常模式 + 800Hz采样率 + 必须设置位 */
        {BMI088_ACC_CONF, (BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set), BMI088_ACC_CONF_ERROR},

        /* 设置加速度计量程为±6g */
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},

        /* INT1引脚配置: 使能 + 推挽输出 + 低电平有效 */
        {BMI088_INT1_IO_CTRL, (BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR},

        /* 中断映射: 数据就绪中断映射到INT1引脚 */
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};

/**
 * @brief BMI088陀螺仪配置数据和错误状态表
 * @note  每行格式: {寄存器地址, 配置数据, 错误状态码}
 */
static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        /* 设置角速度量程为±2000°/s */
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},

        /* 数据传输速率和带宽设置: 2000°/s量程 + 230Hz带宽 + 必须设置位 */
        {BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR},

        /* 电源模式选择: 正常工作模式 */
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},

        /* 数据中断触发寄存器: 使能数据就绪中断 */
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},

        /* 中断引脚触发寄存器: INT3推挽输出 + 低电平有效 */
        {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},

        /* 中断映射寄存器: 数据就绪中断映射到INT3引脚 */
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

/**
 * @brief  BMI088加速度计初始化函数
 * @details 通过向传感器内部配置寄存器写入指定数据来初始化加速度计
 * @param  None
 * @retval BMI088_Status_e 初始化状态码
 */
static BMI088_Status_e BMI088_Accel_Init(void)
{
  uint8_t res = 0;

  /* check the communication ------------------------------------------------*/
  /* read the accelerator ID address */
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  /* waiting 150us */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  /* 再次读取芯片ID确认通信 */
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  /* 等待150us通信稳定 */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* 加速度计软件复位 ------------------------------------------------*/
  /* 向地址0x7E的ACC_SOFTRESET寄存器写入0xB6进行软件复位 */
  BMI088_Accel_Write_Single_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  /* 软件复位等待时间80ms */
  Delay_ms(BMI088_LONG_DELAY_TIME);

  /* 再次检查通信 ------------------------------------------------*/
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* 如果0x00寄存器的值不是0x1E，表示复位失败 */
  if (res != BMI088_ACC_CHIP_ID_VALUE)
  {
    return BMI088_NO_SENSOR;
  }

  /* config the accelerator sensor */
  for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
  {
    /* Write the configuration values in the internal configuration registers of the sensor : */
    /*!< [0][0]  BMI088_ACC_PWR_CTRL 0x7D                Turn on or off the accelerator register */
    /*!< [0][1]  BMI088_ACC_ENABLE_ACC_ON 0x04           Turn on the accelerator */
    /*!< [1][0]  BMI088_ACC_PWR_CONF 0x7C                Switches the accelerator mode register */
    /*!< [1][1]  BMI088_ACC_PWR_ACTIVE_MODE 0x00         start  */
    /*!< [2][0]  BMI088_ACC_CONF 0x40                    accelerator config register */
    /*!< [2][1]  BMI088_ACC_CONF_DATA 0xAB               BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS): normal sampling frequency  */
    /*!<                                                 | BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS): 800hz output frequency */
    /*!<                                                 | BMI088_ACC_CONF_MUST_Set 0x80 */
    /*!< [3][0]  BMI088_ACC_RANGE 0x41                   accelerator scoping register */
    /*!< [3][1]  BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)   +-3g */
    /*!< [4][0]  BMI088_INT1_IO_CTRL 0x53                configure INT1 input and output pins */
    /*!< [4][1]  BMI088_INT1_IO_CTRL_DATA 0x8            BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS): configure INT1 as output pins */
    /*!<                                                 | BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS): push-pull output */
    /*!<                                                 | BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS): pull down */
    /*!< [5][0]  BMI088_INT_MAP_DATA 0x58                interrupts mapping register */
    /*!< [5][1]  BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)  interrupts are mapped to INT1 */
    BMI088_Accel_Write_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* read the configuration */
    BMI088_Accel_Read_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* check the configuration and return the specified error */
    if (res != Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
    {
      return (BMI088_Status_e)Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
    }
  }

  /* no error */
  return BMI088_NO_ERROR;
}
//------------------------------------------------------------------------------

/**
 * @brief Initializes the gyro according to writing the specified data
 *        to the internal configuration registers of the sensor.
 * @param None
 * @retval None
 */
static BMI088_Status_e BMI088_Gyro_Init(void)
{
  uint8_t res = 0;

  /* check the communication ------------------------------------------------*/
  /* read the gyro ID address */
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  /* waiting 150us */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  /* read again */
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  /* waiting 150us */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* gyro software reset ------------------------------------------------*/
  /* write 0xB6 to the register GYRO_SOFTRESET that address is 0x14 to reset software */
  BMI088_Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  /* software reset waiting time, there is 80ms */
  Delay_ms(BMI088_LONG_DELAY_TIME);

  /* check the communication again ------------------------------------------------*/
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* if the value of the 0x00 register is not 0x0F,reset failed */
  if (res != BMI088_GYRO_CHIP_ID_VALUE)
  {
    return BMI088_NO_SENSOR;
  }

  /* config the gyro sensor */
  for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
  {
    /* Write the configuration values in the internal configuration registers of the sensor : */
    /*!< [0][0]  BMI088_GYRO_RANGE 0x0F                angular rate range and resolution register */
    /*!< [0][1]  BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  //+-2000°/s */
    /*!< [1][0]  BMI088_GYRO_BANDWIDTH 0x10               set rate data filter bandwidth and output rate register */
    /*!< [1][1]  BMI088_GYRO_2000_532_HZ                  set the data transmission rate is 2kHZ, and the bandwidth is 532hz */
    /*!< [2][0]  BMI088_GYRO_LPM1 0x11                    power mode selection register */
    /*!< [2][1]  BMI088_GYRO_NORMAL_MODE 0x00             normal mode */
    /*!< [3][0]  BMI088_GYRO_CTRL 0x15                    data interrupt trigger Register */
    /*!< [3][1]  BMI088_DRDY_ON 0x80                      allow new data to trigger a new data interrupt */
    /*!< [4][0]  BMI088_GYRO_INT3_INT4_IO_CONF 0x16       interrupt pin configuration register */
    /*!< [4][1]  BMI088_GYRO_INT3_INT4_IO_CONF_DATA 0x0   BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS): INT3 push-pull output  */
    /*!<                                                  | BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS): INT3 pull down  */
    /*!< [5][0]  BMI088_GYRO_INT3_INT4_IO_MAP 0x18        interrupt map register */
    /*!< [5][1]  BMI088_GYRO_DRDY_IO_INT3 0x01            mapping to INT3 */
    BMI088_Gyro_Write_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* read the configuration */
    BMI088_Gyro_Read_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* check the configuration and return the specified error */
    if (res != Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
    {
      return (BMI088_Status_e)Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
    }
  }

  /* no error */
  return BMI088_NO_ERROR;
}
//------------------------------------------------------------------------------

/**
 * @brief Updates the BMI088 offsets.
 * @param BMI088_Info: pointer to a BMI088_Info_Typedef structure that
 *         contains the information  for the BMI088.
 * @retval None
 */
static void BMI088_Offset_Update(BMI088_Info_Typedef *BMI088_Info)
{
#if IMU_Calibration_ENABLE /* ENABLE the BMI088 Calibration */

  uint8_t buf[8] = {
      0,
  };

  for (uint16_t i = 0; i < 5000; i++)
  {
    /* read the accelerator multi data */
    BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->mpu_info.accelx = (int16_t)((buf[1]) << 8) | buf[0];
    BMI088_Info->mpu_info.accely = (int16_t)((buf[3]) << 8) | buf[2];
    BMI088_Info->mpu_info.accelz = (int16_t)((buf[5]) << 8) | buf[4];

    /* read the gyro multi data */
    BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
      BMI088_Info->mpu_info.gyrox = (int16_t)((buf[3]) << 8) | buf[2];
      BMI088_Info->mpu_info.gyroy = (int16_t)((buf[5]) << 8) | buf[4];
      BMI088_Info->mpu_info.gyroz = (int16_t)((buf[7]) << 8) | buf[6];

      /* update the gyro offsets */
      BMI088_Info->offsets_gyrox += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyrox;
      BMI088_Info->offsets_gyroy += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroy;
      BMI088_Info->offsets_gyroz += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroz;
    }
    /* waiting 1ms */
    Delay_ms(1);
  }

  BMI088_Info->offsets_gyrox = BMI088_Info->offsets_gyrox / 5000.f;
  BMI088_Info->offsets_gyroy = BMI088_Info->offsets_gyroy / 5000.f;
  BMI088_Info->offsets_gyroz = BMI088_Info->offsets_gyroz / 5000.f;

#else /* DISABLE the BMI088 Calibration */
  /* store the previous offsets */
  BMI088_Info->offsets_gyrox = -0.000142103309f;
  BMI088_Info->offsets_gyroy = -0.00511132634f;
  BMI088_Info->offsets_gyroz = 0.00104907132f;
#endif

  /* set the offset init flag */
  BMI088_Info->offsets_init = true;
}
//------------------------------------------------------------------------------

/**
 *
 * @brief Initializes the BMI088 according to writing the specified data
 *        to the internal configuration registers of the sensor.
 * @param None
 * @retval Status
 */
void BMI088_Init(void)
{
  BMI088_Status_e status = BMI088_NO_ERROR;

  /* Initializes the BMI088 */
  do
  {
    /* Judge the accelerator configuration */
    status |= BMI088_Accel_Init();
    /* Judge the gyro configuration */
    status |= BMI088_Gyro_Init();
    /* waiting 2ms */
    Delay_ms(2);
  } while (status);

  /* update the bmi088 offset */
  BMI088_Offset_Update(&BMI088_Info);
}
//------------------------------------------------------------------------------

/**
 * @brief Updates the BMI088 Information.
 * @param BMI088_Info: pointer to a BMI088_Info_Typedef structure that
 *         contains the information  for the BMI088.
 * @retval None
 */
void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info)
{
  uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

  /* read the accelerator multi data */
  BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
  BMI088_Info->mpu_info.accelx = (int16_t)((buf[1] << 8) | buf[0]);
  BMI088_Info->mpu_info.accely = (int16_t)((buf[3] << 8) | buf[2]);
  BMI088_Info->mpu_info.accelz = (int16_t)((buf[5] << 8) | buf[4]);

  /* converts the accelerator data */
  BMI088_Info->accel[0] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accelx;
  BMI088_Info->accel[1] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accely;
  BMI088_Info->accel[2] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accelz;

  /* read the temperature */
  BMI088_Accel_Read_Multi_Reg(BMI088_TEMP_M, buf, 2);
  BMI088_Info->mpu_info.temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
  if (BMI088_Info->mpu_info.temperature > 1023)
    BMI088_Info->mpu_info.temperature -= 2048;

  /* converts the temperature data */
  BMI088_Info->temperature = BMI088_Info->mpu_info.temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

  /* read the gyro multi data */
  BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
  /* check the ID */
  if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
  {
    BMI088_Info->mpu_info.gyrox = (int16_t)((buf[3] << 8) | buf[2]);
    BMI088_Info->mpu_info.gyroy = (int16_t)((buf[5] << 8) | buf[4]);
    BMI088_Info->mpu_info.gyroz = (int16_t)((buf[7] << 8) | buf[6]);
  }

  /* converts the gyro data */
  BMI088_Info->gyro[0] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyrox - BMI088_Info->offsets_gyrox;
  BMI088_Info->gyro[1] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroy - BMI088_Info->offsets_gyroy;
  BMI088_Info->gyro[2] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroz - BMI088_Info->offsets_gyroz;
}
//------------------------------------------------------------------------------

#if defined(BMI088_USE_SPI)
/**
 * @brief Write the single register value to the sensor
 * @param reg: the specified register address
 * @param data: the specified register value
 * @retval none
 */
static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
  /* write the address to the sensor */
  BMI088_Read_Write_Byte(reg);

  /* write the register value to the sensor */
  BMI088_Read_Write_Byte(data);
}
//------------------------------------------------------------------------------

/**
 * @brief Read the single register value to the sensor
 * @param reg: the specified register address
 * @param return_data: pointer to the specified register value
 * @retval none
 */
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *return_data)
{
  /**
   * @brief As mentioned in the manual, in the gyroscope mode,
   *        write 0x80 to the register to trigger a new data interrupt,
   *        and the other modes are the same.
   */
  BMI088_Read_Write_Byte(reg | 0x80);

  /* write/read the register value to the sensor */
  *return_data = BMI088_Read_Write_Byte(0x55);
}
//------------------------------------------------------------------------------

/**
 * @brief Read the multi register value to the sensor
 * @param reg: the specified register address
 * @param buf: pointer to the specified register value
 * @param len: the length of specified register value
 * @retval none
 */
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  /* trigger a new data interrupt */
  BMI088_Read_Write_Byte(reg | 0x80);

  while (len != 0)
  {
    /* write/read the register value to the sensor */
    *buf = BMI088_Read_Write_Byte(0x55);
    buf++;
    len--;
  }
}
//------------------------------------------------------------------------------

#endif

/**
 * @brief  触发一次BMI088传感器数据读取并更新内部静态结构体
 * @note   取代旧的 BMI088_Info_Update(&BMI088_Info) 调用方式
 */
void BMI088_Update(void)
{
  BMI088_Info_Update(&BMI088_Info);
}

/**
 * @brief  获取BMI088数据只读指针
 * @retval const BMI088_Info_Typedef* 指向内部静态结构体的只读指针
 */
const BMI088_Info_Typedef *BMI088_GetInfo(void)
{
  return &BMI088_Info;
}
