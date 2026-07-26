/**
 ******************************************************************************
 * @file           : system_monitor.c
 * @brief          : 系统监控实现
 * @author         :
 * @date           : 2026/01/05
 * @version        : v2.0
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor.h"
#include "main.h" // 用于 DWT_GetTimeline_ms()
// #include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <math.h>
// #include <string.h>

#include "bsp_dwt.h"


/* 外部 IWDG 句柄 (如果不可用，注释掉 Refresh 调用) */
// extern IWDG_HandleTypeDef hiwdg;

/* Private defines -----------------------------------------------------------*/
#define WATCHDOG_TIMEOUT_MS   1500  // 看门狗超时阈值 (1500ms)
#define COMM_TIMEOUT_MS       100  // 通信超时阈值 (100ms)
#define BLINK_SLOW_PERIOD_MS  1000 // 慢闪周期 0.5 Hz = 每 1 秒一个完整周期
#define BLINK_FAST_PERIOD_MS  500  // 快闪周期 2 Hz = 每 0.5 秒一个完整周期

/* Private types -------------------------------------------------------------*/
// 已在头文件中定义 SystemMonitor_t

/* Private variables ---------------------------------------------------------*/
static SystemMonitor_t g_monitor           = {0}; // 全局监控器实例
static float g_last_heartbeat_warning_time = 0;



/* ============================================================================
 * 阶段1: 看门狗和心跳监控实现
 * ========================================================================== */

void Monitor_Heartbeat_Init(void)
{
    // 检查是否由看门狗重启
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        g_monitor.heartbeat.status = 2; // 错误
        g_monitor.heartbeat.watchdog_restart_count++;
        g_monitor.heartbeat.restart_reason = 1; // 看门狗重启
        __HAL_RCC_CLEAR_RESET_FLAGS();
    } else {
        g_monitor.heartbeat.status         = 0; // 正常
        g_monitor.heartbeat.restart_reason = 0;
    }

    g_monitor.heartbeat.heartbeat_count     = 0;
    g_monitor.heartbeat.last_heartbeat_time = DWT_GetTimeline_ms();
}

void Monitor_Heartbeat_Beat(void)
{
    float now = DWT_GetTimeline_ms();

    // 递增计数器
    g_monitor.heartbeat.heartbeat_count++;
    g_monitor.heartbeat.last_heartbeat_time = now;

    // 检查停滞 (每 2ms 调用，50 次未增长 = 100ms 停滞)
    static uint32_t missed_beats    = 0;
    static uint32_t last_beat_count = 0;

    if (g_monitor.heartbeat.heartbeat_count == last_beat_count) {
        missed_beats++;
    } else {
        missed_beats    = 0;
        last_beat_count = g_monitor.heartbeat.heartbeat_count;
    }

    if (missed_beats > 50) { // > 100ms 停滞
        if (g_monitor.heartbeat.status == 0) {
            g_monitor.heartbeat.status    = 1; // 警告
            g_last_heartbeat_warning_time = now;
        }
    } else {
        g_monitor.heartbeat.status = 0; // 正常
    }
}

void Monitor_Watchdog_Refresh(void)
{
    // 刷新 IWDG 以防止超时
    // HAL_IWDG_Refresh(&hiwdg);
}

Monitor_Heartbeat_t *Monitor_GetHeartbeat(void)
{
    return &g_monitor.heartbeat;
}

uint32_t Monitor_GetRestartCount(void)
{
    return g_monitor.heartbeat.watchdog_restart_count;
}

uint8_t Monitor_GetHeartbeatStatus(void)
{
    return g_monitor.heartbeat.status;
}




/* ============================================================================
 * LED CONTROL IMPLEMENTATION
 * ========================================================================== */

// LED pin definitions (from main.h)
// #define LED_R_Pin       GPIO_PIN_12
// #define LED_R_GPIO_Port GPIOH
// #define LED_G_Pin       GPIO_PIN_11
// #define LED_G_GPIO_Port GPIOH
// #define LED_B_Pin       GPIO_PIN_10
// #define LED_B_GPIO_Port GPIOH

static void led_set(uint8_t led, uint8_t on)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin       = 0;

    if (led == 0) { // Red
        port = LED_R_GPIO_Port;
        pin  = LED_R_Pin;
    } else if (led == 1) { // Green
        port = LED_G_GPIO_Port;
        pin  = LED_G_Pin;
    } else if (led == 2) { // Blue
        port = LED_B_GPIO_Port;
        pin  = LED_B_Pin;
    }

    if (port != NULL) {
        HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void led_blink(uint8_t led, uint8_t mode)
{
    float now     = DWT_GetTimeline_ms();
    uint8_t state = 0;

    if (mode == 0) { // Off
        state = 0;
    } else if (mode == 1) { // On
        state = 1;
    } else if (mode == 2) { // Slow Blink (0.5Hz)
        state = (((uint32_t)now / BLINK_SLOW_PERIOD_MS) % 2) == 0 ? 1 : 0;
    } else if (mode == 3) { // Fast Blink (2Hz)
        state = (((uint32_t)now / BLINK_FAST_PERIOD_MS) % 2) == 0 ? 1 : 0;
    }

    led_set(led, state);
}

Monitor_LED_State_t *Monitor_GetLEDState(void)
{
    return &g_monitor.led;
}

void Monitor_SetLED(uint8_t led, uint8_t mode)
{
    if (led == 0) {
        g_monitor.led.red_mode = mode;
    } else if (led == 1) {
        g_monitor.led.green_mode = mode;
    } else if (led == 2) {
        g_monitor.led.blue_mode = mode;
    }
}

void Monitor_UpdateLEDs(void)
{
    // Red LED: Watchdog/Heartbeat errors
    // 0=Off, 1=On (error), 2=Slow Blink (warning)
    uint8_t hb_status = Monitor_GetHeartbeatStatus();
    if (hb_status == 2) {        // Error (WDG restart detected)
        Monitor_SetLED(0, 1);    // Red On
    } else if (hb_status == 1) { // Warning (stall detected)
        Monitor_SetLED(0, 2);    // Red Slow Blink
    } else {
        Monitor_SetLED(0, 0); // Red Off
    }


    // Apply the blinking
    led_blink(0, g_monitor.led.red_mode);
    led_blink(1, g_monitor.led.green_mode);
    led_blink(2, g_monitor.led.blue_mode);
}

/* ============================================================================
 * MAIN MONITOR INTERFACE
 * ========================================================================== */

void Monitor_Init(void)
{
    Monitor_Heartbeat_Init();
    memset(&g_monitor.led, 0, sizeof(Monitor_LED_State_t));
}

void Monitor_Process(void)
{
    // Called every 2ms in control loop
    Monitor_Heartbeat_Beat();
}

void Monitor_PeriodicTask(void)
{
    // Called every ~50ms to update LEDs
    Monitor_UpdateLEDs();
}

uint8_t Monitor_GetSystemStatus(void)
{
    uint8_t max_status = 0;

    max_status = (Monitor_GetHeartbeatStatus() > max_status) ? Monitor_GetHeartbeatStatus() : max_status;

    return max_status;
}
/* ============================================================================
 * 主监控器API实现
 * ========================================================================== */

SystemMonitor_t *SystemMonitor_Get(void)
{
    return &g_monitor;
}

void SystemMonitor_Init(void)
{
    Monitor_Heartbeat_Init();
    memset(&g_monitor.led, 0, sizeof(Monitor_LED_State_t));
}

void SystemMonitor_Process(void)
{
    // 每2ms调用：心跳更新 + CPU循环记录
    Monitor_Heartbeat_Beat();
}

void SystemMonitor_PeriodicTask(void)
{
    // 每50ms调用：更新LED
    Monitor_UpdateLEDs();
}

void SystemMonitor_WatchdogRefresh(void)
{
    Monitor_Watchdog_Refresh();
}

uint8_t SystemMonitor_GetStatus(void)
{
    return Monitor_GetSystemStatus();
}
