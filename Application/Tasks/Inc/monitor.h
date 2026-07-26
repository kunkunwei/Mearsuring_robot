//
// Created by kun on 2026/1/5.
//

#ifndef WHEEL_LEG_INFANTRY_MONITOR_H
#define WHEEL_LEG_INFANTRY_MONITOR_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* ============================================================================
 * 子监控模块结构体定义
 * ========================================================================== */

/**
 * @brief 看门狗/心跳状态结构体
 * status = 0 (正常): CPU 正常运行
 * status = 1 (警告): CPU 过载或卡顿
 * status = 2 (错误): 看门狗触发，系统已重启
 */
typedef struct {
    uint32_t heartbeat_count;        // 心跳计数器 (单调递增)
    float last_heartbeat_time;       // 最后一次心跳时间戳 (毫秒)
    uint32_t watchdog_restart_count; // 看门狗重启总次数
    uint8_t restart_reason;          // 重启原因: 0=正常, 1=看门狗, 2=其他
    uint8_t status;                  // 状态: 0=正常, 1=警告, 2=错误
} Monitor_Heartbeat_t;

/**
 * @brief CAN 总线状态结构体
 */
typedef struct {
    uint32_t tx_count;       // 发送帧总数
    uint32_t rx_count;       // 接收帧总数
    uint32_t tx_error_count; // 发送错误数
    uint32_t rx_error_count; // 接收错误数
    float last_rx_time;      // 最后一次接收的时间戳 (毫秒)
    uint8_t status;          // 状态: 0=正常, 1=警告(有错误), 2=沉默(无数据)
} Monitor_CAN_t;

/**
 * @brief 通信状态结构体 (RC/遥控器/等)
 */
typedef struct {
    float last_rx_time;            // 最后接收到有效命令的时间戳 (毫秒)
    uint32_t timeout_threshold_ms; // 超时阈值 (默认 100ms)
    uint8_t status;                // 状态: 0=正常, 1=延迟, 2=超时
    uint16_t consecutive_timeouts; // 连续超时计数
} Monitor_Comm_t;


/**
 * @brief LED 控制结构体
 * 红灯: 错误, 看门狗
 * 绿灯: 通信状态
 * 蓝灯: CPU 负载警告
 */
typedef struct {
    uint8_t red_mode;   // 0=关, 1=亮, 2=慢闪(0.5Hz), 3=快闪(2Hz)
    uint8_t green_mode; // 0=关, 1=亮, 2=慢闪, 3=快闪
    uint8_t blue_mode;  // 0=关, 1=亮, 2=慢闪, 3=快闪
} Monitor_LED_State_t;

/* ============================================================================
 * 统一监控器结构体
 * ========================================================================== */

/**
 * @brief 系统监控器主结构体（包含所有子监控模块）
 */
typedef struct {
    Monitor_Heartbeat_t heartbeat;  // 心跳和看门狗监控
    Monitor_CAN_t       can;        // CAN总线监控
    Monitor_Comm_t      comm;       // 通信超时监控

    Monitor_LED_State_t led;        // LED状态控制
} SystemMonitor_t;

/* ============================================================================
 * 主监控API（简化接口）
 * ========================================================================== */

/**
 * @brief 获取系统监控器实例指针
 * @return 监控器结构体指针，通过monitor->heartbeat、monitor->can等访问各子模块
 *
 * @example
 *   SystemMonitor_t *mon = SystemMonitor_Get();
 *   uint8_t cpu_load = mon->cpu.cpu_load_percent;
 *   uint32_t can_rx = mon->can.rx_count;
 */
extern SystemMonitor_t* SystemMonitor_Get(void);

/**
 * @brief 初始化系统监控器（所有子模块）
 */
extern void SystemMonitor_Init(void);

/**
 * @brief 主监控任务（每2ms在控制循环中调用）
 * 内部调用：心跳更新、CPU循环记录
 */
extern void SystemMonitor_Process(void);

/**
 * @brief 定期任务（每50ms调用以更新LED）
 */
extern void SystemMonitor_PeriodicTask(void);

/**
 * @brief 刷新看门狗（每50-100ms调用）
 */
extern void SystemMonitor_WatchdogRefresh(void);

/**
 * @brief 获取整体系统状态
 * @return 最高状态代码: 0=正常, 1=警告, 2=错误
 */
extern uint8_t SystemMonitor_GetStatus(void);

/* ============================================================================
 * 事件记录API（在中断或回调中调用）
 * ========================================================================== */

/**
 * @brief 记录CAN发送事件
 */
extern void SystemMonitor_CAN_RecordTx(void);

/**
 * @brief 记录CAN接收事件
 */
extern void SystemMonitor_CAN_RecordRx(void);

/**
 * @brief 记录CAN错误事件
 * @param is_tx: 1=发送错误, 0=接收错误
 */
extern void SystemMonitor_CAN_RecordError(uint8_t is_tx);

/**
 * @brief 记录通信接收事件（重置超时）
 */
extern void SystemMonitor_Comm_RecordRx(void);

#ifdef __cplusplus
}
#endif

#endif //WHEEL_LEG_INFANTRY_MONITOR_H