#include "buzzer_music.h"
#include "bsp_tim.h"
#include "cmsis_os.h"
#include "tim.h" // 引入tim.h以使用htim4
// 声明外部的TIM4句柄
extern TIM_HandleTypeDef htim4;
// 青花瓷乐谱数据
const music_note_t qinghua_ci[] = {
    // // 天 青 色 等 烟 雨  而 我 在 等 你
    // {NOTE_A4, 400}, {NOTE_G4, 200}, {NOTE_A4, 200}, {NOTE_C5, 400}, {NOTE_B4, 200}, {NOTE_A4, 200},
    // {NOTE_G4, 400}, {NOTE_E4, 200}, {NOTE_G4, 200}, {NOTE_A4, 600}, {REST, 100},
    // {NOTE_F4, 400}, {NOTE_G4, 200}, {NOTE_A4, 200}, {NOTE_C5, 400}, {NOTE_B4, 200}, {NOTE_A4, 200},
    // {NOTE_G4, 400}, {NOTE_E4, 200}, {NOTE_G4, 200}, {NOTE_E4, 600}, {REST, 100},
    // 月 色 被 淘 洗 结 局 如 泼 墨 画 里
    {NOTE_A4, 400}, {NOTE_B4, 200}, {NOTE_C5, 200}, {NOTE_B4, 400}, {NOTE_A4, 200}, {NOTE_G4, 200},
    // {NOTE_A4, 400}, {NOTE_F4, 200}, {NOTE_D4, 200}, {NOTE_F4, 600}, {REST, 100},
    // {NOTE_E4, 400}, {NOTE_G4, 200}, {NOTE_A4, 200}, {NOTE_G4, 400}, {NOTE_E4, 200}, {NOTE_C4, 200},
    // {NOTE_D4, 400}, {NOTE_C4, 200}, {NOTE_D4, 200}, {NOTE_E4, 600}, {REST, 200}
};
// 马里奥「地上BGM」核心乐谱（严格遵循官方旋律，适配蜂鸣器）
const music_note_t mario_ground_bgm[] = {
    // 第一乐句（标志性开头：C4→G4→A4→F4→G4→E4）
    // {NOTE_C4, 200}, {NOTE_C4, 200}, {NOTE_G4, 200}, {NOTE_G4, 200},
    // {NOTE_A4, 200}, {NOTE_A4, 200}, {NOTE_G4, 400},
    // {NOTE_F4, 200}, {NOTE_F4, 200}, {NOTE_E4, 200}, {NOTE_E4, 200},
    // {NOTE_D4, 200}, {NOTE_D4, 200}, {NOTE_C4, 400},
    // // 第二乐句（重复动机+变奏）
    // {NOTE_G4, 200}, {NOTE_G4, 200}, {NOTE_F4, 200}, {NOTE_F4, 200},
    // {NOTE_E4, 200}, {NOTE_E4, 200}, {NOTE_D4, 400},
    // {NOTE_G4, 200}, {NOTE_G4, 200}, {NOTE_F4, 200}, {NOTE_F4, 200},
    // {NOTE_E4, 200}, {NOTE_E4, 200}, {NOTE_D4, 400},
    // 第三乐句（高潮部分：A4→F4→G4→E4）
    {NOTE_C4, 200}, {NOTE_C4, 200}, {NOTE_G4, 200}, {NOTE_G4, 200},
    {NOTE_A4, 200}, {NOTE_A4, 200}, {NOTE_G4, 400},
    // {NOTE_F4, 200}, {NOTE_F4, 200}, {NOTE_E4, 200}, {NOTE_E4, 200},
    // {NOTE_D4, 200}, {NOTE_D4, 200}, {NOTE_C4, 400},
    // 结尾（休止+长音，还原原曲收尾）
    // {REST, 200}, {NOTE_C4, 800}
};
// 乐谱长度（自动计算，避免手动计数错误）
const uint16_t mario_ground_bgm_size = sizeof(mario_ground_bgm) / sizeof(music_note_t);
const uint16_t qinghua_ci_size = sizeof(qinghua_ci) / sizeof(music_note_t);


/**
  * @brief  根据频率设置蜂鸣器音高（修正TIM4时钟后）
  * @param  freq: 目标频率(Hz)
  * @retval 无
  */
static void buzzer_set_note(uint16_t freq)
{
    if(freq == 0)
    {
        buzzer_off();
        return;
    }

    // 核心修正：TIM4计数时钟=84MHz/(83+1)=1MHz → ARR = (1000000 / freq) - 1
    // 加入浮点运算避免整数截断误差，再强制转换为整数
    uint32_t arr = (uint32_t)((1000000.0f / freq) - 1);
    // 50%占空比（蜂鸣器发声最稳定）
    uint32_t ccr = arr / 2;

    // 强制同步TIM4的PSC（与初始化一致，防止其他代码篡改）
    __HAL_TIM_SET_PRESCALER(&htim4, 83);
    // 设置ARR（决定频率）
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    // 设置CCR（决定占空比）
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr);
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}




// 播放音乐核心函数
void play_music(const music_note_t* score, uint16_t score_size)
{
    for (int i = 0; i < score_size; i++)
    {
        buzzer_set_note(score[i].freq);
        osDelay(score[i].duration); // 延时控制音符时长
    }
    buzzer_off();
}


// 开机自启函数（在main函数中调用）
void boot_play_music(void)
{
    // 确保TIM4初始化完成后调用
    // HAL_Delay(500); // 系统初始化延时，避免抢占初始化资源
    // play_music(qinghua_ci, qinghua_ci_size);
    play_music(mario_ground_bgm, mario_ground_bgm_size);
}