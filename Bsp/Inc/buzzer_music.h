#ifndef BUZZER_MUSIC_H
#define BUZZER_MUSIC_H

#include "main.h"

// 音符频率定义
// 音符频率定义（修正后，保证计算精度）
// #define NOTE_C3  131
// #define NOTE_D3  147
// #define NOTE_E3  165
// #define NOTE_F3  175
// #define NOTE_G3  196
// #define NOTE_A3  220
// #define NOTE_B3  247
// #define NOTE_C4  262
// #define NOTE_D4  294
// #define NOTE_E4  330
// #define NOTE_F4  349
// #define NOTE_G4  392
// #define NOTE_A4  440
// #define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
// #define REST     0   // 休止符

// 音符频率定义（精准匹配蜂鸣器，基于C大调）
#define REST     0    // 休止符
#define NOTE_C3  131  // 低音C
#define NOTE_D3  147  // 低音D
#define NOTE_E3  165  // 低音E
#define NOTE_F3  175  // 低音F
#define NOTE_G3  196  // 低音G
#define NOTE_A3  220  // 低音A
#define NOTE_B3  247  // 低音B
#define NOTE_C4  262  // 中音C（核心音符）
#define NOTE_D4  294  // 中音D
#define NOTE_E4  330  // 中音E（核心音符）
#define NOTE_F4  349  // 中音F（核心音符）
#define NOTE_G4  392  // 中音G（核心音符）
#define NOTE_A4  440  // 中音A（核心音符）
#define NOTE_B4  494  // 中音B

// 音符结构体（频率+时长，时长单位：ms，适配120BPM）
typedef struct {
    uint16_t freq;    // 音符频率（Hz）
    uint16_t duration;// 音符时长（ms）
} music_note_t;

// 马里奥「地上BGM」乐谱（官方旋律精简版，约20秒）
extern const music_note_t mario_ground_bgm[];

// 青花瓷经典片段乐谱（"天青色等烟雨 而我在等你"）
extern const music_note_t qinghua_ci[];
extern const uint16_t qinghua_ci_size;

void play_music(const music_note_t* score, uint16_t score_size);
void boot_play_music(void);
#endif //BUZZER_MUSIC_H
