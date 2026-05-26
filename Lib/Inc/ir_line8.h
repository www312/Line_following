/**
 * @file ir_line8.h
 * @brief 八路灰度/红外模块 — USART2 TTL 协议（与 EightWayTrack app_usart 一致）
 *
 * 接线：PA2=TX→模块RX，PA3=RX←模块TX，GND 共地，115200 8N1（由 CubeMX USART2 配置）
 * 数字帧：$D,x1:0,x2:0,...,x8:0#  （x1 最左，x8 最右；1=白底，0=黑线）
 * 命令帧：$<校准>,<模拟>,<数字>#  例：$0,0,1# 仅开数字输出
 */
#ifndef IR_LINE8_H
#define IR_LINE8_H

#include <stdint.h>

#define IR_LINE8_COUNT 8U

/** 最新一帧数字量，下标 0=x1 … 7=x8；未收帧前默认为 1（白底） */
extern uint8_t IR_Line8_Value[IR_LINE8_COUNT];

/** 收到合法 $D,...# 后置 1，可用 IR_Line8_ClearFrameFlag() 清除 */
extern volatile uint8_t IR_Line8_FrameReady;

/**
 * @brief 开启数字上报并启动 USART2 单字节中断接收（需已 MX_USART2_UART_Init）
 */
void IR_Line8_Init(void);

/** 发送 $0,0,1# — 仅数字模式 */
void IR_Line8_EnableDigitalMode(void);

/** 发送 $0,1,0# — 仅模拟模式（本驱动不解析 $A 帧，需自行扩展） */
void IR_Line8_EnableAnalogMode(void);

/** 发送 $1,0,0# — 校准命令（以模块手册为准） */
void IR_Line8_Calibrate(void);

/** 手动喂入一字节（中断/轮询共用） */
void IR_Line8_FeedByte(uint8_t byte);

/** 主循环调用：读尽 RX 寄存器，中断异常时仍能更新 IR */
void IR_Line8_PollRx(void);

void IR_Line8_ClearFrameFlag(void);

/** 自检：累计收到的合法 $D,...# 帧数（上电后） */
uint32_t IR_Line8_GetFrameCount(void);
/** 自检：距上一帧的毫秒数；从未收帧则返回 0xFFFFFFFF */
uint32_t IR_Line8_GetMsSinceLastFrame(void);

#endif
