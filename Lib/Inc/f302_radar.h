/* 60GParking 毫米波雷达 F302 串口协议 V1.1 — 仅解析 0xA206 主动上报 */
#ifndef F302_RADAR_H
#define F302_RADAR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define F302_HDR0 0xAAu
#define F302_HDR1 0x55u
#define F302_CMD_REPORT 0xA206u
#define F302_PAYLOAD_LEN 6u
#define F302_FRAME_LEN 14u

typedef struct {
  uint16_t range_0p01m;   /* 0~300 → 0~3.00 m */
  int16_t angle_0p01deg;  /* -6000~6000 → -60.00°~+60.00° */
  uint8_t have_target;    /* range_0p01m != 0 */
} f302_target_t;

uint8_t f302_feed_byte(uint8_t b, f302_target_t *out);
void f302_parser_reset(void);

#ifdef __cplusplus
}
#endif
#endif
