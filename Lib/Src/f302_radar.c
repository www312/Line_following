#include "f302_radar.h"

static uint8_t s_buf[F302_FRAME_LEN];
static uint8_t s_idx;

void f302_parser_reset(void) {
  s_idx = 0;
}

uint8_t f302_feed_byte(uint8_t b, f302_target_t *out) {
  if (s_idx == 0) {
    if (b == 0xAAu || b == 0x55u) {
      s_buf[0] = b;
      s_idx = 1;
    }
    return 0;
  }
  if (s_idx == 1) {
    if ((s_buf[0] == 0xAAu && b == 0x55u) || (s_buf[0] == 0x55u && b == 0xAAu)) {
      s_buf[1] = b;
      s_idx = 2;
    } else if (b == 0xAAu || b == 0x55u) {
      s_buf[0] = b;
      s_idx = 1;
    } else {
      s_idx = 0;
    }
    return 0;
  }
  s_buf[s_idx++] = b;
  if (s_idx < F302_FRAME_LEN) {
    return 0;
  }
  s_idx = 0;

  uint16_t cmd = (uint16_t)s_buf[2] | ((uint16_t)s_buf[3] << 8);
  uint16_t len = (uint16_t)s_buf[4] | ((uint16_t)s_buf[5] << 8);
  if (cmd != F302_CMD_REPORT || len != F302_PAYLOAD_LEN) {
    return 0;
  }
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 12u; i++) {
    sum += s_buf[i];
  }
  if ((uint8_t)(sum & 0xFFu) != s_buf[12]) {
    return 0;
  }

  if (out) {
    out->range_0p01m = (uint16_t)s_buf[6] | ((uint16_t)s_buf[7] << 8);
    out->angle_0p01deg = -(int16_t)((uint16_t)s_buf[8] | ((uint16_t)s_buf[9] << 8));
    out->have_target = (out->range_0p01m != 0u) ? 1u : 0u;
  }
  return 1u;
}
