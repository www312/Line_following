/**
 * @file line_follow.h
 * @brief Proportional line correction — small steer while moving forward
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

#define IRR_SPEED       150
#define VZ_PER_ERR      550
#define VZ_SEARCH_LEFT  -3000   /* lost-line pivot: left turn */
#define VZ_SEARCH_RIGHT  3000   /* lost-line pivot: right turn */
#define TURN_HOLD        30   /* sensor frames to hold turn after junction */
#define VZ_TURN_LEFT    -2600   /* hold clamp: left turn strength */
#define VZ_TURN_RIGHT    4000   /* hold clamp: right turn strength */

typedef enum { MODE_LEFT = 0, MODE_RIGHT = 1 } LineFollow_Mode;

void LineWalking(void);
void LineFollow_SetMode(LineFollow_Mode mode);
LineFollow_Mode LineFollow_GetMode(void);
int16_t LineFollow_GetLastVz(void);
uint32_t LineFollow_GetLastTurnTick(void);

#endif
