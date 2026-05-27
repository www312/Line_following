/**
 * @file line_follow.h
 * @brief Proportional line correction — small steer while moving forward
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

#define IRR_SPEED     150   /* forward speed when line visible */
#define VZ_PER_ERR    475   /* Vz gain per probe offset (small steer) */
#define VZ_SEARCH   -3000   /* Vz for lost-line pivot search (negative = left) */
#define TURN_HOLD     30   /* sensor frames to hold turn after junction */
#define VZ_TURN      -2000   /* Vz floor during turn hold (negative = left) */

typedef enum { MODE_LEFT = 0, MODE_RIGHT = 1 } LineFollow_Mode;

void LineWalking(void);
void LineFollow_SetMode(LineFollow_Mode mode);
LineFollow_Mode LineFollow_GetMode(void);
int16_t LineFollow_GetLastVz(void);

#endif
