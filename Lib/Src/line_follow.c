/**
 * @file line_follow.c
 * @brief Proportional steering + turn-hold with asymmetry extension
 */
#include "line_follow.h"
#include "delay_dwt.h"
#include "ir_line8.h"
#include "motion_car.h"
#include "stm32f1xx_hal.h"

static LineFollow_Mode s_mode = MODE_LEFT;
static int16_t s_last_vz;
static uint32_t s_last_turn_tick;

void LineWalking(void)
{
	uint8_t x1 = IR_Line8_Value[0], x2 = IR_Line8_Value[1],
		x3 = IR_Line8_Value[2], x4 = IR_Line8_Value[3],
		x5 = IR_Line8_Value[4], x6 = IR_Line8_Value[5],
		x7 = IR_Line8_Value[6], x8 = IR_Line8_Value[7];

	static uint32_t turn_until;

	int left  = (x1 == 0) + (x2 == 0) + (x3 == 0) + (x4 == 0);
	int right = (x5 == 0) + (x6 == 0) + (x7 == 0) + (x8 == 0);
	uint8_t all_white = (left + right) == 0;

	int16_t vz_search = (s_mode == MODE_RIGHT) ? VZ_SEARCH_RIGHT : VZ_SEARCH_LEFT;
	int16_t vz_turn   = (s_mode == MODE_RIGHT) ? VZ_TURN_RIGHT   : VZ_TURN_LEFT;

	/* junction entry: arm hold only when line bias matches mode */
	if ((left + right) >= 4) {
		if (s_mode == MODE_LEFT  && left > right)
			turn_until = IR_Line8_GetFrameCount() + TURN_HOLD;
		if (s_mode == MODE_RIGHT && right > left)
			turn_until = IR_Line8_GetFrameCount() + TURN_HOLD;
		if (right > left || left > right)
			s_last_turn_tick = HAL_GetTick();
	}

	uint8_t hold_active = IR_Line8_GetFrameCount() < turn_until;

	/* still turning: extend hold only when asymmetry matches mode */
	if (hold_active) {
		int diff = left - right;
		if (s_mode == MODE_LEFT  && diff >= 2)
			turn_until = IR_Line8_GetFrameCount() + TURN_HOLD;
		if (s_mode == MODE_RIGHT && -diff >= 2)
			turn_until = IR_Line8_GetFrameCount() + TURN_HOLD;
	}

	/* symmetric junction: go straight 100ms, then check if cross or T */
	if ((left + right) >= 4 && left == right) {
		Motion_Car_Control(IRR_SPEED, 0, 0);
		delay_ms(100);
		if (IR_Line8_Value[3] == 0 && IR_Line8_Value[4] == 0) {
			/* cross: middle still on line → keep straight */
			s_last_vz = 0;
			Motion_Car_Control(IRR_SPEED, 0, 0);
		} else {
			s_last_vz = vz_search;
			Motion_Car_Control(IRR_SPEED, 0, vz_search);
		}
		return;
	}

	/* centered & hold expired: straight */
	if (x4 == 0 && x5 == 0 && (left + right) <= 4 && !hold_active) {
		s_last_vz = 0;
		Motion_Car_Control(IRR_SPEED, 0, 0);
		return;
	}

	/* lost: pivot in last known direction */
	if (all_white) {
		if (s_last_vz == 0) s_last_vz = vz_search;
		Motion_Car_Control(0, 0, s_last_vz);
		return;
	}

	/* deviating / turning: proportional with direction floor during hold */
	int16_t vz = (int16_t)((right - left) * VZ_PER_ERR);
	if (vz >  2000) vz =  2000;
	if (vz < -1500) vz = -1500;
	if (hold_active) {
		if (s_mode == MODE_LEFT  && vz > vz_turn) vz = vz_turn;
		if (s_mode == MODE_RIGHT && vz < vz_turn) vz = vz_turn;
	}
	if (s_last_vz == 0 || (vz > 0) == (s_last_vz > 0))
		s_last_vz = vz;
	Motion_Car_Control(IRR_SPEED, 0, vz);
}

void LineFollow_SetMode(LineFollow_Mode mode)
{
	s_mode = mode;
	s_last_vz = (mode == MODE_RIGHT) ? VZ_SEARCH_RIGHT : VZ_SEARCH_LEFT;
}

LineFollow_Mode LineFollow_GetMode(void) { return s_mode; }
int16_t LineFollow_GetLastVz(void)       { return s_last_vz; }
uint32_t LineFollow_GetLastTurnTick(void) { return s_last_turn_tick; }
