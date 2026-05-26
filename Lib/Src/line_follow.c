/**
 * @file line_follow.c
 * @brief Proportional steering + turn-hold with asymmetry extension
 */
#include "line_follow.h"
#include "ir_line8.h"
#include "motion_car.h"

static LineFollow_Mode s_mode = MODE_LEFT;
static int16_t s_last_vz;

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

	int16_t vz_search = (s_mode == MODE_RIGHT) ? -VZ_SEARCH : VZ_SEARCH;
	int16_t vz_turn   = (s_mode == MODE_RIGHT) ? -VZ_TURN  : VZ_TURN;

	/* junction entry: arm hold */
	if ((left + right) > 4)
		turn_until = IR_Line8_GetFrameCount() + TURN_HOLD;

	uint8_t hold_active = IR_Line8_GetFrameCount() < turn_until;

	/* still turning: extend hold while sensors asymmetric */
	if (hold_active) {
		int diff = left - right;
		if (diff < 0) diff = -diff;
		if (diff >= 2)
			turn_until = IR_Line8_GetFrameCount() + TURN_HOLD;
	}

	/* all-black: mode-directed turn */
	if ((left + right) == 8) {
		turn_until = IR_Line8_GetFrameCount() + TURN_HOLD;
		s_last_vz = vz_search;
		Motion_Car_Control(IRR_SPEED, 0, vz_search);
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
	if (hold_active) {
		if (s_mode == MODE_LEFT  && vz > vz_turn) vz = vz_turn;
		if (s_mode == MODE_RIGHT && vz < vz_turn) vz = vz_turn;
	}
	s_last_vz = vz;
	Motion_Car_Control(IRR_SPEED, 0, vz);
}

void LineFollow_SetMode(LineFollow_Mode mode)
{
	s_mode = mode;
	s_last_vz = (mode == MODE_RIGHT) ? -VZ_SEARCH : VZ_SEARCH;
}

LineFollow_Mode LineFollow_GetMode(void) { return s_mode; }
int16_t LineFollow_GetLastVz(void)       { return s_last_vz; }
