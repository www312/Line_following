/**
 * @file motion_car.h
 * @brief Differential-drive kinematics, matches official app_motor.h
 *
 * Formula (official): L = Vx + spin,  R = Vx - spin
 *   where spin = (Vz / 1000.0) * Car_APB
 */
#ifndef MOTION_CAR_H
#define MOTION_CAR_H

#include <stdint.h>

#define Car_APB (188.0f)

/**
 * @brief Configure motor parameters by chassis type.
 * Matches official Set_Motor() presets.
 *
 *  1: 520 motor
 *  2: 310 motor  (4WD 310 chassis)
 *  3: TT motor with speed encoder
 *  4: TT DC reduction motor
 *  5: L-type 520 motor
 */
void Set_Motor(int MOTOR_TYPE);

void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z);

#endif
