/**
 * @file motion_car.c
 * @brief Matches official app_motor.c kinematics exactly.
 */
#include "motion_car.h"
#include "motor_driver.h"
#include "delay_dwt.h"

void Set_Motor(int MOTOR_TYPE)
{
	if (MOTOR_TYPE == 1) {
		Set_motor_type(1);
		delay_ms(100);
		Set_Pluse_Phase(30);
		delay_ms(100);
		Set_Pluse_line(11);
		delay_ms(100);
		Set_Wheel_dis(67.00f);
		delay_ms(100);
		Set_motor_deadzone(1600);
		delay_ms(100);
	} else if (MOTOR_TYPE == 2) {
		Set_motor_type(2);
		delay_ms(100);
		Set_Pluse_Phase(20);
		delay_ms(100);
		Set_Pluse_line(13);
		delay_ms(100);
		Set_Wheel_dis(48.00f);
		delay_ms(100);
		Set_motor_deadzone(1900);
		delay_ms(100);
	} else if (MOTOR_TYPE == 3) {
		Set_motor_type(3);
		delay_ms(100);
		Set_Pluse_Phase(45);
		delay_ms(100);
		Set_Pluse_line(13);
		delay_ms(100);
		Set_Wheel_dis(68.00f);
		delay_ms(100);
		Set_motor_deadzone(1250);
		delay_ms(100);
	} else if (MOTOR_TYPE == 4) {
		Set_motor_type(4);
		delay_ms(100);
		Set_Pluse_Phase(48);
		delay_ms(100);
		Set_motor_deadzone(1000);
		delay_ms(100);
	} else if (MOTOR_TYPE == 5) {
		Set_motor_type(1);
		delay_ms(100);
		Set_Pluse_Phase(40);
		delay_ms(100);
		Set_Pluse_line(11);
		delay_ms(100);
		Set_Wheel_dis(67.00f);
		delay_ms(100);
		Set_motor_deadzone(1600);
		delay_ms(100);
	}
}

void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
	int speed_L1, speed_L2, speed_R1, speed_R2;
	float speed_fb, speed_spin;

	(void)V_y;

	if (V_x == 0 && V_y == 0 && V_z == 0) {
		control_speed(0, 0, 0, 0);
		return;
	}

	speed_fb   = (float)V_x;
	speed_spin = ((float)V_z / 1000.0f) * Car_APB;

	speed_L1 = (int)(speed_fb + speed_spin);
	speed_L2 = (int)(speed_fb + speed_spin);
	speed_R1 = (int)(speed_fb - speed_spin);
	speed_R2 = (int)(speed_fb - speed_spin);

	if (speed_L1 >  1000) speed_L1 =  1000;
	if (speed_L1 < -1000) speed_L1 = -1000;
	if (speed_L2 >  1000) speed_L2 =  1000;
	if (speed_L2 < -1000) speed_L2 = -1000;
	if (speed_R1 >  1000) speed_R1 =  1000;
	if (speed_R1 < -1000) speed_R1 = -1000;
	if (speed_R2 >  1000) speed_R2 =  1000;
	if (speed_R2 < -1000) speed_R2 = -1000;

	control_speed((int16_t)speed_L1, (int16_t)speed_L2,
		      (int16_t)speed_R1, (int16_t)speed_R2);
}
