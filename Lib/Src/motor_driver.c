/**
 * @file motor_driver.c
 * @brief Motor I2C register read/write, matches official bsp_motor_iic.c
 */
#include "motor_driver.h"
#include "motor_soft_i2c.h"
#include <string.h>

int Encoder_Offset[4];
int Encoder_Now[4];

static void float_to_bytes(float f, uint8_t *bytes)
{
	memcpy(bytes, &f, sizeof(float));
}

void IIC_Motor_Init(void)
{
	MotorSoftI2C_Init();
}

void Set_motor_type(uint8_t data)
{
	uint8_t buf[2] = { data, 0 };
	i2cWrite(Motor_model_ADDR, MOTOR_TYPE_REG, 2, buf);
}

void Set_motor_deadzone(uint16_t data)
{
	uint8_t buf[2];
	buf[0] = (uint8_t)((data >> 8) & 0xFFU);
	buf[1] = (uint8_t)(data & 0xFFU);
	i2cWrite(Motor_model_ADDR, MOTOR_DeadZONE_REG, 2, buf);
}

void Set_Pluse_line(uint16_t data)
{
	uint8_t buf[2];
	buf[0] = (uint8_t)((data >> 8) & 0xFFU);
	buf[1] = (uint8_t)(data & 0xFFU);
	i2cWrite(Motor_model_ADDR, MOTOR_PluseLine_REG, 2, buf);
}

void Set_Pluse_Phase(uint16_t data)
{
	uint8_t buf[2];
	buf[0] = (uint8_t)((data >> 8) & 0xFFU);
	buf[1] = (uint8_t)(data & 0xFFU);
	i2cWrite(Motor_model_ADDR, MOTOR_PlusePhase_REG, 2, buf);
}

void Set_Wheel_dis(float data)
{
	uint8_t bytes[4];
	float_to_bytes(data, bytes);
	i2cWrite(Motor_model_ADDR, WHEEL_DIA_REG, 4, bytes);
}

void control_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	uint8_t speed[8];
	speed[0] = (uint8_t)((m1 >> 8) & 0xFF);
	speed[1] = (uint8_t)(m1 & 0xFF);
	speed[2] = (uint8_t)((m2 >> 8) & 0xFF);
	speed[3] = (uint8_t)(m2 & 0xFF);
	speed[4] = (uint8_t)((m3 >> 8) & 0xFF);
	speed[5] = (uint8_t)(m3 & 0xFF);
	speed[6] = (uint8_t)((m4 >> 8) & 0xFF);
	speed[7] = (uint8_t)(m4 & 0xFF);
	i2cWrite(Motor_model_ADDR, SPEED_Control_REG, 8, speed);
}

void control_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	uint8_t pwm[8];
	pwm[0] = (uint8_t)((m1 >> 8) & 0xFF);
	pwm[1] = (uint8_t)(m1 & 0xFF);
	pwm[2] = (uint8_t)((m2 >> 8) & 0xFF);
	pwm[3] = (uint8_t)(m2 & 0xFF);
	pwm[4] = (uint8_t)((m3 >> 8) & 0xFF);
	pwm[5] = (uint8_t)(m3 & 0xFF);
	pwm[6] = (uint8_t)((m4 >> 8) & 0xFF);
	pwm[7] = (uint8_t)(m4 & 0xFF);
	i2cWrite(Motor_model_ADDR, PWM_Control_REG, 8, pwm);
}

void Read_10_Enconder(void)
{
	uint8_t buf[2];

	i2cRead(Motor_model_ADDR, READ_TEN_M1Enconer_REG, 2, buf);
	Encoder_Offset[0] = (int)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

	i2cRead(Motor_model_ADDR, READ_TEN_M2Enconer_REG, 2, buf);
	Encoder_Offset[1] = (int)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

	i2cRead(Motor_model_ADDR, READ_TEN_M3Enconer_REG, 2, buf);
	Encoder_Offset[2] = (int)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

	i2cRead(Motor_model_ADDR, READ_TEN_M4Enconer_REG, 2, buf);
	Encoder_Offset[3] = (int)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

void Read_ALL_Enconder(void)
{
	uint8_t buf[2];
	uint8_t buf2[2];

	i2cRead(Motor_model_ADDR, READ_ALLHigh_M1_REG, 2, buf);
	i2cRead(Motor_model_ADDR, READ_ALLLOW_M1_REG, 2, buf2);
	Encoder_Now[0] = (int)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
			       ((uint32_t)buf2[0] << 8) | (uint32_t)buf2[1]);

	i2cRead(Motor_model_ADDR, READ_ALLHigh_M2_REG, 2, buf);
	i2cRead(Motor_model_ADDR, READ_ALLLOW_M2_REG, 2, buf2);
	Encoder_Now[1] = (int)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
			       ((uint32_t)buf2[0] << 8) | (uint32_t)buf2[1]);

	i2cRead(Motor_model_ADDR, READ_ALLHigh_M3_REG, 2, buf);
	i2cRead(Motor_model_ADDR, READ_ALLLOW_M3_REG, 2, buf2);
	Encoder_Now[2] = (int)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
			       ((uint32_t)buf2[0] << 8) | (uint32_t)buf2[1]);

	i2cRead(Motor_model_ADDR, READ_ALLHigh_M4_REG, 2, buf);
	i2cRead(Motor_model_ADDR, READ_ALLLOW_M4_REG, 2, buf2);
	Encoder_Now[3] = (int)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
			       ((uint32_t)buf2[0] << 8) | (uint32_t)buf2[1]);
}
