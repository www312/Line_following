/**
 * @file motor_driver.h
 * @brief 4-channel motor driver I2C register map & API
 *
 * Matches official bsp_motor_iic.h register layout.
 * I2C slave address: 0x26
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#define Motor_model_ADDR 0x26U

typedef enum {
	MOTOR_TYPE_REG         = 0x01,
	MOTOR_DeadZONE_REG     = 0x02,
	MOTOR_PluseLine_REG    = 0x03,
	MOTOR_PlusePhase_REG   = 0x04,
	WHEEL_DIA_REG          = 0x05,
	SPEED_Control_REG      = 0x06,
	PWM_Control_REG        = 0x07,

	READ_TEN_M1Enconer_REG = 0x10,
	READ_TEN_M2Enconer_REG = 0x11,
	READ_TEN_M3Enconer_REG = 0x12,
	READ_TEN_M4Enconer_REG = 0x13,

	READ_ALLHigh_M1_REG    = 0x20,
	READ_ALLLOW_M1_REG     = 0x21,
	READ_ALLHigh_M2_REG    = 0x22,
	READ_ALLLOW_M2_REG     = 0x23,
	READ_ALLHigh_M3_REG    = 0x24,
	READ_ALLLOW_M3_REG     = 0x25,
	READ_ALLHigh_M4_REG    = 0x26,
	READ_ALLLOW_M4_REG     = 0x27,
	Motor_IIC_REG_MAX
} Motor_IIC_ADDR_t;

extern int Encoder_Offset[4];
extern int Encoder_Now[4];

void IIC_Motor_Init(void);

void Set_motor_type(uint8_t data);
void Set_motor_deadzone(uint16_t data);
void Set_Pluse_line(uint16_t data);
void Set_Pluse_Phase(uint16_t data);
void Set_Wheel_dis(float data);

void control_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void control_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

void Read_10_Enconder(void);
void Read_ALL_Enconder(void);

#endif
