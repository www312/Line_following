/**
 * @file motor_soft_i2c.c
 * @brief HAL software I2C, pins from main.h (MOTOR_SCL=PB10, MOTOR_SDA=PB11)
 *
 * Logic matches official IOI2C.c; uses HAL GPIO with open-drain + pullup
 * which is electrically correct for I2C.
 */
#include "motor_soft_i2c.h"
#include "delay_dwt.h"
#include "main.h"

static void SDA_Mode_Output(void)
{
	GPIO_InitTypeDef g = {0};
	g.Pin = MOTOR_SDA_Pin;
	g.Mode = GPIO_MODE_OUTPUT_OD;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(MOTOR_SDA_GPIO_Port, &g);
}

static void SDA_Mode_Input(void)
{
	GPIO_InitTypeDef g = {0};
	g.Pin = MOTOR_SDA_Pin;
	g.Mode = GPIO_MODE_INPUT;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(MOTOR_SDA_GPIO_Port, &g);
}

#define IIC_SCL_H() HAL_GPIO_WritePin(MOTOR_SCL_GPIO_Port, MOTOR_SCL_Pin, GPIO_PIN_SET)
#define IIC_SCL_L() HAL_GPIO_WritePin(MOTOR_SCL_GPIO_Port, MOTOR_SCL_Pin, GPIO_PIN_RESET)

#define IIC_SDA_WRITE_H()                       \
	do {                                        \
		SDA_Mode_Output();                      \
		HAL_GPIO_WritePin(MOTOR_SDA_GPIO_Port,  \
				  MOTOR_SDA_Pin, GPIO_PIN_SET); \
	} while (0)

#define IIC_SDA_WRITE_L()                        \
	do {                                         \
		SDA_Mode_Output();                       \
		HAL_GPIO_WritePin(MOTOR_SDA_GPIO_Port,   \
				  MOTOR_SDA_Pin, GPIO_PIN_RESET);\
	} while (0)

#define READ_SDA() \
	(HAL_GPIO_ReadPin(MOTOR_SDA_GPIO_Port, MOTOR_SDA_Pin) == GPIO_PIN_SET)

/* ---------- official IOI2C.c equivalents ---------- */

int IIC_Start(void)
{
	SDA_Mode_Output();
	IIC_SDA_WRITE_H();
	if (!READ_SDA()) return 0;
	IIC_SCL_H();
	delay_us(1);
	IIC_SDA_WRITE_L();
	if (READ_SDA()) return 0;
	delay_us(1);
	IIC_SCL_L();
	return 1;
}

void IIC_Stop(void)
{
	SDA_Mode_Output();
	IIC_SCL_L();
	IIC_SDA_WRITE_L();
	delay_us(1);
	IIC_SCL_H();
	IIC_SDA_WRITE_H();
	delay_us(1);
}

int IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	SDA_Mode_Input();
	delay_us(1);
	IIC_SCL_H();
	delay_us(1);
	while (READ_SDA()) {
		ucErrTime++;
		if (ucErrTime > 50U) {
			IIC_Stop();
			return 0;
		}
		delay_us(1);
	}
	IIC_SCL_L();
	return 1;
}

void IIC_Ack(void)
{
	IIC_SCL_L();
	SDA_Mode_Output();
	IIC_SDA_WRITE_L();
	delay_us(1);
	IIC_SCL_H();
	delay_us(1);
	IIC_SCL_L();
}

void IIC_NAck(void)
{
	IIC_SCL_L();
	SDA_Mode_Output();
	IIC_SDA_WRITE_H();
	delay_us(1);
	IIC_SCL_H();
	delay_us(1);
	IIC_SCL_L();
}

void IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
	SDA_Mode_Output();
	IIC_SCL_L();
	for (t = 0; t < 8U; t++) {
		if ((txd & 0x80U) != 0U)
			IIC_SDA_WRITE_H();
		else
			IIC_SDA_WRITE_L();
		txd <<= 1;
		delay_us(1);
		IIC_SCL_H();
		delay_us(1);
		IIC_SCL_L();
		delay_us(1);
	}
}

uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_Mode_Input();
	for (i = 0; i < 8U; i++) {
		IIC_SCL_L();
		delay_us(2);
		IIC_SCL_H();
		receive <<= 1;
		if (READ_SDA()) receive++;
		delay_us(2);
	}
	if (ack != 0U)
		IIC_Ack();
	else
		IIC_NAck();
	return receive;
}

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	if (!IIC_Start())  return 1;
	IIC_Send_Byte((uint8_t)(addr << 1));
	if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	for (i = 0; i < (int)len; i++) {
		IIC_Send_Byte(data[i]);
		if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
	}
	IIC_Stop();
	return 0;
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if (!IIC_Start())  return 1;
	IIC_Send_Byte((uint8_t)(addr << 1));
	if (!IIC_Wait_Ack()) { IIC_Stop(); return 1; }
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte((uint8_t)((addr << 1) + 1U));
	IIC_Wait_Ack();
	while (len != 0U) {
		if (len == 1U)
			*buf = IIC_Read_Byte(0);
		else
			*buf = IIC_Read_Byte(1);
		buf++;
		len--;
	}
	IIC_Stop();
	return 0;
}

void MotorSoftI2C_Init(void)
{
	GPIO_InitTypeDef g = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	g.Pin = MOTOR_SCL_Pin | MOTOR_SDA_Pin;
	g.Mode = GPIO_MODE_OUTPUT_OD;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(MOTOR_SCL_GPIO_Port, &g);
	HAL_GPIO_WritePin(MOTOR_SCL_GPIO_Port,
			  MOTOR_SCL_Pin | MOTOR_SDA_Pin, GPIO_PIN_SET);
}
