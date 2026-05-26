/**
 * @file motor_soft_i2c.h
 * @brief Software I2C on PB10(SCL)/PB11(SDA) for motor driver module
 */
#ifndef MOTOR_SOFT_I2C_H
#define MOTOR_SOFT_I2C_H

#include <stdint.h>

void MotorSoftI2C_Init(void);

int  IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
int  IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
