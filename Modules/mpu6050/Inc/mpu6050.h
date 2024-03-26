/*
 * mpu6050.h
 *
 *  Created on: Aug 10, 2023
 *      Author: Aleyna
 */

#ifndef MODULES_MPU6050_MPU6050_H_
#define MODULES_MPU6050_MPU6050_H_

#include "stm32f4xx.h"
#include <stdbool.h>

#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 9
#define FS_GYRO_2000 10

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 9
#define FS_ACC_16G 10

#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107
#define REG_DATA 59

#define dt 0.001

extern float acc_x1;
extern float acc_y1;
extern float acc_z1;
extern float gyro_x1;
extern float gyro_y1;
extern float gyro_z1;


typedef struct {
	I2C_HandleTypeDef * i2c;//&hi2c1
	uint16_t devAddress; //0x68
	uint8_t *pData;
	uint16_t Size;//1
	uint32_t Timeout;//100
	uint16_t MemAddress;//27 REG_CONFIG_GYRO
	uint16_t MemAddSize; //1


}t_mpu6050Handler;


typedef struct {
	float x_acc;
	float y_acc;
	float z_acc;

	float temp;

	float x_gyro;
	float y_gyro;
	float z_gyro;
}Results;



int mpu6050_init(t_mpu6050Handler *gyro, I2C_HandleTypeDef *i2c,uint16_t devAdress, uint16_t size,uint32_t timeout, uint16_t MemAddSize  );

void first_read(t_mpu6050Handler *gyro);

void mpu6050_read(t_mpu6050Handler *gyro, Results* res);

void mpu6050_read_gyro(t_mpu6050Handler *gyro, Results* res);

float gyro_x(Results* res);

float gyro_y(Results* res);

float gyro_z(Results* res);

float mpu6050_read_acc_x(t_mpu6050Handler *gyro);

float mpu6050_read_acc_y(t_mpu6050Handler *gyro);

float mpu6050_read_acc_z(t_mpu6050Handler *gyro);

float mpu6050_read_temp(t_mpu6050Handler *gyro);

float mpu6050_read_gyro_x(t_mpu6050Handler *gyro);

float mpu6050_read_gyro_y(t_mpu6050Handler *gyro);

float mpu6050_read_gyro_z(t_mpu6050Handler *gyro);



/*
 *
 * temp_data pData 0b00001000 ifade yerine

FS_GYRO_250 = 0
FS_GYRO_500 = 8
FS_GYRO_1000 = 9
FS_GYRO_2000 = 10

MemAddress
REG_CONFIG_GYRO 27
REG_CONFIG_ACC 28
REG_USR_CTRL 107
REG_DATA 59

*/
#endif /* MODULES_MPU6050_MPU6050_H_ */
