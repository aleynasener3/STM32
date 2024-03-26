/*
 * mpu6050.c
 *
 *  Created on: Aug 10, 2023
 *      Author: Aleyna
 */

#include <mpu6050.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


float acc_x1 = 0;
float acc_y1 = 0;
float acc_z1 = 0;
float gyro_x1 = 0;
float gyro_y1 = 0;
float gyro_z1 = 0;


int mpu6050_init(t_mpu6050Handler *gyro, I2C_HandleTypeDef *i2c,uint16_t devAdress, uint16_t size,uint32_t timeout, uint16_t MemAddSize ){

	gyro->i2c = i2c;
	gyro->devAddress = devAdress;
	gyro->Size = size;
	gyro->Timeout = timeout;
	gyro->MemAddSize = MemAddSize;

	int answer = 0;

	uint8_t tempdata;
	uint16_t MemAddress;



	//GYRO

	tempdata = (0x01 << 3);//500
	MemAddress = REG_CONFIG_GYRO ;

	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(gyro->i2c,( gyro->devAddress <<1) +0 , 1, gyro->Timeout);

	if(ret == HAL_OK){
		answer = 100;
	}else{
		answer = -100;
	}


	HAL_StatusTypeDef ret2 =HAL_I2C_Mem_Write(gyro->i2c,( gyro->devAddress <<1) +0, MemAddress, gyro->MemAddSize,&tempdata, gyro->Size, gyro->Timeout);

	if(ret2 == HAL_OK){
		answer = answer + 100;
	}else{
		answer = answer -100;
	}



	//accelerometer

	tempdata = (0x01 << 3);//4g
	MemAddress = REG_CONFIG_ACC ;

	HAL_StatusTypeDef ret3 = HAL_I2C_IsDeviceReady(gyro->i2c,( gyro->devAddress <<1) +0 , 1, gyro->Timeout);

	if(ret3 == HAL_OK){
		answer = answer + 100;
	}else{
		answer = answer -100;
	}


	HAL_StatusTypeDef ret4 =HAL_I2C_Mem_Write(gyro->i2c,( gyro->devAddress <<1) +0, MemAddress, gyro->MemAddSize,&tempdata, gyro->Size, gyro->Timeout);

	if(ret4 == HAL_OK){
		answer = answer + 100;
	}else{
		answer = answer -100;
	}

	//exiting sleep mode


	tempdata = 0;
	MemAddress = REG_USR_CTRL ;


	HAL_StatusTypeDef ret5 =HAL_I2C_Mem_Write(gyro->i2c,( gyro->devAddress <<1) +0, MemAddress, gyro->MemAddSize,&tempdata, gyro->Size, gyro->Timeout);

	if(ret5 == HAL_OK){
		answer = answer + 100;
	}else{
		answer = answer -100;
	}


	return answer;


	tempdata = 0x07;
	MemAddress = 25 ;


	HAL_I2C_Mem_Write(gyro->i2c,( gyro->devAddress <<1) +0, MemAddress, gyro->MemAddSize,&tempdata, gyro->Size, gyro->Timeout);



}

void first_read(t_mpu6050Handler *gyro){
	uint8_t data[2];
	uint8_t data1[2];

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,REG_DATA ,gyro->MemAddSize, data, 1, gyro->Timeout);

	float x_acc_ = ((int16_t)data[0] << 8)+ data[1];
	acc_x1 = x_acc_/8192.0;
	HAL_Delay(500);

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,61 ,gyro->MemAddSize, data, 1, gyro->Timeout);
	float y_acc_ = ((int16_t)data[0] << 8)+ data[1];
	acc_y1 = y_acc_/8192.0;
	HAL_Delay(500);

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,63 ,gyro->MemAddSize, data, 1, gyro->Timeout);
	float z_acc_ = ((int16_t)data[0] << 8)+ data[1];
	acc_z1 = z_acc_/8192.0;

	HAL_Delay(500);

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,67 ,gyro->MemAddSize, data1, 1, gyro->Timeout);
	gyro_x1= ((int16_t)data1[0] << 8)+ data1[1];

	HAL_Delay(500);
	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,69 ,gyro->MemAddSize, data1, 1, gyro->Timeout);
	gyro_y1= ((int16_t)data1[0] << 8)+ data1[2];

	HAL_Delay(500);
		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,71 ,gyro->MemAddSize, data1, 1, gyro->Timeout);
	gyro_z1= ((int16_t)data1[0] << 8)+ data1[1];

}

void mpu6050_read(t_mpu6050Handler *gyro, Results* res){
	uint8_t data_1[2];
	uint8_t data_2[2];

		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,REG_DATA ,gyro->MemAddSize, data_1, 1, gyro->Timeout);

		res->x_acc = ((int16_t)data_1[0] << 8)+ data_1[1];
		res->x_acc = res->x_acc/8192.0;

		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,61 ,gyro->MemAddSize, data_1, 1, gyro->Timeout);
		res->y_acc = ((int16_t)data_1[0] << 8)+ data_1[1];
		res->y_acc = res->y_acc/8192.0;

		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,63 ,gyro->MemAddSize, data_1, 1, gyro->Timeout);
		res->z_acc = ((int16_t)data_1[0] << 8)+ data_1[1];
		res->z_acc = res->z_acc/8192.0;


		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,67 ,gyro->MemAddSize, data_2, 1, gyro->Timeout);
		res->x_gyro= ((int16_t)data_2[0] << 8)+ data_2[1];
		res->x_gyro = ((res->x_gyro-gyro_x1)*0.01526 );

		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,69 ,gyro->MemAddSize, data_2, 1, gyro->Timeout);
		res->y_gyro= ((int16_t)data_2[0] << 8)+ data_2[2];
		res->y_gyro = ((res->y_gyro-gyro_y1)*0.01526 );

		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,71 ,gyro->MemAddSize, data_2, 1, gyro->Timeout);
		res->z_gyro= ((int16_t)data_2[0] << 8)+ data_2[1];
		res->z_gyro = ((res->z_gyro-gyro_z1)*0.01526 );

		HAL_Delay(500);

}




void mpu6050_read_gyro(t_mpu6050Handler *gyro, Results* res){
	uint8_t data[6];



		HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,67 ,gyro->MemAddSize, data, 1, gyro->Timeout);
		res->x_gyro= ((int16_t)data[0] << 8)+ data[1];
		res->x_gyro = ((res->x_gyro)*0.01526 );

		res->y_gyro= ((int16_t)data[2] << 8)+ data[3];
		res->y_gyro = ((res->y_gyro)*0.01526 );


		res->z_gyro= ((int16_t)data[4] << 8)+ data[5];
		res->z_gyro = ((res->z_gyro)*0.01526 );

		//HAL_Delay(500);

}




float gyro_x(Results* res){
	return res->x_gyro;
}

float gyro_y(Results* res){
	return res->y_gyro;
}

float gyro_z(Results* res){
	return res->z_gyro;
}


//ACELEROMETER-----------------------------------------------------------------------

float mpu6050_read_acc_x(t_mpu6050Handler *gyro){

	uint8_t data[2];
	float x_acc = 0;

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,REG_DATA ,gyro->MemAddSize, data, 2, gyro->Timeout);
	x_acc = (int16_t)((data[0] << 8) | data[1]);
	x_acc = x_acc/8192.0-acc_x1;
	return x_acc;
}

float mpu6050_read_acc_y(t_mpu6050Handler *gyro){

	uint8_t data4[2];
	float y_acc = 0;

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,60 ,gyro->MemAddSize, data4, 2, gyro->Timeout);
	y_acc = (int16_t)((data4[0] << 8) | data4[1]);
	y_acc = y_acc/8192.0-acc_y1;
	return y_acc;
}

float mpu6050_read_acc_z(t_mpu6050Handler *gyro){

	uint8_t data5[2];
	float z_acc = 0;

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,61 ,gyro->MemAddSize, data5, 2, gyro->Timeout);
	z_acc = (int16_t)((data5[0] << 8) | data5[1]);
	z_acc = z_acc/8192.0-acc_z1;
	return z_acc;
}





//TEMPERATURE-----------------------------------------------------------------------


float mpu6050_read_temp(t_mpu6050Handler *gyro){

	uint8_t data[2];
	float temp;

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,65 ,gyro->MemAddSize, data, 2, gyro->Timeout);
	temp =  (int16_t)((data[0] << 8) | data[1]);
	float tempC = ((float)temp / 340.0) + 36.53;
	return tempC;
}




//GYRO-----------------------------------------------------------------------


float mpu6050_read_gyro_x(t_mpu6050Handler *gyro){

	uint8_t data1[2];
	float x_gyro = 0;

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,67 ,gyro->MemAddSize, data1, 2, gyro->Timeout);
	x_gyro = (int16_t)((data1[0] << 8) | data1[1]);
	x_gyro = ((x_gyro-gyro_x1)*0.01526 );
	return x_gyro;
}

float mpu6050_read_gyro_y(t_mpu6050Handler *gyro){

	uint8_t data2[2];
	float y_gyro = 0;

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,69 ,gyro->MemAddSize, data2, 2, gyro->Timeout);
	y_gyro = (int16_t)((data2[0] << 8) | data2[1]);
    y_gyro = ((y_gyro-gyro_y1)*0.01526);  // Convert to degrees
	return y_gyro;
}

float mpu6050_read_gyro_z(t_mpu6050Handler *gyro){

	uint8_t data3[2];
	float z_gyro = 0;

	HAL_I2C_Mem_Read(gyro->i2c,( gyro->devAddress <<1) +1,71 ,gyro->MemAddSize, data3, 2, gyro->Timeout);
	z_gyro = (int16_t)((data3[0] << 8) | data3[1]);
    z_gyro = ((z_gyro-gyro_z1)*0.01526);
	return z_gyro;
}








