/*
 * sensorData.cpp
 *
 *  Created on: 12 июн. 2026 г.
 *      Author: user
 */

#include "sensors.h"

sensors::sensors(I2C_HandleTypeDef *hi2c) : sensor_Proc(hi2c) {

	// TODO Auto-generated constructor stub

}


void  sensors::copysave(void * dst, const void *src, size_t sz)
{
	do{
		memcpy(dst,src,sz);
	}
	while(memcmp(dst,src,sz)!=0);

}



uint32_t sensors::get_pitch	(float * pitch)
{
	uint8_t buf[2];
	copysave(&buf,this->raw_pitch_enc,2);
	uint16_t pos = (uint16_t)(buf[0] << 8) | buf[1];
	*pitch = ((float)pos * 6.28318530717959) / 4096.0f;

	return chain[1].cnt;
}

uint32_t sensors::get_yaw	(float * yaw)
{
	uint8_t buf[2];
	copysave(&buf,this->raw_yaw_enc,2);
	uint16_t pos = (uint16_t)(buf[0] << 8) | buf[1];
	*yaw = ((float)pos * 6.28318530717959) / 4096.0f;
	return chain[2].cnt;
}



void sensors::get_gyro(float * gdata, uint8_t * in_ptr)
{
	uint8_t buf[2];
	copysave( buf,in_ptr, 2);
	// Gyro (signed 16-bit, big-endian)
	int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
	//int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
	//int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);

	const float gyro_sensitivity  = 131.0f/0.01745329252f;
	gdata[0]  = (float)gx / gyro_sensitivity;
	//gdata[1]  = (float)gy / gyro_sensitivity;
	//gdata[2]  = (float)gz / gyro_sensitivity;


}


uint32_t sensors::get_gyro_gimb	(float * gdata)
{
	this->get_gyro(gdata,this->raw_imu_gyro_gimb);
	return chain[0].cnt;

}

uint32_t sensors::get_gyro_static(float * gdata)
{
	this->get_gyro(gdata,this->raw_imu_gyro_static);
	return chain[1].cnt;

}




uint32_t sensors::get_temp	(float * temp)
{
	uint8_t buf[2];
	copysave(buf,this->raw_temp,2);
	int16_t t_raw = (int16_t)((buf[0] << 8) | buf[1]);
	*temp = (float)t_raw / 326.8f + 12.0f;
	return chain[3].cnt;
}

sensors::~sensors() {
	// TODO Auto-generated destructor stub
}

void sensors::init_chain()
{

	this->overload = 0;
	int id ;

	id =0;
	this->chain[id].bufAdr 	= this->raw_imu_gyro_gimb;
	this->chain[id].busAdr 	= 0x68;
	this->chain[id].regAdr 	= 0x43; // get only 0axis
	this->chain[id].dataSize = 2;
	this->chain[id].period   = 1000/1000; // 1000hz
	this->chain[id].cnt		= 0;
	this->chain[id].i2ctype  = ALT_I2C;

	id =1;
	this->chain[id].bufAdr 	= this->raw_imu_gyro_static;
	this->chain[id].busAdr 	= 0x68;
	this->chain[id].regAdr 	= 0x43+2; // get only 1 axis
	this->chain[id].dataSize = 2;
	this->chain[id].period   = 1000/1000; // 1000hz
	this->chain[id].cnt		= 0;
	this->chain[id].i2ctype  = MAIN_I2C;

	id =2;
	this->chain[id].bufAdr 	= this->raw_pitch_enc;
	this->chain[id].busAdr 	= 0x30;
	this->chain[id].regAdr 	= 0x21;
	this->chain[id].dataSize = 4;
	this->chain[id].period   = 1000/1000; // 1000hz
	this->chain[id].cnt		= 0;
	this->chain[id].i2ctype  = ALT_I2C;

	id =3;
	this->chain[id].bufAdr 	= this->raw_yaw_enc;
	this->chain[id].busAdr 	= 0x31;
	this->chain[id].regAdr 	= 0x21;
	this->chain[id].dataSize = 4;
	this->chain[id].period   = 1000/1000; // 1000hz
	this->chain[id].cnt		= 0;
	this->chain[id].i2ctype  = ALT_I2C;

	id =4;
	this->chain[id].bufAdr 	= this->raw_temp;
	this->chain[id].busAdr 	= 0x68;
	this->chain[id].regAdr 	= 0x41;
	this->chain[id].dataSize = 2;
	this->chain[id].period   = 5000; // 0.5
	this->chain[id].cnt		= 0;
	this->chain[id].i2ctype  = ALT_I2C;

	id =5;
	this->chain[id].bufAdr 	= 0;
	this->chain[id].busAdr 	= 0;
	this->chain[id].regAdr 	= 0;
	this->chain[id].dataSize = 0;
	this->chain[id].period   = 1000/1000;
	this->chain[id].isExecute = true;
	this->chain[id].cnt		= 0;
	this->chain[id].i2ctype  = ALT_I2C;

	extern void step_motor();
	this->chain[id].funcptr = step_motor;


	this->chain[0].next = &this->chain[1];
	this->chain[1].next = &this->chain[2];
	this->chain[2].next = &this->chain[3];
	this->chain[3].next = &this->chain[4];
	this->chain[4].next = &this->chain[5];
	this->chain[5].next = nullptr;
}



