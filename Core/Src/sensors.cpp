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




uint32_t sensors::get_gyro	(float * gdata)
{
	uint8_t buf[6];
	copysave( buf,this->raw_imu_gyro, 6);
	// Gyro (signed 16-bit, big-endian)
	int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
	int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
	int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);

	const float gyro_sensitivity  = 65.5f/0.01745329252f;
	gdata[0]  = (float)gx / gyro_sensitivity;
	gdata[1]  = (float)gy / gyro_sensitivity;
	gdata[2]  = (float)gz / gyro_sensitivity;
	return chain[0].cnt;

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


	this->chain[0].bufAdr 	= this->raw_imu_gyro;
	this->chain[0].busAdr 	= 0x68;
	this->chain[0].regAdr 	= 0x43;
	this->chain[0].dataSize = 6;
	this->chain[0].period   = 1000/1000; // 1000hz
	this->chain[0].cnt		= 0;

	this->chain[1].bufAdr 	= this->raw_pitch_enc;
	this->chain[1].busAdr 	= 0x30;
	this->chain[1].regAdr 	= 0x20;
	this->chain[1].dataSize = 4;
	this->chain[1].period   = 1000/1000; // 100hz
	this->chain[1].cnt		= 0;


	this->chain[2].bufAdr 	= this->raw_yaw_enc;
	this->chain[2].busAdr 	= 0x31;
	this->chain[2].regAdr 	= 0x20;
	this->chain[2].dataSize = 4;
	this->chain[2].period   = 1000/1000; // 100hz
	this->chain[2].cnt		= 0;

	this->chain[3].bufAdr 	= this->raw_temp;
	this->chain[3].busAdr 	= 0x68;
	this->chain[3].regAdr 	= 0x41;
	this->chain[3].dataSize = 2;
	this->chain[3].period   = 5000; // 0.5
	this->chain[3].cnt		= 0;

	this->chain[4].bufAdr 	= 0;
	this->chain[4].busAdr 	= 0;
	this->chain[4].regAdr 	= 0;
	this->chain[4].dataSize = 0;
	this->chain[4].period   = 1; // 0.5
	this->chain[4].isExecute = true;
	this->chain[4].cnt		= 0;

	extern void step_motor();
	this->chain[4].funcptr = step_motor;

	this->chain[0].next = &this->chain[1];
	this->chain[1].next = &this->chain[2];
	this->chain[2].next = &this->chain[3];
	this->chain[3].next = &this->chain[4];
	this->chain[4].next = nullptr;
}



