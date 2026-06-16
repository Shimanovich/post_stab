#pragma once



#include "sensorProc.h"
#include "cstring"

class sensors : public sensor_Proc{

private:


	void  copysave(void * dst, const void *src, size_t sz);


public:


	uint8_t   raw_pitch_enc[4];
	uint8_t   raw_yaw_enc[4];
	uint8_t   raw_imu_gyro[6];
	uint8_t   raw_temp[2];

public:

	void init_chain();

	uint32_t get_temp	(float * temp);

	uint32_t get_pitch	(float * pitch);


	uint32_t get_yaw	(float * yaw);


	uint32_t get_gyro	(float * gdata);





	sensors(I2C_HandleTypeDef *hi2c);
	virtual ~sensors();
};


