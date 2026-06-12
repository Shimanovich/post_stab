/*
 * sensorProc.cpp
 *
 *  Created on: 11 июн. 2026 г.
 *      Author: user
 */

#include "sensorProc.h"


sensor_Proc::sensor_Proc() {
	// TODO Auto-generated constructor stub
	this->active = &this->chain[0];
	this->isActive = false;
	this->isStoped = true;

	for (int i=0;i<NUM_AREAS;i++ )
	{
		chain[i].regArd =0;
		chain[i].bufAdr = nullptr;
		chain[i].busAdr = 0;
		chain[i].next   = nullptr;
		chain[i].dataSize= 0;
	}

}

sensor_Proc::~sensor_Proc() {
	// TODO Auto-generated destructor stub
}

void sensor_Proc::Start(I2C_HandleTypeDef *hi2c)
{
	this->hi2c = hi2c;
	if (active) {
		this->isActive = true;
		this->isStoped = false;

		this->singleEvent();
	}
}


void sensor_Proc::singleEvent()
{
	if (!this->isActive)
	{
		this->isStoped = true;
	}
	else
	{
		active->cnt++;
		if ((active->bufAdr)&&(active->dataSize))
		{
			HAL_I2C_Mem_Read_DMA(this->hi2c, (active->busAdr << 1), active->regArd,I2C_MEMADD_SIZE_8BIT, (uint8_t*)active->bufAdr, active->dataSize);
		}
		if (active->next)
		{
					active = active->next;
		}
		else
		{
			this->isActive  = false;
		}
	}
}
