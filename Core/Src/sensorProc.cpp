/*
 * sensorProc.cpp
 *
 *  Created on: 11 июн. 2026 г.
 *      Author: user
 */

#include "sensorProc.h"


sensor_Proc::sensor_Proc(I2C_HandleTypeDef *hi2c) {
	// TODO Auto-generated constructor stub
	this->hi2c = hi2c;
	this->active = &this->chain[0];
	this->isActive = false;
	this->isStoped = true;

	for (int i=0;i<NUM_AREAS;i++ )
	{
		chain[i].regAdr 	=0;
		chain[i].bufAdr 	= nullptr;
		chain[i].busAdr 	= 0;
		chain[i].next   	= nullptr;
		chain[i].dataSize	= 0;
		chain[i].tick 		= 0;
		chain[i].period		= 0;
		chain[i].isExecute  = false;

	}

}

sensor_Proc::~sensor_Proc() {
	// TODO Auto-generated destructor stub
}

void sensor_Proc::Start()
{
		this->active = &this->chain[0];
		this->isActive = true;
		this->isStoped = false;
		this->singleEvent();

}


void sensor_Proc::singleEvent()
{

	if ((this->active!=nullptr)&&(this->isActive))
	{
		active->tick++;

		if (active->tick >= active->period) {
			active->tick = 0;
			active->cnt++;

			if (active->isExecute)
			{
				active->funcptr();
			}
			else
			if ((active->bufAdr) && (active->dataSize)) {
				HAL_I2C_Mem_Read_DMA(this->hi2c, (active->busAdr << 1),
						active->regAdr, I2C_MEMADD_SIZE_8BIT,
						(uint8_t*) active->bufAdr, active->dataSize);
			}

			active = active->next;

			if (active == nullptr) {
				this->isActive = false;
			}
		}
		else
		{
			active = active->next;
			if (active == nullptr) {
			this->isActive = false;
			}
			this->singleEvent();
		}

	}
	else
	{
		this->isStoped = true;

	}
}
