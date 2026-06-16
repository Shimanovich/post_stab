/*
 * sensorProc.h
 *
 *  Created on: 11 июн. 2026 г.
 *      Author: user
 */
#pragma once
#include "main.h"
#include "stdio.h"




#define NUM_AREAS 5

class sensor_Proc {

public:
	typedef struct i2cChain_t
	{
		uint8_t 		busAdr;
		uint8_t 		regAdr;
		void * 			bufAdr;
		uint8_t     	dataSize;
		i2cChain_t * 	next;
		uint32_t        cnt;
		uint32_t        period;
		uint32_t        tick;
		bool 			isExecute;
		void  			(*funcptr)(void);

	}i2cChain_t;

public:
	i2cChain_t chain[NUM_AREAS] ;

	void Start();
	void singleEvent();
	sensor_Proc(I2C_HandleTypeDef *hi2c);
	~sensor_Proc();
private:

	bool isActive;
	bool isStoped;

	uint32_t ticVal;
	i2cChain_t * active;
	I2C_HandleTypeDef *hi2c;




};


