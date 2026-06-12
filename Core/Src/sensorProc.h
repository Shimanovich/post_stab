/*
 * sensorProc.h
 *
 *  Created on: 11 июн. 2026 г.
 *      Author: user
 */
#pragma once
#include "main.h"
#include "stdio.h"




#define NUM_AREAS 3

class sensor_Proc {

public:
	typedef struct i2cChain_t
	{
		uint8_t 		busAdr;
		uint8_t 		regArd;
		void * 		bufAdr;
		uint8_t     	dataSize;
		i2cChain_t * 	next;
		uint32_t        cnt;
	}i2cChain_t;

public:
	i2cChain_t chain[NUM_AREAS] ;

	void Start(I2C_HandleTypeDef *hi2c);
	void singleEvent();
	sensor_Proc();
	~sensor_Proc();
private:

	bool isActive;
	bool isStoped;

	i2cChain_t * active;
	I2C_HandleTypeDef *hi2c;




};


