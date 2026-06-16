#pragma once

#include "main.h"
#include "common/base_classes/Sensor.h"   // <-- важно: путь относительно вашего проекта
#include "math.h"

/**
 * @file AM4096.h
 * @brief Драйвер энкодера AM4096 (адаптирован под интерфейс Sensor SimpleFOC)
 */
class emulator : public Sensor {



public:

	// Для Sensor
	    float    zero_offset = 0.0f;
	    float    prev_angle = 0.0f;
	    uint32_t prev_timestamp = 0;
	    bool     first_velocity = true;

	    uint8_t * encDataPtr;
	    uint8_t * gyroDataPtr;



	    float getAngle() override {
	        uint16_t pos = (uint16_t)(encDataPtr[0] << 8) | encDataPtr[1];
	        const float PI = 3.14159265358979323846f;

	        float angle = natural_direction * (((float)pos * 2.0f * PI) / 4096.0f - zero_offset);

	        // Нормализация в [-π, π]
	        angle = fmodf(angle + PI, 2.0f * PI);
	        if (angle < 0) angle += 2.0f * PI;
	        return angle - PI;
	    }

	    float getVelocity() override {
	        float current_angle = getAngle();


	        if (first_velocity) {
	            prev_angle = current_angle;
	            first_velocity = false;
	            return 0.0f;
	        }

	        float dt = (1.0) / 1000.0f;


	        float velocity = (current_angle - prev_angle) / dt;

	        prev_angle = current_angle;

	        return velocity;


//	    		int16_t gx = (int16_t)((gyroDataPtr[0] << 8) | gyroDataPtr[1]);
//	    		int16_t gy = (int16_t)((gyroDataPtr[2] << 8) | gyroDataPtr[3]);
//	    		int16_t gz = (int16_t)((gyroDataPtr[4] << 8) | gyroDataPtr[5]);
//
//	    		const float gyro_sensitivity  = 65.5f;
//	    		float gdata[3];
//	    		gdata[0]  = (float)gx / gyro_sensitivity;
//	    		gdata[1]  = (float)gy / gyro_sensitivity;
//	    		gdata[2]  = (float)gz / gyro_sensitivity;
//
//	    		return -gdata[0];


	    }

	    float initRelativeZero() override {
	        return initAbsoluteZero();   // для абсолютного энкодера — одно и то же
	    }

	    float initAbsoluteZero() override {
			int16_t pos = (uint16_t)(encDataPtr[0] << 8) | encDataPtr[1];// 32 - Absolute pos

	        float current_rad = (pos * 2.0f * 3.14159265358979323846f) / 4096.0f;
	        float diff = current_rad - zero_offset;
	        zero_offset = current_rad;

	        // Раскомментируйте, если хотите записывать ноль в EEPROM энкодера
	        // setZeroPosition(pos, false);

	        return diff;
	    }

	    int hasAbsoluteZero() override { return 1; }           // есть абсолютная позиция
	    int needsAbsoluteZeroSearch() override { return 0; }   // магнитный — поиск не нужен

public:

	void begin(uint8_t * ptr,uint8_t * ptr2)
	{
		encDataPtr = ptr;
		gyroDataPtr = ptr2;
	}
};
