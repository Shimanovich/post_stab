#pragma once

#include "main.h"
#include "common/base_classes/Sensor.h"   // <-- важно: путь относительно вашего проекта
#include "math.h"

/**
 * @file
 * @brief данные о положении энкодера принимает цепочка DMA, этот драйвер нужен для того, чтобы извлечь последнее принятое значение и работать с ним.
 * он же подменяет реализует метод getAngle() и getVelocity() для FOC
 */
class dataDma : public Sensor {



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

	        // Приводим к диапазону (-π, π]
	            angle = _normalizeAngle(angle);     // ← лучше всего вызвать эту функцию
	            return angle;
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
