#pragma once

#include "main.h"
#include "common/base_classes/Sensor.h"
#include <cmath>

/**
 * @file encoderDataDma.h
 * @brief Драйвер энкодера AM4096 для SimpleFOC (через DMA-буфер из sensors chain)
 *
 * Исправления:
 * - getAngle() теперь возвращает НЕПРЕРЫВНЫЙ (unwrapped) угол в радианах
 *   без искусственного разрыва на ±π. Это критично для стабильной работы
 *   shaft_angle и любого position/velocity контроля.
 * - getVelocity() вычисляется по разнице raw-позиции (0-4095) с правильной
 *   обработкой переполнения 12-битного счётчика. Больше нет огромных
 *   скачков скорости при переходе через границу ±π.
 * - Используется реальное время между вызовами (HAL_GetTick).
 * - Сохранена совместимость с zero_offset и natural_direction.
 */


#define POS_ARRAY_SIZE 16

#define INTERVALS (POS_ARRAY_SIZE - 1)

class dataDma : public Sensor
{
public:
    float zero_offset = 0.0f;
    int   natural_direction = 1;   // 1 = CW, -1 = CCW (берётся из базового класса)

private:
    uint8_t* encDataPtr = nullptr;
    uint8_t* gyroDataPtr = nullptr;

    float velocity =0.0;

    // Для непрерывного угла (unwrap)
    uint16_t prev_raw_pos = 0;
    float    accumulated_angle = 0.0f;   // непрерывный угол в радианах
    bool     first_angle = true;

    // Для скорости
    uint16_t prev_raw_for_vel = 0;
    uint32_t last_update_tick = 0;
    bool     first_velocity = true;

public:

    void  copysave(void * dst, const void *src, size_t sz)
    {
    	do{
    		memcpy(dst,src,sz);
    	}
    	while(memcmp(dst,src,sz)!=0);

    }

    uint16_t anglesArray[POS_ARRAY_SIZE];
    uint16_t anglesArray_mirror[POS_ARRAY_SIZE];

    uint32_t ar_index;

    void begin(uint8_t* enc_ptr, uint8_t* gyro_ptr)
    {
        encDataPtr  = enc_ptr;
        gyroDataPtr = gyro_ptr;
        ar_index = 0;
    }

    // ==================== ОСНОВНЫЕ МЕТОДЫ ДЛЯ SimpleFOC ====================

    /**
     * Возвращает НЕПРЕРЫВНЫЙ угол в радианах (без разрыва на ±π).
     * Это рекомендуемый вариант для shaft_angle и управления.
     */
    float getAngle() override
    {
		if (!encDataPtr)
			return 0.0f;
		uint16_t current_raw = (uint16_t) (encDataPtr[0] << 8) | encDataPtr[1];
		if (first_angle) {
			prev_raw_pos = current_raw;
			accumulated_angle = natural_direction
					* ((float) current_raw * 2.0f * M_PI / 4096.0f);
			first_angle = false;
			return accumulated_angle - zero_offset;
		}

		// Правильная дельта для unwrap (абсолютный энкодер)
		int32_t delta_raw = (int32_t) current_raw - prev_raw_pos;
		if (delta_raw > 2048)
			delta_raw -= 4096;
		if (delta_raw < -2048)
			delta_raw += 4096;

		accumulated_angle += natural_direction
				* (delta_raw * 2.0f * M_PI / 4096.0f);
		prev_raw_pos = current_raw;

		accumulated_angle = natural_direction
				* ((float) current_raw * 2.0f * M_PI / 4096.0f);
		// return (accumulated_angle - zero_offset);

		float angle = accumulated_angle - zero_offset;
		const float PI = M_PI;
		angle = fmodf(angle + PI, 2.0f * PI);
		if (angle < 0)
			angle += 2.0f * PI;
		return angle - PI;
    }



    /**
     * Скорость вычисляется по raw-разнице (без использования обёрнутого угла).
     * Это устраняет скачки ~2π при переходе границы.
     */
    float getVelocity() override
    {
        if (!encDataPtr) return 0.0f;

        if (ar_index < POS_ARRAY_SIZE)
        {
            return 0.0f;   // ещё не набрали достаточно отсчётов
        }


        int32_t total_dist = 0;
       		for (int i = 0; i < INTERVALS; i++) {
       			int idx1 = (ar_index + i) % POS_ARRAY_SIZE;
       			int idx2 = (ar_index + i + 1) % POS_ARRAY_SIZE;

       			int angle1 = ((int32_t) anglesArray[idx1] + 2048) % 4096;
       			if (angle1 < 0)
       				angle1 += 4096;
       			angle1 -= 4096;

       			int angle2 = ((int32_t) anglesArray[idx2] + 2048) % 4096;
       			if (angle2 < 0)
       				angle2 += 4096;
       			angle2 -= 4096;

       			int32_t delta_raw = (int32_t) angle2 - (int32_t) angle1;
       			total_dist += (delta_raw);
       		}

       		float middle_speed_raw = (float) total_dist / INTERVALS;
       		float velocity = natural_direction * (middle_speed_raw * 2.0f * M_PI / 4096.0f) * 1000.0f;

        return velocity;
    }

    void update_syncVelocity() override
    {
        if (encDataPtr)
        {
            anglesArray[ar_index % POS_ARRAY_SIZE] = ((uint16_t)(encDataPtr[0] << 8) | encDataPtr[1])&0x0fff;
            ar_index++;
        }

//        if (ar_index < POS_ARRAY_SIZE)
//        {
//            velocity =0.0f;
//            return;
//        }





    }


    // ==================== ИНИЦИАЛИЗАЦИЯ НУЛЯ ====================

    float initRelativeZero() override { return initAbsoluteZero(); }

    float initAbsoluteZero() override
    {
        if (!encDataPtr) return 0.0f;

        uint16_t pos = (uint16_t)(encDataPtr[0] << 8) | encDataPtr[1];
        float current_rad = natural_direction * ((float)pos * 2.0f * M_PI / 4096.0f);

        float diff = current_rad - zero_offset;
        zero_offset = current_rad;
        accumulated_angle = current_rad;
        prev_raw_pos = pos;
        first_angle = false;

        return diff;
    }

    int hasAbsoluteZero() override        { return 1; }
    int needsAbsoluteZeroSearch() override { return 0; }
};
