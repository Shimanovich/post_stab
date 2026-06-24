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

class dataDma : public Sensor
{
public:
    float zero_offset = 0.0f;
    int   natural_direction = 1;   // 1 = CW, -1 = CCW (берётся из базового класса)

private:
    uint8_t* encDataPtr = nullptr;
    uint8_t* gyroDataPtr = nullptr;

    // Для непрерывного угла (unwrap)
    uint16_t prev_raw_pos = 0;
    float    accumulated_angle = 0.0f;   // непрерывный угол в радианах
    bool     first_angle = true;

    // Для скорости
    uint16_t prev_raw_for_vel = 0;
    uint32_t last_update_tick = 0;
    bool     first_velocity = true;

public:
    void begin(uint8_t* enc_ptr, uint8_t* gyro_ptr)
    {
        encDataPtr  = enc_ptr;
        gyroDataPtr = gyro_ptr;
    }

    // ==================== ОСНОВНЫЕ МЕТОДЫ ДЛЯ SimpleFOC ====================

    /**
     * Возвращает НЕПРЕРЫВНЫЙ угол в радианах (без разрыва на ±π).
     * Это рекомендуемый вариант для shaft_angle и управления.
     */
    float getAngle() override
    {
        if (!encDataPtr) return 0.0f;

        uint16_t current_raw = (uint16_t)(encDataPtr[0] << 8) | encDataPtr[1];

        if (first_angle)
        {
            prev_raw_pos     = current_raw;
            accumulated_angle = natural_direction * ((float)current_raw * 2.0f * M_PI / 4096.0f);
            first_angle = false;
            return accumulated_angle - zero_offset;
        }

        // Вычисляем дельту с учётом переполнения 12-битного энкодера
        int32_t delta_raw = (int32_t)current_raw - prev_raw_pos;

        if (delta_raw >  2048) delta_raw -= 4096;
        if (delta_raw < -2048) delta_raw += 4096;

        // Накапливаем непрерывный угол
        accumulated_angle += natural_direction * (delta_raw * 2.0f * M_PI / 4096.0f);
        prev_raw_pos = current_raw;

        return accumulated_angle - zero_offset;
    }

    /**
     * Возвращает угол, обёрнутый в [-π, π] (если кому-то очень нужно для внешних нужд).
     * В большинстве случаев лучше использовать getAngle() — непрерывный.
     */
    float getWrappedAngle()
    {
        float angle = getAngle();                    // берём непрерывный
        const float PI = M_PI;

        angle = fmodf(angle + PI, 2.0f * PI);
        if (angle < 0) angle += 2.0f * PI;
        return angle - PI;
    }

    /**
     * Скорость вычисляется по raw-разнице (без использования обёрнутого угла).
     * Это устраняет скачки ~2π при переходе границы.
     */
    float getVelocity() override
    {
        if (!encDataPtr) return 0.0f;

        uint16_t current_raw = (uint16_t)(encDataPtr[0] << 8) | encDataPtr[1];
        uint32_t now = HAL_GetTick();

        if (first_velocity)
        {
            prev_raw_for_vel = current_raw;
            last_update_tick = now;
            first_velocity = false;
            return 0.0f;
        }

        uint32_t dt_ms = now - last_update_tick;
        if (dt_ms == 0) dt_ms = 1;                    // защита от деления на ноль

        float dt = dt_ms / 1000.0f;                   // секунды

        // Дельта raw с правильной обработкой переполнения
        int32_t delta_raw = (int32_t)current_raw - prev_raw_for_vel;
        if (delta_raw >  2048) delta_raw -= 4096;
        if (delta_raw < -2048) delta_raw += 4096;

        float delta_angle = natural_direction * (delta_raw * 2.0f * M_PI / 4096.0f);
        float velocity = delta_angle / dt;

        prev_raw_for_vel = current_raw;
        last_update_tick = now;

        return velocity;
    }

    // ==================== ИНИЦИАЛИЗАЦИЯ НУЛЯ ====================

    float initRelativeZero() override
    {
        return initAbsoluteZero();
    }

    float initAbsoluteZero() override
    {
        if (!encDataPtr) return 0.0f;

        uint16_t pos = (uint16_t)(encDataPtr[0] << 8) | encDataPtr[1];
        float current_rad = (pos * 2.0f * M_PI) / 4096.0f;

        float diff = current_rad - zero_offset;
        zero_offset = current_rad;

        // Сбрасываем накопленный угол, чтобы новый ноль стал началом
        accumulated_angle = current_rad;
        prev_raw_pos = pos;

        return diff;
    }

    int hasAbsoluteZero() override        { return 1; }
    int needsAbsoluteZeroSearch() override { return 0; }
};
