#pragma once

#include "main.h"

/**
 * @file AM4096.h
 * @brief Полный драйвер энкодера AM4096 для STM32 (HAL I2C)
 *        Версия с отдельной инициализацией шины и адреса
 */
class AM4096 {
public:
    struct OutputData {
        uint16_t rpos;
        uint16_t apos;
        bool     rposValid;
        bool     aposValid;
        bool     magnetTooClose;
        bool     magnetTooFar;
    };

    AM4096() = default;   // Конструктор по умолчанию

    /**
     * Инициализация драйвера (задание шины и адреса)
     */
    void begin(I2C_HandleTypeDef* hi2c, uint8_t address = 0x00) {
        _hi2c = hi2c;
        _devAddr = address;
        _isInitialized = true;
    }

    // ==================== БАЗОВЫЕ ОПЕРАЦИИ ====================

    HAL_StatusTypeDef readRegister(uint8_t reg, uint16_t* data) {
        if (!_isInitialized || _hi2c == nullptr) return HAL_ERROR;

        uint8_t buf[2] = {0};
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
            _hi2c, _devAddr << 1, reg,
            I2C_MEMADD_SIZE_8BIT, buf, 2, TIMEOUT_MS
        );
        if (status == HAL_OK) {
            *data = (uint16_t)(buf[0] << 8) | buf[1];
        }
        return status;
    }

    HAL_StatusTypeDef writeRegister(uint8_t reg, uint16_t data) {
        if (!_isInitialized || _hi2c == nullptr) return HAL_ERROR;

        uint8_t buf[2];
        buf[0] = (data >> 8) & 0xFF;
        buf[1] = data & 0xFF;

        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
            _hi2c, _devAddr << 1, reg,
            I2C_MEMADD_SIZE_8BIT, buf, 2, TIMEOUT_MS
        );

        if (status == HAL_OK && reg <= 0x1F) {
            HAL_Delay(20);
        }
        return status;
    }

    // ==================== ЧТЕНИЕ ПОЗИЦИИ ====================

    HAL_StatusTypeDef getRelativePosition(uint16_t* position) {
        uint16_t raw = 0;
        HAL_StatusTypeDef st = readRegister(32, &raw);
        if (st == HAL_OK) {
            *position = raw & 0x0FFF;
        }
        return st;
    }

    HAL_StatusTypeDef getAbsolutePosition(uint16_t* position) {
        uint16_t raw = 0;
        HAL_StatusTypeDef st = readRegister(33, &raw);
        if (st == HAL_OK) {
            *position = raw & 0x0FFF;
        }
        return st;
    }

    HAL_StatusTypeDef readOutputData(OutputData* out) {
        uint16_t regs[4] = {0};

        for (uint8_t i = 0; i < 4; ++i) {
            HAL_StatusTypeDef st = readRegister(32 + i, &regs[i]);
            if (st != HAL_OK) return st;
        }

        out->rpos           = regs[0] & 0x0FFF;
        out->rposValid      = (regs[0] & 0x8000) != 0;
        out->apos           = regs[1] & 0x0FFF;
        out->aposValid      = (regs[1] & 0x8000) != 0;
        out->magnetTooClose = (regs[2] & (1 << 13)) != 0;
        out->magnetTooFar   = (regs[2] & (1 << 14)) != 0;

        return HAL_OK;
    }

    HAL_StatusTypeDef getAngleDegrees(float* angle, bool useAbsolute = true) {
        uint16_t pos = 0;
        HAL_StatusTypeDef st = useAbsolute ? getAbsolutePosition(&pos)
                                           : getRelativePosition(&pos);
        if (st == HAL_OK) {
            *angle = (pos * 360.0f) / 4096.0f;
        }
        return st;
    }

    // ==================== УСТАНОВКА НУЛЕВОЙ ПОЗИЦИИ ====================

    HAL_StatusTypeDef setZeroPosition(uint16_t zin, bool permanent = false) {
        if (zin > 4095) zin = 4095;

        uint8_t reg = permanent ? 1 : 49;

        uint16_t current = 0;
        HAL_StatusTypeDef st = readRegister(reg, &current);
        if (st != HAL_OK) return st;

        current = (current & 0xF000) | (zin & 0x0FFF);
        return writeRegister(reg, current);
    }

    HAL_StatusTypeDef calibrateZero(bool permanent = false) {
        uint16_t currentPos = 0;
        HAL_StatusTypeDef st = getAbsolutePosition(&currentPos);
        if (st != HAL_OK) return st;

        return setZeroPosition(currentPos, permanent);
    }

    // ==================== СЛУЖЕБНЫЕ ====================

    bool isInitialized() const { return _isInitialized; }

    HAL_StatusTypeDef isConnected() {
        if (!_isInitialized) return HAL_ERROR;
        uint16_t dummy = 0;
        return readRegister(33, &dummy);
    }

private:
    I2C_HandleTypeDef* _hi2c = nullptr;
    uint8_t            _devAddr = 0;
    bool               _isInitialized = false;
    static constexpr uint32_t TIMEOUT_MS = 100;
};
