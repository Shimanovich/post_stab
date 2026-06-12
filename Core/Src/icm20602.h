#ifndef ICM20602_H
#define ICM20602_H

#include "main.h"          // В CubeIDE подключает HAL и все периферийные заголовки
#include <cstdint>

//#define ICM20602_DEFAULT_ADDR    0x68  // Или 0x69, если AD0 = 1
#define ICM20602_WHO_AM_I        0x75
#define ICM20602_WHO_AM_I_VALUE  0x12  // Из даташита ICM-20602

// Регистры (из репозитория и даташита)
#define ICM20602_PWR_MGMT_1      0x6B
#define ICM20602_PWR_MGMT_2      0x6C
#define ICM20602_USER_CTRL       0x6A
#define ICM20602_ACCEL_CONFIG    0x1C
#define ICM20602_ACCEL_CONFIG_2  0x1D
#define ICM20602_GYRO_CONFIG     0x1B
#define ICM20602_CONFIG          0x1A  // Для DLPF гиро
#define ICM20602_SMPLRT_DIV      0x19
#define ICM20602_FIFO_EN         0x23
#define ICM20602_ACCEL_XOUT_H    0x3B
#define ICM20602_TEMP_OUT_H      0x41
#define ICM20602_GYRO_XOUT_H     0x43

// Коды ошибок
#define ICM20602_OK              0
#define ICM20602_ERR_I2C         -1
#define ICM20602_ERR_ID          -2
#define ICM20602_ERR_CONFIG      -3

// Масштабы по умолчанию
#define ICM20602_ACCEL_FS_2G     0x00  // ±2g
#define ICM20602_GYRO_FS_500DPS  0x01  // ±500 dps
#define ICM20602_ACCEL_DLPF_BYP  0x09  // Bypass 1046 Hz
#define ICM20602_GYRO_DLPF_20HZ  0x05  // 20 Hz

/**
 * @brief Класс драйвера ICM-20602 для STM32 с HAL (CubeIDE)
 *
 * Адрес I2C и экземпляр hi2c задаются через метод begin(),
 * а не через конструктор (как вы просили).
 *
 * Использование:
 *   extern I2C_HandleTypeDef hi2c1;
 *   ICM20602 imu;
 *   if (imu.begin(&hi2c1, 0x68) == ICM20602_OK) { ... }
 */
class ICM20602 {
public:
    /**
     * @brief Конструктор по умолчанию (без параметров)
     */
    ICM20602() = default;

    /**
     * @brief Задаёт интерфейс I2C + адрес и выполняет полную инициализацию датчика
     * @param hi2c  указатель на I2C_HandleTypeDef (например &hi2c1 из CubeMX)
     * @param addr  I2C-адрес датчика (0x68 по умолчанию, 0x69 если AD0=1)
     * @return ICM20602_OK (0) при успехе, иначе отрицательный код ошибки
     */
    int8_t begin(I2C_HandleTypeDef* hi2c, uint8_t addr = 0x68);

    /**
     * @brief Чтение данных с датчика
     * @param gyro  [out] массив [X, Y, Z] угловой скорости в °/с
     * @param accel [out] массив [X, Y, Z] ускорения в g
     * @param temp  [out] температура в °C
     * @return ICM20602_OK (0) при успехе, иначе отрицательный код ошибки
     */
    int8_t read(float gyro[3], float accel[3], float* temp);

private:
    I2C_HandleTypeDef* _hi2c = nullptr;
    uint8_t _addr = 0;

    int8_t write_reg(uint8_t reg, uint8_t value);
    int8_t read_regs(uint8_t reg, uint8_t* buffer, uint8_t len);
};

// ==================== РЕАЛИЗАЦИЯ МЕТОДОВ ====================

int8_t ICM20602::begin(I2C_HandleTypeDef* hi2c, uint8_t addr) {
    if (hi2c == nullptr) {
        return ICM20602_ERR_CONFIG;
    }

    _hi2c = hi2c;
    _addr = addr;

    uint8_t chip_id;
    int8_t ret;

    // Проверка ID чипа
    ret = read_regs(ICM20602_WHO_AM_I, &chip_id, 1);
    if (ret != 0 || chip_id != ICM20602_WHO_AM_I_VALUE) {
        return ICM20602_ERR_ID;
    }

    // Сброс чипа
    ret = write_reg(ICM20602_PWR_MGMT_1, 0x80);
    if (ret != 0) return ret;
    HAL_Delay(100);

    // Установка источника тактирования PLL
    ret = write_reg(ICM20602_PWR_MGMT_1, 0x01);
    if (ret != 0) return ret;

    // Перевод акселерометра и гироскопа в standby
    ret = write_reg(ICM20602_PWR_MGMT_2, 0x3F);
    if (ret != 0) return ret;

    // Отключение FIFO
    ret = write_reg(ICM20602_USER_CTRL, 0x00);
    if (ret != 0) return ret;

    // Конфигурация акселерометра: ±2g + bypass DLPF
    ret = write_reg(ICM20602_ACCEL_CONFIG_2, ICM20602_ACCEL_DLPF_BYP);
    if (ret != 0) return ret;
    ret = write_reg(ICM20602_ACCEL_CONFIG, ICM20602_ACCEL_FS_2G << 3);
    if (ret != 0) return ret;

    // Конфигурация гироскопа: 1KHZ
    ret = write_reg(ICM20602_CONFIG, 0x01);
    if (ret != 0) return ret;

    ret = write_reg(ICM20602_GYRO_CONFIG, (ICM20602_GYRO_FS_500DPS << 3) | 0x00);// FCHOICE_B = 00 (DLPF включён)
    if (ret != 0) return ret;

    // Делитель частоты выборки (~100 Гц)
    ret = write_reg(ICM20602_SMPLRT_DIV, 0x00);
    if (ret != 0) return ret;

    // Включение акселерометра и гироскопа
    ret = write_reg(ICM20602_PWR_MGMT_2, 0x00);
    if (ret != 0) return ret;

    HAL_Delay(50);

    return ICM20602_OK;
}

int8_t ICM20602::read(float gyro[3], float accel[3], float* temp) {
    if (_hi2c == nullptr) {
        return ICM20602_ERR_CONFIG;
    }

    uint8_t buffer[14];
    int8_t ret;

    ret = read_regs(ICM20602_ACCEL_XOUT_H, buffer, 14);
    if (ret != 0) return ret;

    // Accel (signed 16-bit, big-endian)
    int16_t ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t az = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Температура (raw)
    int16_t t_raw = (int16_t)((buffer[6] << 8) | buffer[7]);

    // Gyro (signed 16-bit, big-endian)
    int16_t gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Чувствительность по умолчанию (из настроек begin())
    const float accel_sensitivity = 16384.0f;   // LSB/g для ±2g
    const float gyro_sensitivity  = 65.5f;      // LSB/(°/s) для ±500 dps

    accel[0] = (float)ax / accel_sensitivity;
    accel[1] = (float)ay / accel_sensitivity;
    accel[2] = (float)az / accel_sensitivity;

    gyro[0]  = (float)gx / gyro_sensitivity;
    gyro[1]  = (float)gy / gyro_sensitivity;
    gyro[2]  = (float)gz / gyro_sensitivity;

    // Температура — формула взята без изменений из оригинального Arduino-кода
    *temp = (float)t_raw / 326.8f + 12.0f;

    return ICM20602_OK;
}

int8_t ICM20602::write_reg(uint8_t reg, uint8_t value) {
    if (_hi2c == nullptr) {
        return ICM20602_ERR_CONFIG;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        _hi2c,
        (uint16_t)(_addr << 1),     // 7-битный адрес → формат HAL
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY
    );
    return (status == HAL_OK) ? ICM20602_OK : ICM20602_ERR_I2C;
}

int8_t ICM20602::read_regs(uint8_t reg, uint8_t* buffer, uint8_t len) {
    if (_hi2c == nullptr) {
        return ICM20602_ERR_CONFIG;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        _hi2c,
        (uint16_t)(_addr << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        len,
        HAL_MAX_DELAY
    );
    return (status == HAL_OK) ? ICM20602_OK : ICM20602_ERR_I2C;
}

#endif // ICM20602_H
