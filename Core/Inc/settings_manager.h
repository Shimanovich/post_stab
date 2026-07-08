#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include "stm32f3xx_hal.h"
#include <cstddef>
#include <cstring>
#include <cstdint>
#include "stdio.h"


#define STORANGE_FLASH_PAGE_SIZE         2048U

#define SETTINGS_FLASH_ADDR     (0x08000000UL + (256U * 1024U) - STORANGE_FLASH_PAGE_SIZE)

struct Settings_t {
    uint32_t    magic;          // 0xDEADBEEF
    uint32_t    version;

    float azMotor_voltage_limit;
    uint32_t azMotor_pole_pairs;
    float azMotor_velocity_limit;
    float azMotor_Pid_velocity_P;
    float azMotor_Pid_velocity_I;
    float azMotor_Pid_velocity_D;
    float azMotor_LPF_velocity_TF;
    float azMotor_electric_angle;

    float elMotor_voltage_limit;
    float elMotor_velocity_limit;
    float elMotor_Pid_velocity_P;
    float elMotor_Pid_velocity_I;
    float elMotor_Pid_velocity_D;
    float elMotor_LPF_velocity_TF;
    float elMotor_electric_angle;




    uint32_t    crc32;          // контрольная сумма (CRC-32)
    //uint8_t     reserved[FLASH_PAGE_SIZE - 52];
};

class SettingsManager {
public:
    static constexpr uint32_t MAGIC = 0xDEADBEEF;


    SettingsManager();

    void PrintAllData();
    bool loadFromFlash();
    bool saveToFlash();


    Settings_t&       get()       { return m_settings; }
    const Settings_t& get() const { return m_settings; }

private:
    Settings_t m_settings;

    void setDefaults();
    uint32_t computeCRC() const;
    bool     verifyCRC() const;

    static uint32_t crc32(const uint8_t* data, size_t length);

    HAL_StatusTypeDef eraseFlashPage(uint32_t address);
};

#endif
