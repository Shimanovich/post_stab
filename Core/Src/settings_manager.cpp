#include "settings_manager.h"

SettingsManager::SettingsManager() {
}

bool SettingsManager::loadFromFlash() {
    memcpy(&m_settings, (void*)SETTINGS_FLASH_ADDR, sizeof(Settings_t));

    if (m_settings.magic != MAGIC || !verifyCRC()) {
        setDefaults();
        return false;
    }
    return true;
}

bool SettingsManager::saveToFlash() {
    m_settings.magic = MAGIC;
    m_settings.crc32 = computeCRC();   // считаем CRC перед записью

    if (eraseFlashPage(SETTINGS_FLASH_ADDR) != HAL_OK) {
        return false;
    }

    HAL_FLASH_Unlock();

    bool success = true;
    const uint32_t* src = reinterpret_cast<const uint32_t*>(&m_settings);
    uint32_t addr = SETTINGS_FLASH_ADDR;

    for (size_t i = 0; i < ((sizeof(Settings_t)+4) / 4); ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) {
            success = false;
            break;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return success;
}

void SettingsManager::setDefaults() {
    memset(&m_settings, 0, sizeof(m_settings));
    m_settings.magic      				= MAGIC;
    m_settings.version     				= 2;

    m_settings.azMotor_voltage_limit 	= 8.0;
    m_settings.azMotor_velocity_limit	= 40.0;
    m_settings.azMotor_Pid_velocity_P	= 0.0;
    m_settings.azMotor_Pid_velocity_I	= 0.0;
    m_settings.azMotor_Pid_velocity_D	= 0.0;
    m_settings.azMotor_LPF_velocity_TF	= 0.05;
    m_settings.azMotor_electric_angle   = 0.0;
    m_settings.azZero_encoder_offet     = 0.0;


    m_settings.elMotor_voltage_limit	= 6.0;
    m_settings.elMotor_velocity_limit	= 30.0;
    m_settings.elMotor_Pid_velocity_P	= 0.0;
    m_settings.elMotor_Pid_velocity_I	= 0.0;
    m_settings.elMotor_Pid_velocity_D	= 0.0;
    m_settings.elMotor_LPF_velocity_TF	= 0.0;
    m_settings.elMotor_electric_angle   = 0.0;
    m_settings.elZero_encoder_offet     = 0.0;

}


void SettingsManager::PrintAllData()
{
	printf("version %u\n\r", m_settings.version );

	printf("azMotor_Voltage_limit   %f \n\r",	m_settings.azMotor_voltage_limit);
	printf("azMotor_velocity_limit  %f \n\r",	m_settings.azMotor_velocity_limit);
	printf("azMotor_Pid_velocity_P  %f \n\r",   m_settings.azMotor_Pid_velocity_P);
	printf("azMotor_Pid_velocity_I  %f \n\r",	m_settings.azMotor_Pid_velocity_I);
	printf("azMotor_Pid_velocity_D  %f \n\r",   m_settings.azMotor_Pid_velocity_D);
	printf("azMotor_LPF_velocity_TF	%f \n\r", 	m_settings.azMotor_LPF_velocity_TF);
	printf("azMotor_electric_angle	%f \n\r", 	m_settings.azMotor_electric_angle);
	printf("az_encoder_zero_offset	%f \n\r", 	m_settings.azZero_encoder_offet);



	printf("elMotor_Voltage_limit   %f \n\r",	m_settings.elMotor_voltage_limit);
	printf("elMotor_velocity_limit  %f \n\r",	m_settings.elMotor_velocity_limit);
	printf("elMotor_Pid_velocity_P  %f \n\r",   m_settings.elMotor_Pid_velocity_P);
	printf("elMotor_Pid_velocity_I  %f \n\r",	m_settings.elMotor_Pid_velocity_I);
	printf("elMotor_Pid_velocity_D  %f \n\r",   m_settings.elMotor_Pid_velocity_D);
	printf("elMotor_LPF_velocity_TF	%f \n\r", 	m_settings.elMotor_LPF_velocity_TF);
	printf("elMotor_electric_angle  %f \n\r", 	m_settings.elMotor_electric_angle);
	printf("el_encoder_zero_offset	%f \n\r", 	m_settings.elZero_encoder_offet);


}

uint32_t SettingsManager::computeCRC() const {
    constexpr size_t dataLen = offsetof(Settings_t, crc32);
    return crc32(reinterpret_cast<const uint8_t*>(&m_settings), dataLen);
}

bool SettingsManager::verifyCRC() const {
    return computeCRC() == m_settings.crc32;
}

uint32_t SettingsManager::crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

HAL_StatusTypeDef SettingsManager::eraseFlashPage(uint32_t address) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit = {};
    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = address;
    eraseInit.NbPages     = 1;

    uint32_t pageError = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &pageError);

    HAL_FLASH_Lock();
    return status;
}
