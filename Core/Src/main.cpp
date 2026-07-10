/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "BLDCDriver3PWM.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "stdio.h"
#include "AM4096.h"
#include "icm20602.h"
#include "sensors.h"
#include "uartParcer.h"
#include "bytefifo.h"
#include "encoderDataDma.h"
#include "LADRC_SpeedController.h"

#include "settings_manager.h"

#include <cmath>



#define FILTER_AHRS_ENABLED
#include "filter_ahrs.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_RX_DMA_BUF_SIZE  8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_rx_dma_buffer[UART_RX_DMA_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemClock_Config_104MHz(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Драйверы
BLDCDriver3PWM driverMot0(&htim2, TIM_CHANNEL_2, &htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4);

BLDCDriver3PWM driverMot1(&htim3, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_3, &htim3, TIM_CHANNEL_4);

// Моторы DC-2813C (7 pole pairs)
BLDCMotor motor0 = BLDCMotor(7);
BLDCMotor motor1 = BLDCMotor(7);


//#define DEF_POWER_SUPPLY 14.0 //!< default power supply voltage
//// velocity PI controller params
//#define DEF_PID_VEL_P 0.5 //!< default PID controller P value
//#define DEF_PID_VEL_I 10.0 //!<  default PID controller I value
//#define DEF_PID_VEL_D 0.0 //!<  default PID controller D value
//#define DEF_PID_VEL_U_RAMP 1000.0 //!< default PID controller voltage ramp value


PIDController  stabilizationPID = PIDController(DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_U_RAMP,30.0f);
PIDController  velocityPID = PIDController(DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_U_RAMP,30.0f);

AM4096 		pitch_encoder;
AM4096 		yaw_encoder;

dataDma    pitchDma;
dataDma    yawDma;

SettingsManager   settings;

ICM20602  	frameImu;
ICM20602  	baseImu;
static filter_ctx_t  filter_ctx;



UartProtocolParser parser;

ByteFifo fifo(32);

float az_speed = 0;
float el_speed = 0;


bool Need_to_calibrate = false;

float up_test_pos;
float dw_test_pos;
float left_test_pos;
float right_test_pos;


bool runtest_el, runtest_az;

bool autopid_az = false;
bool autopid_el = false;



sensors chainI2C = sensors(&hi2c1);

uint8_t rx_byte;
float gyro_Shift=0.0f;






#ifdef __cplusplus
extern "C" {
#endif
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}



// === Callbacks ===
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    	chainI2C.singleEvent();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Копируем весь DMA-буфер в fifo (как в оригинале — побайтово)
        for (uint16_t i = 0; i < UART_RX_DMA_BUF_SIZE; i++) {
            fifo.push(uart_rx_dma_buffer[i]);
        }
        HAL_UART_Receive_DMA(&huart1, uart_rx_dma_buffer, UART_RX_DMA_BUF_SIZE);
    }
}





void I2C_Recover(I2C_HandleTypeDef *hi2c);

//void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
//{
//    if (hi2c->Instance == I2C1)
//    {
//      //  i2cError = 1;
//        printf("I2C Error! Code=%lu State=%d\r\n", hi2c->ErrorCode, hi2c->State);
//
////        // Более агрессивный recovery при BERR и ARLO
////        HAL_I2C_Master_Abort_IT(hi2c, 0x30 << 1);
////        HAL_Delay(5);
////
////        // Дополнительная Bus Recovery при BERR/ARLO
////        if (hi2c->ErrorCode == HAL_I2C_ERROR_BERR ||
////            hi2c->ErrorCode == HAL_I2C_ERROR_ARLO)
////        {
////            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_RESET)
////            {
////                printf("Bus Recovery triggered\r\n");
////                // 9 тактов SCL
////                GPIO_InitTypeDef GPIO_InitStruct = {0};
////                GPIO_InitStruct.Pin   = GPIO_PIN_15;
////                GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
////                GPIO_InitStruct.Pull  = GPIO_PULLUP;
////                HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
////
////                for (int i = 0; i < 9; i++) {
////                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
////                    HAL_Delay(1);
////                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
////                    HAL_Delay(1);
////                }
////                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
////                HAL_Delay(5);
////
////                GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
////                GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
////                HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
////            }
////        }
////
////        __HAL_RCC_I2C1_FORCE_RESET();
////        __HAL_RCC_I2C1_RELEASE_RESET();
////
////        HAL_I2C_DeInit(hi2c);
////        MX_I2C1_Init();
////
////        hi2c->State     = HAL_I2C_STATE_READY;
////        hi2c->Mode      = HAL_I2C_MODE_NONE;
////        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
////
////        i2cRxComplete = 0;
////        i2cError = 0;
////
////        printf("I2C Recovered (after BERR/ARLO)\r\n");
////        HAL_Delay(20);
//    }
//}


uint32_t timerCnt=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		timerCnt++;
		chainI2C.Start();

		if (chainI2C.overload > 100) {
			HAL_TIM_Base_Stop_IT(&htim6);
			I2C_Recover(&hi2c1);
			chainI2C.overload = 0;
			HAL_TIM_Base_Start_IT(&htim6);
		}
	}
}


#ifdef __cplusplus
}
#endif


void step_motor() {
	motor0.sensor->update_syncVelocity();
	motor1.sensor->update_syncVelocity();

	float gxyz[3] = { 0 };
	if (chainI2C.get_gyro_gimb(gxyz) > 0) {
		motor1.move(el_speed);
		if ((motor1.controller == ControlType::velocity)||
		    (motor1.controller == ControlType::angle)||
			(motor1.controller == ControlType::voltage))
		{
			motor1.loopFOC();
		}

		motor0.move(az_speed);
		if ((motor0.controller == ControlType::velocity)||
			(motor0.controller == ControlType::angle)||
			(motor0.controller == ControlType::voltage))
		{
			motor0.loopFOC();
		}
	}



}



/*************************************
 *
 *
 */

void initMotor(void)
{

		float vm = 14.0f;

		// settings for drive 0 (Azimuth engine)
	    driverMot0.voltage_power_supply = vm;
	    driverMot0.voltage_limit = settings.get().azMotor_voltage_limit;
	    motor0.pole_pairs = 7;
	    motor0.voltage_limit = settings.get().azMotor_voltage_limit;
	    motor0.velocity_limit = settings.get().azMotor_velocity_limit;
	    motor0.controller = ControlType::velocity;
	    motor0.foc_modulation = FOCModulationType::SinePWM;

	    motor0.PID_velocity.P = settings.get().azMotor_Pid_velocity_P;
	    motor0.PID_velocity.I = settings.get().azMotor_Pid_velocity_I;
	    motor0.PID_velocity.D = settings.get().azMotor_Pid_velocity_D;
	    motor0.LPF_velocity.Tf = settings.get().azMotor_LPF_velocity_TF;

	    motor0.PID_velocity.output_ramp = 10000.0f; // ??
	    motor0.PID_velocity.limit = settings.get().azMotor_velocity_limit;
	    motor0.voltage_sensor_align = settings.get().azMotor_voltage_limit;

	    yawDma.zero_offset        = settings.get().azZero_encoder_offet;

	    //motor0.sensor->natural_direction = Direction::CW;

///////////////////////////////////////////////////////////

		// settings for drive 1 (Pitch engine)
	    driverMot1.voltage_power_supply = vm;
	    driverMot1.voltage_limit = settings.get().elMotor_voltage_limit;
	    motor1.pole_pairs = 7;
	    motor1.voltage_limit = settings.get().elMotor_voltage_limit;;
	    motor1.velocity_limit = 30.0f;
	    motor1.controller = ControlType::velocity;
	    motor1.PID_velocity.output_ramp = 10000.0f;
	    motor1.PID_velocity.limit = settings.get().elMotor_velocity_limit;
	    motor1.foc_modulation = FOCModulationType::SinePWM;


	    motor1.PID_velocity.P = settings.get().elMotor_Pid_velocity_P;
	    motor1.PID_velocity.I = settings.get().elMotor_Pid_velocity_I;
	    motor1.PID_velocity.D = settings.get().elMotor_Pid_velocity_D;
	    motor1.LPF_velocity.Tf = settings.get().elMotor_LPF_velocity_TF;
	    motor1.voltage_sensor_align = settings.get().elMotor_voltage_limit;

	    pitchDma.zero_offset        = settings.get().elZero_encoder_offet;

	    //motor1.sensor->natural_direction = Direction::UNKNOWN;
////////////////////////////////////////////////////////////////


	    driverMot0.init();
	    motor0.linkDriver(&driverMot0);
	    motor0.init();
	    motor0.sensor = &yawDma;

	    driverMot1.init();
	    motor1.linkDriver(&driverMot1);
	    motor1.init();
	    motor1.sensor = &pitchDma;


	    motor0.enable();
	    motor1.enable();

	    printf("init  Motor Done\n\r");

//	    Filter_Init(&filter_ctx, "AHRS");
//	    filter_ctx.mode = 5;


}


void I2C_ScanExternalBus(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        printf("Error: hi2c == NULL\r\n");
        return;
    }

    printf("=== SCAN I2C BUS ===\r\n");
    printf("Devs addrs (7-bit):\r\n");

    uint8_t found = 0;
    uint32_t start_tick = HAL_GetTick();

    for (uint8_t addr = 1; addr < 128; addr++)   // 0x01 .. 0x7F
    {
        // Проверяем наличие устройства (3 попытки, таймаут 100 мс)
        if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 3, 100) == HAL_OK)
        {
            printf("  find: 0x%02X  (0x%02X)\r\n", addr, (addr << 1));
            found++;
        }

        // Небольшая пауза, чтобы не забивать шину
        HAL_Delay(1);
    }

    uint32_t duration_ms = HAL_GetTick() - start_tick;

    if (found == 0)
        printf("  No devs find!\r\n");
    else
        printf("  find devs : %d\r\n", found);

    printf("end of scan  %lu ms\r\n", duration_ms);
    printf("====================================\r\n\r\n");
}




#define DIAG_PRINT 1


// Возвращает:
// 0  - успех
// 1  - таймаут на этапе отправки адреса устройства
// 2  - таймаут на этапе отправки адреса регистра
// 3  - таймаут на этапе чтения данных
// 4  - таймаут на STOP
// 5  - ошибка на шине (NACK / ARLO / BERR)
// 6  - SDA прижат к земле (bus stuck)



// Новая универсальная функция Bus Recovery
void I2C_BusRecovery(uint8_t isAltPins)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_TypeDef* GPIOx;
    uint16_t SCL_Pin, SDA_Pin;

    if (isAltPins) {
        GPIOx = GPIOA;
        SCL_Pin = GPIO_PIN_15;
        SDA_Pin = GPIO_PIN_14;
        printf("Bus Recovery: Alternative pins (PA14/PA15)\r\n");
    } else {
        GPIOx = GPIOB;
        SCL_Pin = GPIO_PIN_6;
        SDA_Pin = GPIO_PIN_7;
        printf("Bus Recovery: Main pins (PB6/PB7)\r\n");
    }

    // 1. Освобождаем оба пина в Open-Drain
    GPIO_InitStruct.Pin   = SCL_Pin | SDA_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOx, SDA_Pin, GPIO_PIN_SET);  // SDA high
    HAL_GPIO_WritePin(GPIOx, SCL_Pin, GPIO_PIN_SET);  // SCL high
    HAL_Delay(1);

    // 2. Если SDA прижат — генерируем clock pulses
    if (HAL_GPIO_ReadPin(GPIOx, SDA_Pin) == GPIO_PIN_RESET) {
        printf("SDA stuck low → generating 10 SCL pulses\r\n");
        for (int i = 0; i < 10; i++) {
            HAL_GPIO_WritePin(GPIOx, SCL_Pin, GPIO_PIN_RESET);
            HAL_Delay(1);           // ~500-1000 Гц (можно Delay_us для точности)
            HAL_GPIO_WritePin(GPIOx, SCL_Pin, GPIO_PIN_SET);
            HAL_Delay(1);
        }
    }

    // 3. Генерируем STOP условие (SCL high + SDA low→high)
    HAL_GPIO_WritePin(GPIOx, SDA_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOx, SCL_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOx, SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    // 4. Восстанавливаем альтернативную функцию
    if (isAltPins) {
        Switch_I2C1_to_Alt();
    } else {
    	Switch_I2C1_to_Main();
    }
}


// ====================== ОСНОВНАЯ ФУНКЦИЯ RECOVER ======================
void I2C_Recover(I2C_HandleTypeDef *hi2c)
{
    printf("=== I2C_Recover started. Error=0x%lx State=%d ===\r\n",
           hi2c->ErrorCode, hi2c->State);

    // 1. Прервать текущую транзакцию
    HAL_I2C_Master_Abort_IT(hi2c, 0xFF);  // broadcast address
    HAL_Delay(5);

//    // 2. Определяем текущий набор пинов и делаем recovery
//    // Простая эвристика: проверяем, на каком пине SDA low
//    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_RESET) {
//        I2C_BusRecovery(1);  // alt pins
//    } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
//        I2C_BusRecovery(0);  // main pins
//    } else {
//        // Если ничего не прижато — всё равно делаем для текущего (alt по умолчанию)
//        I2C_BusRecovery(1);
//    }

    I2C_BusRecovery(0);
    I2C_BusRecovery(1);

    // 3. Полный сброс периферии
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C1_RELEASE_RESET();

    HAL_I2C_DeInit(hi2c);
    MX_I2C1_Init();

    // 4. Очистка DMA (критично!)
    if (hi2c->hdmatx != NULL) HAL_DMA_Abort(hi2c->hdmatx);
    if (hi2c->hdmarx != NULL) HAL_DMA_Abort(hi2c->hdmarx);

    hi2c->State     = HAL_I2C_STATE_READY;
    hi2c->Mode      = HAL_I2C_MODE_NONE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    HAL_Delay(10);
    printf("=== I2C_Recover finished ===\r\n");
}



void setMotorToCentres(BLDCMotor * thisMotor,float * target )
{
	ControlType old_t =thisMotor->controller;
	FOCModulationType old_mod = thisMotor->foc_modulation;

	thisMotor->foc_modulation= FOCModulationType::SinePWM;
	thisMotor->controller = ControlType::velocity_openloop;


	float oldangle;
	*target = -0.5;
	do
	{
		oldangle = thisMotor->sensor->getAngle();
		HAL_Delay(100);
	}
	while(oldangle!=thisMotor->sensor->getAngle());

	*target = 0.0;
	HAL_Delay(100);
	*target = 0.5;
	uint32_t t_mov = HAL_GetTick();
	HAL_Delay(500);
	do
	{
		oldangle = thisMotor->sensor->getAngle();
		HAL_Delay(100);

	}
	while(oldangle!=thisMotor->sensor->getAngle());
	*target = 0;
	t_mov = (HAL_GetTick() - t_mov)/2;
	*target = -0.5;
	HAL_Delay(t_mov);
	*target = 0;

	thisMotor->controller = old_t;
	thisMotor->foc_modulation = old_mod;
}




void move_motor_to_zero_position(BLDCMotor * thisMotor, float * target, float polarity)
{
    thisMotor->foc_modulation = FOCModulationType::SinePWM;
    thisMotor->controller = ControlType::velocity_openloop;

    const float PI = 3.14159265f;


    // ===  Основной цикл приближения к нулю ===
    float speed = 0.1f;

    while (fabs(thisMotor->sensor->getAngle()) > 0.01) {

        float vel_for_pos = + polarity * speed;
        float vel_for_neg = - polarity * speed;

        while (thisMotor->sensor->getAngle() > 0.0f) {
            *target = vel_for_pos;
            printf(">az:%f\n", *target);
            printf(">sp:%f\n", speed);
        }

        speed /= 2.0f;

        while (thisMotor->sensor->getAngle() < 0.0f) {
            *target = vel_for_neg;
            printf(">az:%f\n", *target);
            printf(">sp:%f\n", speed);
        }

        speed /= 2.0f;

        printf(">az:%f\n", *target);
        printf(">sp:%f\n", speed);
    }

    printf("Stopped at position %f\n\r", thisMotor->sensor->getAngle());
    *target = 0.0f;
}


// Пример структуры (упрощённо)
void autoTuneVelocityPID(BLDCMotor* motor, uint8_t motorId,float *speedPointer, float voltageAmplitude = 2.5f) {

	printf("=== Auto-tuning PID for Motor %d (voltageAmplitude=%.1f) ===\r\n", motorId, voltageAmplitude);
    // Переключаемся в режим voltage (torque)
    motor->controller = ControlType::voltage;   // или voltage
    motor->PID_velocity.P = 0;

    float targetVoltage = voltageAmplitude;
    uint32_t start = HAL_GetTick();
    float lastVel = 0;
    int crossings = 0;
    float periodSum = 0;
    float maxVel = 0, minVel = 0;

    while (HAL_GetTick() - start < 4000) {   // 4 секунд

    	*speedPointer = targetVoltage;

    	float vel = motor->shaft_velocity;

        // Детектируем пересечение нуля скорости
        if ((lastVel * vel) < 0 && fabs(vel) > 0.5f) {
            crossings++;
            if (crossings > 3) {
                periodSum += (HAL_GetTick() - start) * 0.001f;
            }
        }
        lastVel = vel;

        if (vel > maxVel) maxVel = vel;
        if (vel < minVel) minVel = vel;

        // Переключаем реле
        if ((targetVoltage > 0 && vel > 1.0f) || (targetVoltage < 0 && vel < -1.0f)) {
            targetVoltage = -targetVoltage;
        }

        //HAL_Delay(1);
    }

    // Расчёт Ku и Tu
        float Ku = (4.0f * voltageAmplitude) / (M_PI * (maxVel - minVel + 0.01f));
        float Tu = (crossings > 5) ? (periodSum / (crossings - 3)) : 0.0f;

        if (Tu > 0.01f && Ku > 0.05f) {
            motor->PID_velocity.P = 0.45f * Ku;
            motor->PID_velocity.I = (0.54f * Ku) / Tu;
            motor->PID_velocity.D = 0.0f;

            printf("Motor %d tuned OK: P=%.3f, I=%.3f (Ku=%.3f, Tu=%.3f)\r\n",
                   motorId, motor->PID_velocity.P, motor->PID_velocity.I, Ku, Tu);

            // Сохранение в настройки
            if (motorId == 0) {
                settings.get().azMotor_Pid_velocity_P = motor->PID_velocity.P;
                settings.get().azMotor_Pid_velocity_I = motor->PID_velocity.I;
            } else {
                settings.get().elMotor_Pid_velocity_P = motor->PID_velocity.P;
                settings.get().elMotor_Pid_velocity_I = motor->PID_velocity.I;
            }
            //settings.saveToFlash();
            //printf("Settings saved to flash\r\n");
        } else {
            printf("Motor %d tuning FAILED (weak oscillations or Ku/Tu invalid)\r\n", motorId);

        }

        // Возврат в нормальный режим

       *speedPointer =0.0f;


        if (motorId == 0)
        	motor1.controller = ControlType::velocity;
        else
        	motor0.controller = ControlType::velocity;


        printf("=== Tuning Motor %d finished ===\r\n", motorId);
}


void run_encoder_test()
{
	//
	Switch_I2C1_to_Main();
	baseImu.begin(&hi2c1, 0x68);
	Switch_I2C1_to_Alt();
	pitch_encoder.begin(&hi2c1, 0x30);
	yaw_encoder.begin(&hi2c1, 0x31);
	frameImu.begin(&hi2c1, 0x68);


//	pitchEmulator.begin(chainI2C.raw_pitch_enc);
//	yawEmulator.begin(chainI2C.raw_yaw_enc);

	pitchDma.begin(chainI2C.raw_pitch_enc, chainI2C.raw_imu_gyro_gimb);
	  yawDma.begin(chainI2C.raw_yaw_enc,   chainI2C.raw_imu_gyro_gimb);



	chainI2C.init_chain();

	initMotor();


	// Привязываем обработчики к ключам
	parser.registerHandler(0x0001, [](uint16_t key, uint32_t value) { az_speed = *(float*)&value;});
	parser.registerHandler(0x0002, [](uint16_t key, uint32_t value) {  el_speed = *(float*)&value;});

	parser.registerHandler(0xABCD, [](uint16_t key, uint32_t value) { if(settings.saveToFlash()) {printf(" Save OK\n\r");}else {printf(" Save ERROR\n\r");}});
	parser.registerHandler(0xAABB, [](uint16_t key, uint32_t value) { settings.PrintAllData();});

	parser.registerHandler(0xAACC, [](uint16_t key, uint32_t value) {
		if (settings.loadFromFlash()) {
			printf(" Load OK\n\r");
			initMotor();
		} else {
			printf(" Load ERROR\n\r");
		}
	});

	parser.registerHandler(0xBBAA, [](uint16_t key, uint32_t value) { Need_to_calibrate = true;});



	parser.registerHandler(0x0013, [](uint16_t key, uint32_t value) {settings.get().azMotor_Pid_velocity_P   = motor0.PID_velocity.P  = *(float*)&value;});
	parser.registerHandler(0x0014, [](uint16_t key, uint32_t value) {settings.get().azMotor_Pid_velocity_I   = motor0.PID_velocity.I  = *(float*)&value;});
	parser.registerHandler(0x0015, [](uint16_t key, uint32_t value) {settings.get().azMotor_Pid_velocity_D   = motor0.PID_velocity.D  = *(float*)&value;});
	parser.registerHandler(0x0016, [](uint16_t key, uint32_t value) {settings.get().azMotor_LPF_velocity_TF  = motor0.LPF_velocity.Tf = *(float*)&value;});

	parser.registerHandler(0x0017, [](uint16_t key, uint32_t value) {settings.get().azMotor_voltage_limit    = motor0.driver->voltage_limit = motor0.voltage_limit = *(float*)&value;});

	parser.registerHandler(0x0018, [](uint16_t key, uint32_t value) { motor0.absoluteZeroAlign();  settings.get().azZero_encoder_offet = yawDma.zero_offset;});



	parser.registerHandler(0x0023, [](uint16_t key, uint32_t value) {settings.get().elMotor_Pid_velocity_P   = motor1.PID_velocity.P  = *(float*)&value;});
	parser.registerHandler(0x0024, [](uint16_t key, uint32_t value) {settings.get().elMotor_Pid_velocity_I   = motor1.PID_velocity.I  = *(float*)&value;});
	parser.registerHandler(0x0025, [](uint16_t key, uint32_t value) {settings.get().elMotor_Pid_velocity_D   = motor1.PID_velocity.D  = *(float*)&value;});
	parser.registerHandler(0x0026, [](uint16_t key, uint32_t value) {settings.get().elMotor_LPF_velocity_TF  = motor1.LPF_velocity.Tf = *(float*)&value;});

	parser.registerHandler(0x0027, [](uint16_t key, uint32_t value) {settings.get().elMotor_voltage_limit    = motor1.driver->voltage_limit = motor1.voltage_limit = *(float*)&value;});

	parser.registerHandler(0x0028, [](uint16_t key, uint32_t value) {motor1.absoluteZeroAlign();  settings.get().elZero_encoder_offet = pitchDma.zero_offset;});




	parser.registerHandler(0x0030, [](uint16_t key, uint32_t value) { up_test_pos = pitchDma.getAngle(); });
	parser.registerHandler(0x0031, [](uint16_t key, uint32_t value) { dw_test_pos = pitchDma.getAngle(); });

	parser.registerHandler(0x0032, [](uint16_t key, uint32_t value) { runtest_el = true;   el_speed = 1.0f; });
	parser.registerHandler(0x0033, [](uint16_t key, uint32_t value) { runtest_el = false;  el_speed = 0.0f; });

	parser.registerHandler(0x0040, [](uint16_t key, uint32_t value) { left_test_pos = yawDma.getAngle(); });
	parser.registerHandler(0x0041, [](uint16_t key, uint32_t value) { right_test_pos = yawDma.getAngle(); });

	parser.registerHandler(0x0042, [](uint16_t key, uint32_t value) { runtest_az = true;   az_speed = 1.0f; });
	parser.registerHandler(0x0043, [](uint16_t key, uint32_t value) { runtest_az = false;  az_speed = 0.0f; });

	parser.registerHandler(0x0050, [](uint16_t key, uint32_t value) { autopid_az = true; });
	parser.registerHandler(0x0051, [](uint16_t key, uint32_t value) { autopid_el = true; });





	// Опционально — обработчик всех неизвестных ключей
	parser.setDefaultHandler([](uint16_t key, uint32_t value) {
		printf("Unknown key 0x%04X, value=%lu\n", key, value);
	});

	//HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	HAL_UART_Receive_DMA(&huart1, uart_rx_dma_buffer, UART_RX_DMA_BUF_SIZE);


	HAL_TIM_Base_Start_IT(&htim6);
	HAL_Delay(100);


	motor0.initFOC(settings.get().azMotor_electric_angle);
	motor1.initFOC(settings.get().elMotor_electric_angle);

	el_speed = 0.0;

	uint8_t b;



	while(1)
	{



		if (fifo.pop(b))
		{
			parser.feed(b);
		}



				if (Need_to_calibrate)
				{


					motor0.zero_electric_angle = 0;
					motor1.controller = ControlType::none;
					move_motor_to_zero_position(&motor0, &az_speed, +1.0f);
					HAL_Delay(100);
					motor0.setZeroPosDirect();
					HAL_Delay(2000);

					motor0.controller = ControlType::none;
					motor0.initFOC(motor0.sensor->getAngle()-M_PI/2.0f);
					motor0.controller = ControlType::voltage;

					printf("az electric angle %f \n\r",motor0.zero_electric_angle);

					//motor0.controller = ControlType::voltage;



//					motor1.zero_electric_angle = 0;
//					motor0.controller = ControlType::none;
//					move_motor_to_zero_position(&motor1, &el_speed, -1.0f);
//					HAL_Delay(100);
//
//					//reet back all sensors
//
//					motor0.controller = ControlType::none;
//
//
//					motor0.setZeroPosDirect();
//					HAL_Delay(500);



					//motor0.controller = ControlType::none;


//					//motor0.initFOC(NOT_SET);
//					motor0.controller = ControlType::voltage;
//					printf(">yaw_angle:%f\n",yawDma.getAngle());
//					motor0.absoluteZeroAlign();
//					HAL_Delay(1000);
//					printf("motor0 elangle %f\n\r", motor0.zero_electric_angle);




//					move_motor_to_zero_position(&motor1, &el_speed, -1.0f);
//					HAL_Delay(100);
//					motor1.controller = ControlType::none;
//					motor1.initFOC(NOT_SET);
//
//					printf("motor0 elangle %f\n\r", motor0.zero_electric_angle);
//					printf("motor1 elangle %f\n\r", motor1.zero_electric_angle);
//
//					motor0.controller = ControlType::velocity;
//					motor1.controller = ControlType::velocity;
//



//					etMotorToCentres(&motor0, &az_speed);
//					motor0.controller = ControlType::none;
//					motor0.initFOC(NOT_SET);
//
//
//					setMotorToCentres(&motor1, &el_speed);
//					motor1.controller = ControlType::none;
//					motor1.initFOC(NOT_SET);
//
//					settings.get().azMotor_electric_angle = motor0.zero_electric_angle;
//					settings.get().elMotor_electric_angle = motor1.zero_electric_angle;
//
//					printf("motor0 elangle %f\n\r", motor0.zero_electric_angle);
//					printf("motor1 elangle %f\n\r", motor1.zero_electric_angle);
//
//					motor0.controller = ControlType::velocity;
//					motor1.controller = ControlType::velocity;
					Need_to_calibrate  = false;


				}


				/* tune autopid */
				if (autopid_az==true)
				{
					autoTuneVelocityPID(&motor0,0,&az_speed, settings.get().azMotor_voltage_limit);
					autopid_az=false;
				}

				/* tune autopid */
				if (autopid_el==true)
				{
					autoTuneVelocityPID(&motor1,1,&el_speed, settings.get().elMotor_voltage_limit);
					autopid_el=false;
				}

				// print real angle position
				 float angle = pitchDma.getAngle();
				 printf(">pitch_angle:%f\n",angle);
				 angle = yawDma.getAngle();
				 printf(">yaw_angle:%f\n",angle);

                 printf(">vin_el:%f\n",el_speed);
				 printf(">vin_az:%f\n",az_speed);

				 printf(">azEncSpeed:%f\n",motor0.shaft_velocity);
				 printf(">elEncSpeed:%f\n",motor1.shaft_velocity);





	    int dir_az=0;
	    int dir_el=0;


		angle = pitchDma.getAngle();
		if (runtest_el) {
			if ((angle > up_test_pos) && (dir_el == 1)) {
				dir_el = 0;
				el_speed = -1.0;
			}

			if ((angle < dw_test_pos) && (dir_el == 0)) {
				dir_el = 1;
				el_speed = +1.0;
			}
		}

		angle = yawDma.getAngle();

		if (runtest_az) {
			if ((angle > left_test_pos) && (dir_az == 1)) {
				dir_az = 0;
				az_speed = -1.0;
			}

			if ((angle < right_test_pos) && (dir_az == 0)) {
				dir_az = 1;
				az_speed = +1.0;
			}
		}


	}




}

#if 0
void run_encoder_test()
{
	pitch_encoder.begin(&hi2c1, 0x30);
	yaw_encoder.begin(&hi2c1, 0x31);
	frameImu.begin(&hi2c1, 0x68);

	DWT_Init();
	while(1)
	{
		uint16_t pos;
		float angle;
		HAL_StatusTypeDef res;


		uint32_t end;



		DWT->CYCCNT = 0;
		res= pitch_encoder.getAbsolutePosition(&pos);
		end = DWT->CYCCNT;
        if (res == HAL_OK)
        {
        	pitch_encoder.getAngleDegrees(&angle);
#if DIAG_PRINT
            printf(">pA:%f\n",angle);
            printf(">pP:%d\n",pos);
            printf(">dtp:%d\n",end / (SystemCoreClock / 1000000));
#endif

		}
        else
        {
        	printf("Enc pitch err %d \n", res);
        }




		DWT->CYCCNT = 0;
		int r1= I2C_MemRead_LowLevel(0x30, 33, (uint8_t*)&pos, 2);
		end = DWT->CYCCNT;
        if (res == 0)
        {
        	pitch_encoder.getAngleDegrees(&angle);
#if DIAG_PRINT
            printf(">dA:%f\n",angle);
            printf(">dP:%d\n",pos);
            printf(">dtd:%d\n",end / (SystemCoreClock / 1000000));
#endif

		}
        else
        {
        	printf("Enc pitch err %d \n", res);
        }


        DWT->CYCCNT = 0;
		res= yaw_encoder.getAbsolutePosition(&pos);
		end = DWT->CYCCNT;
        if (res == HAL_OK)
        {
        	yaw_encoder.getAngleDegrees(&angle);
#if DIAG_PRINT
            printf(">yA:%f\n",angle);
            printf(">yP:%d\n",pos);
            printf(">dty:%d\n",end / (SystemCoreClock / 1000000));
#endif
		}
        else
        {
          	printf("Enc yaw err %d \n", res);
        }

        DWT->CYCCNT = 0;
        float gyro[3], accel[3], temp;
		int gres= frameImu.read(gyro, accel, &temp);
		end = DWT->CYCCNT;
        if (gres == ICM20602_OK)
        {
#if DIAG_PRINT
            printf(">g0:%f\n",gyro[0]);
            printf(">g1:%f\n",gyro[1]);
            printf(">g2:%f\n",gyro[2]);
            printf(">dtg:%d\n",end / (SystemCoreClock / 1000000));
#endif
		}
        else
        {
          	printf("Enc yaw err %d \n", res);
        }



        HAL_Delay(1);
	}
}




void run_encoder_test()
{
    pitch_encoder.begin(&hi2c1, 0x30);
    yaw_encoder.begin(&hi2c1, 0x31);
    frameImu.begin(&hi2c1, 0x68);

    while(1)
    {
        uint16_t pos = 0;
        i2cRxComplete = 0;
        i2cError = 0;

        // Защита от BUSY-состояния
        if (hi2c1.State != HAL_I2C_STATE_READY)
        {
            printf("I2C not READY, forcing recovery...\r\n");
            HAL_I2C_Master_Abort_IT(&hi2c1, 0x30 << 1);
            HAL_Delay(5);

            __HAL_RCC_I2C1_FORCE_RESET();
            __HAL_RCC_I2C1_RELEASE_RESET();
            HAL_I2C_DeInit(&hi2c1);
            MX_I2C1_Init();
            hi2c1.State = HAL_I2C_STATE_READY;
            HAL_Delay(10);
        }

        HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(&hi2c1,
                                                       0x30 << 1,
                                                       33,
                                                       I2C_MEMADD_SIZE_8BIT,
                                                       (uint8_t*)&pos,
                                                       2);



        if (status != HAL_OK)
        {
            printf("I2C_IT start failed: %d\r\n", status);
            if (status == HAL_BUSY)
            {
                // Дополнительный recovery при BUSY
                HAL_I2C_Master_Abort_IT(&hi2c1, 0x30 << 1);
                HAL_Delay(5);
                __HAL_RCC_I2C1_FORCE_RESET();
                __HAL_RCC_I2C1_RELEASE_RESET();
                HAL_I2C_DeInit(&hi2c1);
                MX_I2C1_Init();
                hi2c1.State = HAL_I2C_STATE_READY;
                HAL_Delay(15);
            }
            HAL_Delay(50);
            continue;
        }

        // Ожидание с таймаутом
        uint32_t timeout = HAL_GetTick() + 120;
        while (i2cRxComplete == 0 && i2cError == 0)
        {
            if (HAL_GetTick() > timeout)
            {
                printf("I2C Timeout — recovery...\r\n");
                HAL_I2C_Master_Abort_IT(&hi2c1, 0x30 << 1);
                HAL_Delay(5);

                __HAL_RCC_I2C1_FORCE_RESET();
                __HAL_RCC_I2C1_RELEASE_RESET();
                HAL_I2C_DeInit(&hi2c1);
                MX_I2C1_Init();
                hi2c1.State = HAL_I2C_STATE_READY;

                i2cRxComplete = 0;
                i2cError = 0;
                HAL_Delay(15);
                break;
            }
        }

        if (i2cRxComplete)
        {
            printf("I2C Read OK, pos=0x%04X\r\n", pos);
            i2cRxComplete = 0;
        }
        else if (i2cError)
        {
            i2cError = 0;
        }

        HAL_Delay(80);   // 80 мс между чтениями
    }
}
#endif

//void run_encoder_test()
//{
//    pitch_encoder.begin(&hi2c1, 0x30);
//    yaw_encoder.begin(&hi2c1, 0x31);
//    frameImu.begin(&hi2c1, 0x68);
//
//    while(1)
//    {
//        uint16_t pos = 0;
//        i2cRxComplete = 0;
//        i2cError = 0;
//
//        // Защита от BUSY (оставляем без изменений)
//        if (hi2c1.State != HAL_I2C_STATE_READY)
//        {
//            printf("I2C not READY, forcing recovery...\r\n");
//            HAL_I2C_Master_Abort_IT(&hi2c1, 0x30 << 1);
//            HAL_Delay(5);
//            __HAL_RCC_I2C1_FORCE_RESET();
//            __HAL_RCC_I2C1_RELEASE_RESET();
//            HAL_I2C_DeInit(&hi2c1);
//            MX_I2C1_Init();
//            hi2c1.State = HAL_I2C_STATE_READY;
//            HAL_Delay(10);
//        }
//
//        // === ИЗМЕНЕНИЕ ЗДЕСЬ ===
//        HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1,
//                                                        0x30 << 1,
//                                                        33,
//                                                        I2C_MEMADD_SIZE_8BIT,
//                                                        (uint8_t*)&pos,
//                                                        2);
//
//        if (status != HAL_OK)
//        {
//            printf("I2C_DMA start failed: %d\r\n", status);
//            if (status == HAL_BUSY)
//            {
//                HAL_I2C_Master_Abort_IT(&hi2c1, 0x30 << 1);
//                HAL_Delay(5);
//                __HAL_RCC_I2C1_FORCE_RESET();
//                __HAL_RCC_I2C1_RELEASE_RESET();
//                HAL_I2C_DeInit(&hi2c1);
//                MX_I2C1_Init();
//                hi2c1.State = HAL_I2C_STATE_READY;
//                HAL_Delay(15);
//            }
//            HAL_Delay(50);
//            continue;
//        }
//
//        // Ожидание с таймаутом (логика остаётся той же)
//        uint32_t timeout = HAL_GetTick() + 120;
//        while (i2cRxComplete == 0 && i2cError == 0)
//        {
//            if (HAL_GetTick() > timeout)
//            {
//                printf("I2C Timeout — recovery...\r\n");
//                HAL_I2C_Master_Abort_IT(&hi2c1, 0x30 << 1);
//                HAL_Delay(5);
//                __HAL_RCC_I2C1_FORCE_RESET();
//                __HAL_RCC_I2C1_RELEASE_RESET();
//                HAL_I2C_DeInit(&hi2c1);
//                MX_I2C1_Init();
//                hi2c1.State = HAL_I2C_STATE_READY;
//                i2cRxComplete = 0;
//                i2cError = 0;
//                HAL_Delay(15);
//                break;
//            }
//        }
//
//        if (i2cRxComplete)
//        {
//            printf("I2C Read OK (DMA), pos=0x%04X\r\n", pos);
//            i2cRxComplete = 0;
//        }
//        else if (i2cError)
//        {
//            i2cError = 0;
//        }
//
//        HAL_Delay(80);
//    }
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  // SystemClock_Config_104MHz();
  SystemClock_Config();





  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_I2C1_Init();

  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  bool loadFlash_result = settings.loadFromFlash();

  // enable UART IC
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	for (int i = 0; i < 10; i++) {
		printf(
				"-------------------------------------------------------------------------------------------\n\r");
	}


  printf("---------------------------------------START-----------------------------------------------\n\r");

  printf(loadFlash_result ? "Settings Load Ok \n\r" : "Settings Load ERR\n\r");


//  if (!loadFlash_result)
//  {
//	  printf("WARNING: Load Default Settings \n\r");
//	  settings.saveToFlash();
//  }




  while(1)
  {
	  run_encoder_test();
  }


  //runMotor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief  Разгон до 104 МГц (overclock)
 * @note   8 МГц HSE × 13 = 104 МГц
 *         ВНИМАНИЕ: Это неофициально! Тестируйте тщательно.
 */
void SystemClock_Config_104MHz(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Инициализация генераторов */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL13;   // ← главное изменение (8 × 13 = 104 МГц)

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Настройка шин CPU / AHB / APB */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     // HCLK = 104 МГц
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      // PCLK1 = 52 МГц  (официально max 36 МГц!)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      // PCLK2 = 104 МГц (официально max 72 МГц!)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Периферийные часы (оставлено как было) */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2
                                     | RCC_PERIPHCLK_I2C1   | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection  = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;   // Хорошо — независимо от PLL

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
