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

ICM20602  	frameImu;
ICM20602  	baseImu;
static filter_ctx_t  filter_ctx;



UartProtocolParser parser;

ByteFifo fifo(32);



float az_spped = 0;

float el_speed = 0;





sensors chainI2C = sensors(&hi2c1);

uint8_t rx_byte;
float gyro_Shift=0.0f;

//void SetP(char* cmd) {
//	float val = atof(cmd);
//	motor1.PID_velocity.P = val;
//
// }
//
//void SetI(char* cmd) {
//	float val = atof(cmd);
//	motor1.PID_velocity.I = val;
//
// }
//
//void SetD(char* cmd) {
//	float val = atof(cmd);
//	motor1.PID_velocity.D = val;
//
// }
//
//void SetTf(char* cmd) {
//	float val = atof(cmd);
//
//	LPF_velocity.Tf = val;
//	//motor1.LPF_velocity.Tf = val;
// }
//
//
//void SetRamp(char* cmd) {
//	float val = atof(cmd);
//	printf("x\n");
//	gyro_Shift = val;
// }




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
        // Сразу перезапускаем DMA-приём фиксированного объёма
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
	  timerCnt++;
	  chainI2C.Start();
  }
}


#ifdef __cplusplus
}
#endif


void step_motor() {
    float gxyz[3] = {0};
    if (chainI2C.get_gyro_gimb(gxyz) > 0) {
        float desiredRelSpeed = stabilizationPID( el_speed - gxyz[0] );   // только outer
        motor1.loopFOC();
        motor1.move( -desiredRelSpeed );   // библиотека сама закроет velocity loop
    }
}



/*************************************
 *
 *
 */

void initMotor(void)
{
	// === НАСТРОЙКИ ДЛЯ DC-2813C + 7 В ===


		float vm = 14.0f;
		float vl = 7.0f;


	    driverMot0.voltage_power_supply = vm;
	    driverMot0.voltage_limit = vl;
	    motor0.pole_pairs = 7;
	    motor0.voltage_limit = vl;
	    motor0.velocity_limit = 30.1f;
	    motor0.controller = ControlType::velocity;

	    driverMot1.voltage_power_supply = vm;
	    driverMot1.voltage_limit = vl;
	    motor1.pole_pairs = 7;
	    motor1.voltage_limit = vl;
	    motor1.velocity_limit = 30.0f;
	    motor1.controller = ControlType::velocity;

	    motor1.PID_velocity.P = 0.5f;   // если используете режим velocity
	    motor1.PID_velocity.I = 10.0f;
	    motor1.PID_velocity.D = 0.0f;
	    motor1.PID_velocity.output_ramp = 1000.0f;
	    motor1.voltage_limit = 7.0f;

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
	    printf("Wait\n\r");
	    printf("start\n\r");


	    Filter_Init(&filter_ctx, "AHRS");
	    filter_ctx.mode = 5;


}

//	    //HAL_Delay(2000);
//
//
//	    uint32_t start_tick = HAL_GetTick();
//	    uint32_t circle_tick = HAL_GetTick();
//
//	    speed = 0.1;
//
//	    int movcnt = 0;
//    	float gxyz[3];
//
//	    while((HAL_GetTick() - start_tick)<10000 )
//	    {
//
//
//	    	if (chainI2C.get_gyro(gxyz)>10)
//	    	{
//	    		printf(">gx:%f\n",gxyz[0]);
//	    		printf(">fx:%f\n",gyro_filter(gxyz[0]));
//	    	}
//	    	else
//	    	{
//	    		printf("wait imu\n\r");
//
//	    	}
//
//	    	//HAL_Delay(1);
//
//	    	movcnt++;
//
//	    }
//
//	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
//
//	    HAL_TIM_Base_Stop(&htim6);
//
//	    while(1)
//	    {
//
//	    }
//
//}

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

void DWT_Init(void)
{
    // Включаем доступ к DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Сбрасываем счётчик (по желанию — можно не сбрасывать)
    DWT->CYCCNT = 0;

    // Включаем счётчик циклов
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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


void I2C_Recover(I2C_HandleTypeDef *hi2c)
{
    // 1. Прерываем текущую операцию
    HAL_I2C_Master_Abort_IT(hi2c, 0x30 << 1);
    HAL_Delay(3);

    // 2. Bus Recovery (если SDA прижат)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_RESET)   // SDA = PA14
    {
        printf("SDA stuck → Bus Recovery\r\n");

        // Переводим SCL в Output Open-Drain
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin   = GPIO_PIN_15;           // SCL = PA15
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull  = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // 9 тактов SCL
        for (int i = 0; i < 9; i++)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
            HAL_Delay(1);
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_Delay(5);

        // Возвращаем SCL в режим I2C
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    // 3. Полный сброс периферии
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    HAL_I2C_DeInit(hi2c);
    MX_I2C1_Init();

    // 4. Принудительно сбрасываем состояние
    hi2c->State     = HAL_I2C_STATE_READY;
    hi2c->Mode      = HAL_I2C_MODE_NONE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

//    // 5. Сбрасываем пользовательские флаги
//    i2cRxComplete = 0;
//    i2cError = 0;

    printf("I2C Recovered\r\n");
    HAL_Delay(15);
}




void run_encoder_test()
{

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


	DWT_Init();


	uint32_t prev[5];
	uint32_t cur[5];

	uint32_t tim_prev;
	uint32_t tim_cur;



	chainI2C.init_chain();
	printf("Chain init\n\r");
	initMotor();


	// Привязываем обработчики к ключам
	parser.registerHandler(0x0001, [](uint16_t key, uint32_t value) {
	    // Обработка параметра 0x0001
	    az_spped = *(float*)&value;
	    //printf("az %f\n\r",az_spped);
	});

	parser.registerHandler(0x0002, [](uint16_t key, uint32_t value) {
	    // Команда
	    el_speed = *(float*)&value;
	    //printf("el %f\n\r",el_spped);
	});


	parser.registerHandler(0x0003, [](uint16_t key, uint32_t value) {
	    // Команда


	});

	// Опционально — обработчик всех неизвестных ключей
	parser.setDefaultHandler([](uint16_t key, uint32_t value) {
	    printf("Unknown key 0x%04X, value=%lu\n", key, value);
	});


	//HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	HAL_UART_Receive_DMA(&huart1, uart_rx_dma_buffer, UART_RX_DMA_BUF_SIZE);

	HAL_TIM_Base_Start_IT(&htim6);


	//while(1)
	//{

		//start _motor
	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);


	   // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	//}


	    float w;



	    uint8_t b;
	while(1)
	{
			{

				if (chainI2C.overload>10)
				{
					chainI2C.overload=0;
					HAL_TIM_Base_Stop_IT(&htim6);
					I2C_Recover(&hi2c1);
					HAL_TIM_Base_Start_IT(&htim6);
				}

					//printf(">px:%f\n",pitchEmulator.getAngle());

					printf(">vp:%f\n",motor1.shaft_velocity_sp);
					printf(">va:%f\n",motor0.shaft_velocity_sp);


					chainI2C.get_gyro_gimb(&w);
					printf(">gp:%f\n",w);

					chainI2C.get_gyro_static(&w);
					printf(">ga:%f\n",w);
//					printf(">s1:%f\n",gxyz[1]);
//					printf(">s2:%f\n",gxyz[2]);

//					printf(">Av:%f\n",az_spped);
//					printf(">Pv:%f\n",el_spped);

					printf(">ov:%d\n",chainI2C.overload);



					if (fifo.pop(b))
					{
						//commander.processIncomingChar(b);
						parser.feed(b);
					}
//					HAL_UART_Receive(&huart1, &rx_byte, 1,1);
//					commander.processIncomingChar(rx_byte);
					//HAL_Delay(1);
				}
	}


	while(1)
	{

		float imu[3];

//		if (chainI2C.get_gyro(imu))
//		{
//			printf(">gx:%f\n",imu[0]);
//			printf(">gy:%f\n",imu[1]);
//			printf(">gz:%f\n",imu[2]);
//
//			HAL_Delay(1);
//		}

//		printf(">gx:%d\n",chainI2C.raw_imu_gyro[0]);


		cur[0] = chainI2C.chain[0].cnt;
		cur[1] = chainI2C.chain[1].cnt;
		cur[2] = chainI2C.chain[2].cnt;
		cur[3] = chainI2C.chain[3].cnt;
		cur[4] = chainI2C.chain[4].cnt;


		tim_cur = timerCnt;


		printf("Cnt: 0: %d 1: %d 2: %d 3: %d 4: %d   ---- ",cur[0],cur[1],cur[2],cur[3],cur[4]);

		printf("rate: 0: %d 1: %d 2: %d 3: %d 4: %d  timcnt: %d \n\r",cur[0]-prev[0],cur[1]-prev[1],cur[2]-prev[2],cur[3]-prev[3],cur[4]-prev[4],tim_cur-tim_prev);

		prev[0] = cur[0];
		prev[1] = cur[1];
		prev[2] = cur[2];
		prev[3] = cur[3];
		prev[4] = cur[4];

		tim_prev = tim_cur;


		HAL_Delay(1000);
	}

//	while(1)
//	{
//		uint16_t pos;
//		float angle;
//		HAL_StatusTypeDef res;
//
//
//		uint32_t end;
//
//
//
//		DWT->CYCCNT = 0;
//		res= pitch_encoder.getAbsolutePosition(&pos);
//		end = DWT->CYCCNT;
//        if (res == HAL_OK)
//        {
//        	pitch_encoder.getAngleDegrees(&angle);
//#if DIAG_PRINT
//            printf(">pA:%f\n",angle);
//            printf(">pP:%d\n",pos);
//            printf(">dtp:%d\n",end / (SystemCoreClock / 1000000));
//#endif
//
//		}
//        else
//        {
//        	printf("Enc pitch err %d \n", res);
//        }
//
//
//
//
//		DWT->CYCCNT = 0;
//		int r1= I2C_MemRead_LowLevel(0x30, 33, (uint8_t*)&pos, 2);
//		end = DWT->CYCCNT;
//        if (res == 0)
//        {
//        	pitch_encoder.getAngleDegrees(&angle);
//#if DIAG_PRINT
//            printf(">dA:%f\n",angle);
//            printf(">dP:%d\n",pos);
//            printf(">dtd:%d\n",end / (SystemCoreClock / 1000000));
//#endif
//
//		}
//        else
//        {
//        	printf("Enc pitch err %d \n", res);
//        }
//
//
//        DWT->CYCCNT = 0;
//		res= yaw_encoder.getAbsolutePosition(&pos);
//		end = DWT->CYCCNT;
//        if (res == HAL_OK)
//        {
//        	yaw_encoder.getAngleDegrees(&angle);
//#if DIAG_PRINT
//            printf(">yA:%f\n",angle);
//            printf(">yP:%d\n",pos);
//            printf(">dty:%d\n",end / (SystemCoreClock / 1000000));
//#endif
//		}
//        else
//        {
//          	printf("Enc yaw err %d \n", res);
//        }
//
//        DWT->CYCCNT = 0;
//        float gyro[3], accel[3], temp;
//		int gres= frameImu.read(gyro, accel, &temp);
//		end = DWT->CYCCNT;
//        if (gres == ICM20602_OK)
//        {
//#if DIAG_PRINT
//            printf(">g0:%f\n",gyro[0]);
//            printf(">g1:%f\n",gyro[1]);
//            printf(">g2:%f\n",gyro[2]);
//            printf(">dtg:%d\n",end / (SystemCoreClock / 1000000));
//#endif
//		}
//        else
//        {
//          	printf("Enc yaw err %d \n", res);
//        }
//
//
//
//        HAL_Delay(1);
//	}
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



  //I2C_ScanExternalBus(&hi2c1);

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
