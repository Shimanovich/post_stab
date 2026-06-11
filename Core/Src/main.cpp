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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

AM4096 		pitch_encoder;
AM4096 		yaw_encoder;
ICM20602  	frameImu;

#ifdef __cplusplus
extern "C" {
#endif
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// === I2C Interrupt Mode Flags ===
volatile uint8_t i2cRxComplete = 0;
volatile uint8_t i2cError = 0;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2cRxComplete = 1;
        printf("I2C Rx complete\n\r");   // раскомментируй при отладке
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2cError = 1;
        printf("I2C Error! Code=%lu State=%d Xfer=%d\r\n",
               hi2c->ErrorCode, hi2c->State, hi2c->XferCount);

        HAL_I2C_Master_Abort_IT(hi2c, 0x30 << 1);
        HAL_Delay(3);

        // Простая попытка разблокировать шину (если SDA прижат)
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_RESET)
        {
            printf("SDA stuck low - trying bus recovery\r\n");
            // Переводим SCL в output и тактируем 9 раз
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            GPIO_InitStruct.Pin = GPIO_PIN_15;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

            for (int i = 0; i < 9; i++)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                HAL_Delay(1);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                HAL_Delay(1);
            }
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_Delay(5);

            // Возвращаем SCL обратно в AF
            GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
            GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        }

        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();

        HAL_I2C_DeInit(hi2c);
        MX_I2C1_Init();

        hi2c->State     = HAL_I2C_STATE_READY;
        hi2c->Mode      = HAL_I2C_MODE_NONE;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        i2cRxComplete = 0;
        i2cError = 0;

        printf("I2C recovered\r\n");
        HAL_Delay(20);
    }
}


#ifdef __cplusplus
}
#endif

void runMotor(void)
{
	// === НАСТРОЙКИ ДЛЯ DC-2813C + 7 В ===


		float vm = 14.0f;
		float vl = 6.0f;


	    driverMot0.voltage_power_supply = vm;
	    driverMot0.voltage_limit = vl;
	    motor0.pole_pairs = 7;
	    motor0.voltage_limit = vl;
	    motor0.velocity_limit = 30.1f;
	    motor0.controller = ControlType::velocity_openloop;

	    driverMot1.voltage_power_supply = vm;
	    driverMot1.voltage_limit = vl;
	    motor1.pole_pairs = 7;
	    motor1.voltage_limit = vl;
	    motor1.velocity_limit = 30.0f;
	    motor1.controller = ControlType::velocity_openloop;


	    driverMot0.init();
	    motor0.linkDriver(&driverMot0);
	    motor0.init();
	    motor0.sensor = nullptr;

	    driverMot1.init();
	    motor1.linkDriver(&driverMot1);
	    motor1.init();
	    motor1.sensor = nullptr;

	    motor0.enable();
	    motor1.enable();

	    HAL_Delay(8000);


	    float speed = 0.0000;

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);


//	    while(1)
//	    {
//	    	printf("Test  \n\r");
//	    	HAL_Delay(2000);
//	    }

	    HAL_Delay(8000);


	    uint32_t start_tick = HAL_GetTick();
	    uint32_t circle_tick = HAL_GetTick();

	    speed = 0.1;
	    while((HAL_GetTick() - start_tick)<50000 )
	    //while(1)
	    {

	    	if ((HAL_GetTick()-circle_tick)>10000)
	    	{
	    		speed = -speed;
	    		circle_tick = HAL_GetTick();
	    	}
//	    	speed += .001;
	    	motor0.move(speed);
	    	motor1.move(speed);
	    	HAL_Delay(1);

	    }

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);


	    while(1)
	    {

	    }

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

#endif


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
                                                       0x00,
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("Heloo\n");


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
