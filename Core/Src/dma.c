/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file dma.c
  * @brief This file provides code for the configuration
  *        of the DMA instances.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "dma.h"

/* External I2C handle from i2c.c */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

/* DMA handles */
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

DMA_HandleTypeDef hdma_usart1_rx;

/**
  * Enable DMA controller clock and configure DMA for I2C1
  */
void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* I2C1_TX Init */
	hdma_i2c1_tx.Instance = DMA1_Channel6;
	hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
	hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;

	if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK) {
		Error_Handler();
	}

	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	__HAL_LINKDMA(&hi2c1, hdmatx, hdma_i2c1_tx);

	/* I2C1_RX Init */
	hdma_i2c1_rx.Instance = DMA1_Channel7;
	hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
	hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;

	if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK) {
		Error_Handler();
	}
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	__HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);

	/* ==================== USART1 RX DMA  ==================== */
	hdma_usart1_rx.Instance = DMA1_Channel5; // USART1_RX обычно на Channel 5 (STM32F3)
	hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_rx.Init.Mode = DMA_NORMAL;        // Фиксированный размер буфера
	hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM; // Можно HIGH, как у I2C

	if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
		Error_Handler();
	}

	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4, 0); // важнее  чем от i2c
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	__HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
	/* ======================================================================== */
}

/**
  * @brief DMA MSP Initialization
  */
void HAL_DMA_MspInit(DMA_HandleTypeDef* dmaHandle)
{
    if(dmaHandle->Instance == DMA1_Channel6)        // I2C1_TX
    {
        __HAL_RCC_DMA1_CLK_ENABLE();

        HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    }
    else if(dmaHandle->Instance == DMA1_Channel7)   // I2C1_RX
    {
        __HAL_RCC_DMA1_CLK_ENABLE();

        HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    }
    else if (dmaHandle->Instance == DMA1_Channel5)   // USART1_RX
        {
            __HAL_RCC_DMA1_CLK_ENABLE();

            HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4, 0);
            HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
        }
}

/**
  * @brief DMA MSP De-Initialization
  */
void HAL_DMA_MspDeInit(DMA_HandleTypeDef* dmaHandle)
{
    if(dmaHandle->Instance == DMA1_Channel6)
    {
        HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
    }
    else if(dmaHandle->Instance == DMA1_Channel7)
    {
        HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);
    }
    else if (dmaHandle->Instance == DMA1_Channel5)
    {
        HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    }
}
