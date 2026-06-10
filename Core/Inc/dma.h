/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file dma.h
  * @brief Header for DMA configuration
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;

void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */
