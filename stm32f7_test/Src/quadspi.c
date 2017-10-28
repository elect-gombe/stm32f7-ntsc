/**
  ******************************************************************************
  * File Name          : QUADSPI.c
  * Description        : This file provides code for the configuration
  *                      of the QUADSPI instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "quadspi.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include "composite.h"
/* USER CODE END 0 */

QSPI_HandleTypeDef hqspi;
DMA_HandleTypeDef hdma_quadspi;

/* QUADSPI init function */
void MX_QUADSPI_Init(void)
{

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 31;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_3;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */
    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();
  
    /**QUADSPI GPIO Configuration    
    PE2     ------> QUADSPI_BK1_IO2
    PF8     ------> QUADSPI_BK1_IO0
    PF9     ------> QUADSPI_BK1_IO1
    PF10     ------> QUADSPI_CLK
    PB10     ------> QUADSPI_BK1_NCS
    PD13     ------> QUADSPI_BK1_IO3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* QUADSPI DMA Init */
    /* QUADSPI Init */
    hdma_quadspi.Instance = DMA2_Stream2;
    hdma_quadspi.Init.Channel = DMA_CHANNEL_11;
    hdma_quadspi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_quadspi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_quadspi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_quadspi.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_quadspi.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_quadspi.Init.Mode = DMA_CIRCULAR;
    hdma_quadspi.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_quadspi.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_quadspi) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(qspiHandle,hdma,hdma_quadspi);

  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{

  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();
  
    /**QUADSPI GPIO Configuration    
    PE2     ------> QUADSPI_BK1_IO2
    PF8     ------> QUADSPI_BK1_IO0
    PF9     ------> QUADSPI_BK1_IO1
    PF10     ------> QUADSPI_CLK
    PB10     ------> QUADSPI_BK1_NCS
    PD13     ------> QUADSPI_BK1_IO3 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_13);

    /* QUADSPI DMA DeInit */
    HAL_DMA_DeInit(qspiHandle->hdma);
  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
#define QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE ((uint32_t)0x00000000U)          /*!<Indirect write mode*/
#define QSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)QUADSPI_CCR_FMODE_0) /*!<Indirect read mode*/
#define QSPI_FUNCTIONAL_MODE_AUTO_POLLING   ((uint32_t)QUADSPI_CCR_FMODE_1) /*!<Automatic polling mode*/
#define QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)QUADSPI_CCR_FMODE)   /*!<Memory-mapped mode*/

/**
  * @brief  DMA QSPI transmit process complete callback. 
  * @param  hdma: DMA handle
  * @retval None
  */
static void QSPI_DMATxCplt(DMA_HandleTypeDef *hdma)     
{
  QSPI_HandleTypeDef* hqspi = ( QSPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hqspi->TxXferCount = 0;
  
  vtask(composite_buffer);
  /* Enable the QSPI transfer complete Interrupt */
  __HAL_QSPI_ENABLE_IT(hqspi, QSPI_IT_TC);
}

/**
  * @brief  DMA QSPI transmit process half complete callback 
  * @param  hdma : DMA handle
  * @retval None
  */
static void QSPI_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{

  vtask(composite_buffer+BUFFER_SIZE);
}

/**
  * @brief  DMA QSPI communication error callback.
  * @param  hdma: DMA handle
  * @retval None
  */
static void QSPI_DMAError(DMA_HandleTypeDef *hdma)   
{
  QSPI_HandleTypeDef* hqspi = ( QSPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  
  /* if DMA error is FIFO error ignore it */
  if(HAL_DMA_GetError(hdma) != HAL_DMA_ERROR_FE)
  {
    hqspi->RxXferCount = 0;
    hqspi->TxXferCount = 0;
    hqspi->ErrorCode   |= HAL_QSPI_ERROR_DMA;
    
    /* Disable the DMA transfer by clearing the DMAEN bit in the QSPI CR register */
    CLEAR_BIT(hqspi->Instance->CR, QUADSPI_CR_DMAEN);
    
    /* Abort the QSPI */
    HAL_QSPI_Abort_IT(hqspi);
  }
}


/**
  * @brief  Configure the communication registers.
  * @param  hqspi: QSPI handle
  * @param  cmd: structure that contains the command configuration information
  * @param  FunctionalMode: functional mode to configured
  *           This parameter can be one of the following values:
  *            @arg QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE: Indirect write mode
  *            @arg QSPI_FUNCTIONAL_MODE_INDIRECT_READ: Indirect read mode
  *            @arg QSPI_FUNCTIONAL_MODE_AUTO_POLLING: Automatic polling mode
  *            @arg QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED: Memory-mapped mode
  * @retval None
  */
void QSPI_Config_infenity(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, uint32_t FunctionalMode)
{
  assert_param(IS_QSPI_FUNCTIONAL_MODE(FunctionalMode));

  if ((cmd->DataMode != QSPI_DATA_NONE) && (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED))
  {
    /* Configure QSPI: DLR register with the number of data to read or write */
    WRITE_REG(hqspi->Instance->DLR, 0xFFFFFFFF);
  }
      
  if (cmd->InstructionMode != QSPI_INSTRUCTION_NONE)
  {
    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
    {
      /* Configure QSPI: ABR register with alternate bytes value */
      WRITE_REG(hqspi->Instance->ABR, cmd->AlternateBytes);

      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with instruction, address and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
                                         cmd->AlternateByteMode | cmd->AddressSize | cmd->AddressMode |
                                         cmd->InstructionMode | cmd->Instruction | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(hqspi->Instance->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with instruction and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
                                         cmd->AlternateByteMode | cmd->AddressMode | cmd->InstructionMode | 
                                         cmd->Instruction | FunctionalMode));
      }
    }
    else
    {
      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with instruction and address ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode | 
                                         cmd->AddressSize | cmd->AddressMode | cmd->InstructionMode | 
                                         cmd->Instruction | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(hqspi->Instance->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only instruction ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode | 
                                         cmd->AddressMode | cmd->InstructionMode | cmd->Instruction  | 
                                         FunctionalMode));
      }
    }
  }
  else
  {
    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
    {
      /* Configure QSPI: ABR register with alternate bytes value */
      WRITE_REG(hqspi->Instance->ABR, cmd->AlternateBytes);

      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with address and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
                                         cmd->AlternateByteMode | cmd->AddressSize | cmd->AddressMode |
                                         cmd->InstructionMode | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(hqspi->Instance->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
                                         cmd->AlternateByteMode | cmd->AddressMode | cmd->InstructionMode | 
                                         FunctionalMode));
      }
    }
    else
    {
      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with only address ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode | 
                                         cmd->AddressSize | cmd->AddressMode | cmd->InstructionMode | 
                                         FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(hqspi->Instance->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only data phase ----*/
        if (cmd->DataMode != QSPI_DATA_NONE)
        {
          /* Configure QSPI: CCR register with all communications parameters */
          WRITE_REG(hqspi->Instance->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                           cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode | 
                                           cmd->AddressMode | cmd->InstructionMode | FunctionalMode));
        }
      }
    }
  }
}

HAL_StatusTypeDef HAL_QSPI_Transmit_DMA_infenity(QSPI_HandleTypeDef *hqspi, uint8_t *pData,size_t data_size)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t *tmp;
  
  /* Process locked */
  __HAL_LOCK(hqspi);
  
  if(hqspi->State == HAL_QSPI_STATE_READY)
  {
    /* Clear the error code */                
    hqspi->ErrorCode = HAL_QSPI_ERROR_NONE;
    
    if(pData != NULL ) 
    {
      /* Configure counters of the handle */
      if (hqspi->hdma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE)
      {
        hqspi->TxXferCount = data_size;
      }
      else if (hqspi->hdma->Init.PeriphDataAlignment == DMA_PDATAALIGN_HALFWORD)
      {
        if (((data_size % 2) != 0) || ((hqspi->Init.FifoThreshold % 2) != 0))
        {
          /* The number of data or the fifo threshold is not aligned on halfword 
          => no transfer possible with DMA peripheral access configured as halfword */
          hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
          status = HAL_ERROR; 
          /* Process unlocked */
          __HAL_UNLOCK(hqspi);
        }
        else
        {
          hqspi->TxXferCount = (data_size >> 1);
        }
      }
      else if (hqspi->hdma->Init.PeriphDataAlignment == DMA_PDATAALIGN_WORD)
      {
        if (((data_size % 4) != 0) || ((hqspi->Init.FifoThreshold % 4) != 0))
        {
          /* The number of data or the fifo threshold is not aligned on word 
          => no transfer possible with DMA peripheral access configured as word */
          hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
          status = HAL_ERROR;
          
          /* Process unlocked */
          __HAL_UNLOCK(hqspi);
        }
        else
        {
          hqspi->TxXferCount = (data_size >> 2);
        }
      }
      
      if (status == HAL_OK)
      {

      /* Update state */
      hqspi->State = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

      /* Clear interrupt */
      __HAL_QSPI_CLEAR_FLAG(hqspi, (QSPI_FLAG_TE | QSPI_FLAG_TC));

      /* Configure size and pointer of the handle */
      hqspi->TxXferSize = hqspi->TxXferCount;
      hqspi->pTxBuffPtr = pData;
    
      /* Configure QSPI: CCR register with functional mode as indirect write */
      MODIFY_REG(hqspi->Instance->CCR, QUADSPI_CCR_FMODE, QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);
    
      /* Set the QSPI DMA transfer complete callback */
      hqspi->hdma->XferCpltCallback = QSPI_DMATxCplt;
    
      /* Set the QSPI DMA Half transfer complete callback */
      hqspi->hdma->XferHalfCpltCallback = QSPI_DMATxHalfCplt;
    
      /* Set the DMA error callback */
      hqspi->hdma->XferErrorCallback = QSPI_DMAError;
      
      /* Clear the DMA abort callback */      
      hqspi->hdma->XferAbortCallback = NULL;

      /* Configure the direction of the DMA */
      hqspi->hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
      MODIFY_REG(hqspi->hdma->Instance->CR, DMA_SxCR_DIR, hqspi->hdma->Init.Direction);

      /* Enable the QSPI transmit DMA Channel */
      tmp = (uint32_t*)&pData;
      HAL_DMA_Start_IT(hqspi->hdma, *(uint32_t*)tmp, (uint32_t)&hqspi->Instance->DR, hqspi->TxXferSize);

      /* Process unlocked */
      __HAL_UNLOCK(hqspi);

      /* Enable the QSPI transfer error Interrupt */
      __HAL_QSPI_ENABLE_IT(hqspi, QSPI_IT_TE);

      /* Enable the DMA transfer by setting the DMAEN bit in the QSPI CR register */
      SET_BIT(hqspi->Instance->CR, QUADSPI_CR_DMAEN);
    }
    }
    else
    {
      hqspi->ErrorCode |= HAL_QSPI_ERROR_INVALID_PARAM;
      
      status = HAL_ERROR;

      /* Process unlocked */
      __HAL_UNLOCK(hqspi);
    }
  }
  else
  {
    status = HAL_BUSY;   

    /* Process unlocked */
    __HAL_UNLOCK(hqspi);
  }

  return status;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
