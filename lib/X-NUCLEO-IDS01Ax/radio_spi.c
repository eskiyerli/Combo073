/**
******************************************************************************
* @file    radio_spi.c
* @author  Central Labs
* @version V1.0.0
* @date    18-Apr-2018
* @brief   This file provides code for the configuration of the SPI instances.                     
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "radio_spi.h"
/**
* @addtogroup BSP
* @{
*/


/**
* @addtogroup X-NUCLEO-IDS01Ax
* @{
*/


/**
* @defgroup RADIO_SPI_Private_TypesDefinitions       RADIO_SPI Private Types Definitions
* @{
*/

/**
* @}
*/


/**
* @defgroup RADIO_SPI_Private_Defines                RADIO_SPI Private Defines
* @{
*/

/**
* @}
*/


/**
* @defgroup RADIO_SPI_Private_Macros                 RADIO_SPI Private Macros
* @{
*/

/**
* @}
*/


/**
* @defgroup RADIO_SPI_Private_Variables              RADIO_SPI Private Variables
* @{
*/
extern SPI_HandleTypeDef hspi1;
uint32_t SpiTimeout = NUCLEO_SPIx_TIMEOUT_MAX;                         /*<! Value of Timeout when SPI communication fails */
/**
* @}
*/


/**
* @defgroup RADIO_SPI_Private_FunctionPrototypes     RADIO_SPI Private Function Prototypes
* @{
*/
//void RadioSpiInit(void);
//void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi1);
/* void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi1); */
static void SPI_Write(uint8_t Value);
static void SPI_Error(void);
StatusBytes RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes RadioSpiCommandStrobes(uint8_t cCommandCode);
StatusBytes RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

/**
* @}
*/


/**
* @defgroup RADIO_SPI_Private_Functions              RADIO_SPI Private Functions
* @{
*/


/**
* @brief  Initializes SPI HAL.
* @param  None
* @retval None
*/
/*
void RadioSpiInit(void)
{
  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET)
  {
     SPI Config
    hspi1.Instance               = RADIO_SPI;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLED;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    
    //HAL_SPI_MspInit(&hspi1);
    HAL_SPI_Init(&hspi1);
  }
}
*/


/**
* @brief  Initializes SPI MSP.
* @param  SPI_HandleTypeDef* hspi1
* @retval None
*/
/*void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi1)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  if (hspi1->Instance==RADIO_SPI)
  {
    ** Configure the GPIOs **
     Enable GPIO clock
    RADIO_SPI_SCLK_CLK_ENABLE();
    RADIO_SPI_MISO_CLK_ENABLE();
    RADIO_SPI_MOSI_CLOCK_ENABLE();
    
    *SPI1 GPIO Configuration
    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = RADIO_SPI_SCK_AF;
    
    GPIO_InitStruct.Pin = RADIO_SPI_SCK_PIN;
    HAL_GPIO_Init(RADIO_SPI_SCK_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = RADIO_SPI_MISO_AF;
    
    GPIO_InitStruct.Pin = RADIO_SPI_MISO_PIN;
    HAL_GPIO_Init(RADIO_SPI_MISO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = RADIO_SPI_MOSI_PIN;
    HAL_GPIO_Init(RADIO_SPI_MOSI_PORT, &GPIO_InitStruct); 
    
    RADIO_SPI_CS_CLOCK_ENABLE();
    
     Configure SPI pin: CS
    GPIO_InitStruct.Pin = RADIO_SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(RADIO_SPI_CS_PORT, &GPIO_InitStruct);
    
    RADIO_SPI_CLK_ENABLE();
  }
}*/

/**
* @}
*/


/**
* @brief  SPI Write a byte to device
* @param  Value: value to be written
* @retval None
*/
static void SPI_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
  status = HAL_SPI_Transmit(&hspi1, (uint8_t*) &Value, 1, SpiTimeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPI_Error();
  }
}


/**
* @brief  SPI error treatment function
* @param  None
* @retval None
*/
static void SPI_Error(void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hspi1);
  
  /* Re-Initiaize the SPI communication BUS */
  RadioSpiInit();
}


/**
* @brief  Write single or multiple RF Transceivers register
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval StatusBytes
*/
StatusBytes RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t aHeader[2] = {0};
  uint16_t tmpstatus = 0x0000;
  
  StatusBytes *pStatus=(StatusBytes *)&tmpstatus;
  
  /* Built the aHeader bytes */
  aHeader[0] = WRITE_HEADER;
  aHeader[1] = cRegAddress;
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  RadioSpiCSLow();
  
  for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[0], (uint8_t *)&(tmpstatus), 1, SpiTimeout);
  tmpstatus = tmpstatus << 8;  
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  
  /* Writes the registers according to the number of bytes */
  for (int index = 0; index < cNbBytes; index++)
  {
    SPI_Write(pcBuffer[index]);
  }
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
  /* Puts the SPI chip select high to end the transaction */
  RadioSpiCSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *pStatus;
  
}


/**
* @brief  Read single or multiple SPIRIT1 register
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval StatusBytes
*/
StatusBytes RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x00;
  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;
  
  uint8_t aHeader[2] = {0};
  uint8_t dummy = 0xFF;
  
  /* Built the aHeader bytes */
  aHeader[0] = READ_HEADER;
  aHeader[1] = cRegAddress;
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  RadioSpiCSLow();
  
  for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[0], (uint8_t *)&(tmpstatus), 1, SpiTimeout);
  tmpstatus = tmpstatus << 8;  
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  
  for (int index = 0; index < cNbBytes; index++)
  { 
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&dummy, (uint8_t *)&(pcBuffer)[index], 1, SpiTimeout);
  } 
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  RadioSpiCSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *pStatus;
  
}


/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval StatusBytes
*/
StatusBytes RadioSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t aHeader[2] = {0};
  uint16_t tmpstatus = 0x0000;
  
  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;
  
  /* Built the aHeader bytes */
  aHeader[0] = COMMAND_HEADER;
  aHeader[1] = cCommandCode;
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  RadioSpiCSLow();
  
  for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[0], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  tmpstatus = tmpstatus<<8;  
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
  
  /* Puts the SPI chip select high to end the transaction */
  RadioSpiCSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *pStatus;
  
}


/**
* @brief  Write data into TX FIFO
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval StatusBytes
*/
StatusBytes RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;
  
  uint8_t aHeader[2] = {0};
  
  /* Built the aHeader bytes */
  aHeader[0] = WRITE_HEADER;
  aHeader[1] = LINEAR_FIFO_ADDRESS;
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  RadioSpiCSLow();
  
  for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[0], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  tmpstatus = tmpstatus<<8;  
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  
  /* Writes the registers according to the number of bytes */
  for (int index = 0; index < cNbBytes; index++)
  {
    SPI_Write(pcBuffer[index]);
  }
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  RadioSpiCSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *pStatus; 
}

/**
* @brief  Read data from RX FIFO
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval StatusBytes
*/
StatusBytes RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  StatusBytes *pStatus = (StatusBytes *)&tmpstatus;
  
  uint8_t aHeader[2];
  uint8_t dummy=0xFF;
  
  /* Built the aHeader bytes */
  aHeader[0]=READ_HEADER;
  aHeader[1]=LINEAR_FIFO_ADDRESS;
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  RadioSpiCSLow();
  
  for (volatile uint16_t Index = 0; Index < CS_TO_SCLK_DELAY; Index++);
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[0], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  tmpstatus = tmpstatus<<8;  
  
  /* Write the aHeader bytes and read the SPIRIT1 status bytes */
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&aHeader[1], (uint8_t *)&tmpstatus, 1, SpiTimeout);
  
  for (int index = 0; index < cNbBytes; index++)
  { 
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&dummy, (uint8_t *)&pcBuffer[index], 1, SpiTimeout);
  } 
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  RadioSpiCSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *pStatus;  
}


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
