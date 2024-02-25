/**
  ******************************************************************************
  * @file    es_wifi_io.c
  * @author  MCD Application Team
  * @brief   This file implements the IO operations to deal with the es-wifi
  *          module. It mainly Inits and Deinits the SPI interface. Send and
  *          receive data over it.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "es_wifi_io.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define MIN(a, b)  ((a) < (b) ? (a) : (b))
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi3;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
                       COM Driver Interface (SPI)
*******************************************************************************/

/**
  * @brief  Initialize the SPI3
  * @param  None
  * @retval 0 if init success, -1 otherwise.
  */
int8_t SPI_WIFI_Init(void)
{
  uint32_t tickstart;
  uint8_t Prompt[6];

  if(HAL_SPI_Init( &hspi3 ) != HAL_OK)
  {
    return -1;
  }

  WIFI_RESET_MODULE();

  WIFI_ENABLE_NSS();

  tickstart = HAL_GetTick();

  for (uint8_t count = 0; count < 6; count += 2)
  {
    if (HAL_SPI_Receive(&hspi3 , &Prompt[count], 1, 100) != HAL_OK)
    {
      WIFI_DISABLE_NSS();
      return -1;
    }
  }

  while (WIFI_IS_CMDDATA_READY())
  {
    if((HAL_GetTick() - tickstart ) > 100)
    {
      WIFI_DISABLE_NSS();
      return -1;
    }
  }

  WIFI_DISABLE_NSS();

  if((Prompt[0] != 0x15) ||(Prompt[1] != 0x15) ||(Prompt[2] != '\r')||
       (Prompt[3] != '\n') ||(Prompt[4] != '>') ||(Prompt[5] != ' '))
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  DeInitialize the SPI
  * @param  None
  * @retval None
  */
int8_t SPI_WIFI_DeInit(void)
{
  HAL_SPI_DeInit( &hspi3 );
  return 0;
}

/**
  * @brief  Receive wifi Data from SPI
  * @param  pdata : pointer to data
  * @param  len : Data length. if == 0 => receive all available data
  * @param  timeout : send timeout in mS
  * @retval Length of received data (payload)
  */
int16_t SPI_WIFI_ReceiveData(uint8_t *pData, uint16_t len, uint32_t timeout)
{
  uint32_t tickstart = HAL_GetTick();
  int16_t length = 0;
  uint8_t tmp[2];

  HAL_SPIEx_FlushRxFifo(&hspi3);

  WIFI_DISABLE_NSS();

  while (!WIFI_IS_CMDDATA_READY())
  {
    if((HAL_GetTick() - tickstart ) > timeout)
    {
      return -1;
    }
  }

  WIFI_ENABLE_NSS();

  while (WIFI_IS_CMDDATA_READY())
  {
    if((length < len) || (!len))
    {
      HAL_SPI_Receive(&hspi3, tmp, 1, timeout) ;
      /* let some time to hardware to change CMDDATA signal */
      if(tmp[1] == 0x15)
      {
       SPI_WIFI_Delay(1);
      }
      /*This the last data */
      if(!WIFI_IS_CMDDATA_READY())
      {
        if(tmp[1] == 0x15)
        {
          // Only 1 byte of data, the other one is padding
          if((tmp[0] != 0x15))
          {
            pData[0] = tmp[0];
            length++;
          }
          break;
        }
      }

      pData[0] = tmp[0];
      pData[1] = tmp[1];
      length += 2;
      pData  += 2;

      if((HAL_GetTick() - tickstart ) > timeout)
      {
        WIFI_DISABLE_NSS();
        return -1;
      }
    }
    else
    {
      break;
    }
  }

  WIFI_DISABLE_NSS();
  return length;
}
/**
  * @brief  Send wifi Data thru SPI
  * @param  pdata : pointer to data
  * @param  len : Data length
  * @param  timeout : send timeout in mS
  * @retval Length of sent data
  */
int16_t SPI_WIFI_SendData(const uint8_t *pdata,  uint16_t len, uint32_t timeout)
{
  uint32_t tickstart = HAL_GetTick();
  uint8_t Padding[2];

  while (!WIFI_IS_CMDDATA_READY())
  {
    if((HAL_GetTick() - tickstart ) > timeout)
    {
      WIFI_DISABLE_NSS();
      return -1;
    }
  }

  WIFI_ENABLE_NSS();
  if (len > 1)
  {
   if( HAL_SPI_Transmit(&hspi3, (uint8_t *)pdata , len/2, timeout) != HAL_OK)
   {
     WIFI_DISABLE_NSS();
     return -1;
   }
  }

  if ( len & 1)
  {
    Padding[0] = pdata[len-1];
    Padding[1] = '\n';

    if( HAL_SPI_Transmit(&hspi3, Padding, 1, timeout) != HAL_OK)
    {
      WIFI_DISABLE_NSS();
      return -1;
    }
  }

  return len;
}

/**
  * @brief  Delay
  * @param  Delay in ms
  * @retval None
  */
void SPI_WIFI_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
