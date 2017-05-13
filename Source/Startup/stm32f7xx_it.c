/**
  ******************************************************************************
  * @file    LCD_DSI/LCD_DSI_VideoMode_SingleBuffer/Src/stm32f7xx_it.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
  *      without specific prior written permission.
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
#include "stm32f7xx_it.h"
#include "SDMMC.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup LCD_DSI_VideoMode_SingleBuffer
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}


void SDMMC2_IRQHandler(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    /* Check for SDMMC interrupt flags */
    if(SDMMC2->STA & SDMMC_IT_DATAEND)
    {
        SDMMC2->ICR = SDMMC_FLAG_DATAEND;

        SDMMC2->MASK &= ~SDMMC_IT_DATAEND;


        if((SDCard.Context & SD_CONTEXT_DMA) != RESET)
        {
            if((SDCard.Context & SD_CONTEXT_WRITE_MULTIPLE_BLOCK) != RESET)
            {
                SDMMC_CmdStopTransfer(SDMMC2);
            }
            if(((SDCard.Context & SD_CONTEXT_READ_SINGLE_BLOCK) == RESET) && ((SDCard.Context & SD_CONTEXT_READ_MULTIPLE_BLOCK) == RESET))
            {
                /* Disable the DMA transfer for transmit request by setting the DMAEN bit
                in the SD DCTRL register */
                SDMMC2->DCTRL &= (uint32_t)~((uint32_t)SDMMC_DCTRL_DMAEN);

                //SDCard.SDWriteStatus = 1;
                xSemaphoreGiveFromISR( SDCard.SDWriteStatus, &xHigherPriorityTaskWoken );
                SDCard.Context = SD_CONTEXT_NONE;
            }
        }
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void DMA2_Stream5_IRQHandler(void)
{
    if(LL_DMA_GetChannelSelection(DMA2, LL_DMA_STREAM_5) == LL_DMA_CHANNEL_11)
    {
        if(LL_DMA_IsEnabledIT_TC(DMA2, LL_DMA_STREAM_5) != RESET)
        {
            if (LL_DMA_IsActiveFlag_TC5(DMA2))
            {

                LL_DMA_ClearFlag_TC5(DMA2);
                LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_5);
                /* Transfer complete callback */
                SDMMC2->MASK |= SDMMC_IT_DATAEND;
            }
        }

    }
}

void DMA2_Stream0_IRQHandler(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    if(LL_DMA_GetChannelSelection(DMA2, LL_DMA_STREAM_0) == LL_DMA_CHANNEL_11)
    {
        if(LL_DMA_IsEnabledIT_TC(DMA2, LL_DMA_STREAM_0) != RESET)
        {
            if (LL_DMA_IsActiveFlag_TC0(DMA2))
            {
                LL_DMA_ClearFlag_TC0(DMA2);
                LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_0);

                /* Send stop command in multiblock write */
                if(SDCard.Context == (SD_CONTEXT_READ_MULTIPLE_BLOCK | SD_CONTEXT_DMA))
                {
                    SDMMC_CmdStopTransfer(SDMMC2);
                }

                /* Disable the DMA transfer for transmit request by setting the DMAEN bit
                in the SD DCTRL register */
                SDMMC2->DCTRL &= (uint32_t)~((uint32_t)SDMMC_DCTRL_DMAEN);

                /* Clear all the static flags */
                SDMMC2->ICR = SDMMC_STATIC_FLAGS;

                //SDCard.SDReadStatus = 1;
                xSemaphoreGiveFromISR( SDCard.SDReadStatus, &xHigherPriorityTaskWoken );

                SDCard.Context = SD_CONTEXT_NONE;

            }
        }

    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );



}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
