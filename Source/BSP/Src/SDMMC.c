#include "SDMMC.h"

BSP_SD_CardInfo SDCard;

/**
  * @brief  Initializes the SD Detect pin MSP.
  * @param  : SD handle
  * @param  None
  * @retval None
  */
__weak void BSP_SD_Detect_MspInit(void)
{
    /* Enable the LED1 Clock */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOI);

    /* Configure IO in output push-pull mode to drive external LED1 */
    LL_GPIO_SetPinMode(GPIOI, LL_GPIO_PIN_15, LL_GPIO_MODE_INPUT);
    /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
    LL_GPIO_SetPinSpeed(GPIOI, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_LOW);
    /* Reset value is LL_GPIO_PULL_NO */
    LL_GPIO_SetPinPull(GPIOI, LL_GPIO_PIN_15, LL_GPIO_PULL_UP);
}

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @retval Returns if SD is detected or not
 */
uint8_t BSP_SD_IsDetected(void)
{
    __IO uint8_t  status = SD_PRESENT;

    /* Check SD card detect pin */
    if (LL_GPIO_IsInputPinSet(GPIOI, LL_GPIO_PIN_15))
    {
        status = SD_NOT_PRESENT;
    }

    return status;
}

static uint32_t SD_FindSCR(uint32_t *pSCR)
{
    SDMMC_DataInitTypeDef config;
    uint32_t errorstate = SDMMC_ERROR_NONE;
    uint32_t index = 0;
    uint32_t tempscr[2] = {0, 0};

    /* Set Block Size To 8 Bytes */
    errorstate = SDMMC_CmdBlockLength(SDMMC2, 8);
    if(errorstate != MSD_OK)
    {
        return errorstate;
    }

    /* Send CMD55 APP_CMD with argument as card's RCA */
    errorstate = SDMMC_CmdAppCommand(SDMMC2, (uint32_t)((SDCard.RelCardAdd) << 16));
    if(errorstate != MSD_OK)
    {
        return errorstate;
    }

    config.DataTimeOut   = SDMMC_DATATIMEOUT;
    config.DataLength    = 8;
    config.DataBlockSize = SDMMC_DATABLOCK_SIZE_8B;
    config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
    config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
    config.DPSM          = SDMMC_DPSM_ENABLE;
    SDMMC_ConfigData(SDMMC2, &config);

    /* Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
    errorstate = SDMMC_CmdSendSCR(SDMMC2);
    if(errorstate != MSD_OK)
    {
        return errorstate;
    }

    while(!(SDMMC2->STA & (SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DBCKEND)))
    {
        if(SDMMC2->STA & SDMMC_FLAG_RXDAVL)
        {
            *(tempscr + index) = SDMMC_ReadFIFO(SDMMC2);
            index++;
        }
    }

    if(SDMMC2->STA & SDMMC_FLAG_DTIMEOUT)
    {
        SDMMC2->ICR = SDMMC_FLAG_DTIMEOUT;

        return BSP_SD_ERROR_DATA_TIMEOUT;
    }
    else if(SDMMC2->STA & SDMMC_FLAG_DCRCFAIL)
    {
        SDMMC2->ICR = SDMMC_FLAG_DCRCFAIL;

        return BSP_SD_ERROR_DATA_CRC_FAIL;
    }
    else if(SDMMC2->STA & SDMMC_FLAG_RXOVERR)
    {
        SDMMC2->ICR = SDMMC_FLAG_RXOVERR;

        return BSP_SD_ERROR_RX_OVERRUN;
    }
    else
    {
        /* No error flag set */
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;

        *(pSCR + 1) = ((tempscr[0] & SDMMC_0TO7BITS) << 24)  | ((tempscr[0] & SDMMC_8TO15BITS) << 8) | \
                      ((tempscr[0] & SDMMC_16TO23BITS) >> 8) | ((tempscr[0] & SDMMC_24TO31BITS) >> 24);

        *(pSCR) = ((tempscr[1] & SDMMC_0TO7BITS) << 24)  | ((tempscr[1] & SDMMC_8TO15BITS) << 8) | \
                  ((tempscr[1] & SDMMC_16TO23BITS) >> 8) | ((tempscr[1] & SDMMC_24TO31BITS) >> 24);
    }

    return SDMMC_ERROR_NONE;
}

static uint32_t SD_WideBus_Enable(void)
{
    uint32_t scr[2] = {0, 0};
    uint32_t errorstate = SDMMC_ERROR_NONE;

    if((SDMMC_GetResponse(SDMMC2, SDMMC_RESP1) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
    {
        return BSP_SD_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Get SCR Register */
    errorstate = SD_FindSCR(scr);
    if(errorstate != MSD_OK)
    {
        return errorstate;
    }

    /* If requested card supports wide bus operation */
    if((scr[1] & SDMMC_WIDE_BUS_SUPPORT) != SDMMC_ALLZERO)
    {
        /* Send CMD55 APP_CMD with argument as card's RCA.*/
        errorstate = SDMMC_CmdAppCommand(SDMMC2, (uint32_t)(SDCard.RelCardAdd << 16));
        if(errorstate != MSD_OK)
        {
            return errorstate;
        }

        /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
        errorstate = SDMMC_CmdBusWidth(SDMMC2, 2);
        if(errorstate != MSD_OK)
        {
            return errorstate;
        }

        return SDMMC_ERROR_NONE;
    }
    else
    {
        return BSP_SD_ERROR_REQUEST_NOT_APPLICABLE;
    }
}

/**
  * @brief  Initializes the SD MSP.
  * @param  : SD handle
  * @param  Params : None
  */
__weak void BSP_SD_MspInit(void)
{

    /* Enable SDMMC2 clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SDMMC2);

    /* Enable DMA2 clocks */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    /* Enable GPIOs clock */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);

    /* GPIOB configuration */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_10);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_10);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOD, LL_GPIO_PIN_6, LL_GPIO_AF_11);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

    LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOD, LL_GPIO_PIN_7, LL_GPIO_AF_11);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOD, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);

    /* GPIOG configuration */
    LL_GPIO_SetPinMode(GPIOG, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOG, LL_GPIO_PIN_9, LL_GPIO_AF_11);
    LL_GPIO_SetPinSpeed(GPIOG, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOG, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOG, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

    LL_GPIO_SetPinMode(GPIOG, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOG, LL_GPIO_PIN_10, LL_GPIO_AF_11);
    LL_GPIO_SetPinSpeed(GPIOG, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOG, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOG, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

    /* NVIC configuration for SDMMC2 interrupts */
    NVIC_SetPriority(SDMMC2_IRQn, 0x0A);
    NVIC_EnableIRQ(SDMMC2_IRQn);

    /* Configure Function */

    LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                          LL_DMA_PRIORITY_VERYHIGH              |
                          LL_DMA_MODE_PFCTRL                |
                          LL_DMA_PERIPH_NOINCREMENT           |
                          LL_DMA_MEMORY_INCREMENT           |
                          LL_DMA_PDATAALIGN_WORD            |
                          LL_DMA_MDATAALIGN_WORD);

    LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_0, LL_DMA_FIFOTHRESHOLD_FULL);
    LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_0);
    LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_MBURST_INC4);
    LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_0, LL_DMA_PBURST_INC4);
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_11);

    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);


    LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                          LL_DMA_PRIORITY_VERYHIGH              |
                          LL_DMA_MODE_PFCTRL                |
                          LL_DMA_PERIPH_NOINCREMENT           |
                          LL_DMA_MEMORY_INCREMENT           |
                          LL_DMA_PDATAALIGN_WORD            |
                          LL_DMA_MDATAALIGN_WORD);

    LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_5, LL_DMA_FIFOTHRESHOLD_FULL);
    LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_5);
    LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_5, LL_DMA_MBURST_INC4);
    LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_5, LL_DMA_PBURST_INC4);
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_11);

    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
    // LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_5);

    /* NVIC configuration for DMA transfer complete interrupt */
    NVIC_SetPriority(DMA2_Stream0_IRQn, 0x0A);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt */
    NVIC_SetPriority(DMA2_Stream5_IRQn, 0x0A);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

static uint32_t SD_PowerON(void)
{
    __IO uint32_t count = 0;
    uint32_t response = 0, validvoltage = 0;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* CMD0: GO_IDLE_STATE */
    errorstate = SDMMC_CmdGoIdleState(SDMMC2);
    if(errorstate != SDMMC_ERROR_NONE)
    {
        return errorstate;
    }

    /* CMD8: SEND_IF_COND: Command available only on V2.0 cards */
    errorstate = SDMMC_CmdOperCond(SDMMC2);
    if(errorstate != SDMMC_ERROR_NONE)
    {
        SDCard.CardVersion = CARD_V1_X;

        /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while(validvoltage == 0)
        {
            if(count++ == SDMMC_MAX_VOLT_TRIAL)
            {
                return BSP_SD_ERROR_INVALID_VOLTRANGE;
            }

            /* SEND CMD55 APP_CMD with RCA as 0 */
            errorstate = SDMMC_CmdAppCommand(SDMMC2, 0);
            if(errorstate != SDMMC_ERROR_NONE)
            {
                return BSP_SD_ERROR_UNSUPPORTED_FEATURE;
            }

            /* Send CMD41 */
            errorstate = SDMMC_CmdAppOperCommand(SDMMC2, SDMMC_STD_CAPACITY);
            if(errorstate != SDMMC_ERROR_NONE)
            {
                return BSP_SD_ERROR_UNSUPPORTED_FEATURE;
            }

            /* Get command response */
            response = SDMMC_GetResponse(SDMMC2, SDMMC_RESP1);

            /* Get operating voltage*/
            validvoltage = (((response >> 31) == 1) ? 1 : 0);
        }
        /* Card type is SDSC */
        SDCard.CardType = CARD_SDSC;
    }
    else
    {
        SDCard.CardVersion = CARD_V2_X;

        /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while(validvoltage == 0)
        {
            if(count++ == SDMMC_MAX_VOLT_TRIAL)
            {
                return BSP_SD_ERROR_INVALID_VOLTRANGE;
            }

            /* SEND CMD55 APP_CMD with RCA as 0 */
            errorstate = SDMMC_CmdAppCommand(SDMMC2, 0);
            if(errorstate != SDMMC_ERROR_NONE)
            {
                return errorstate;
            }

            /* Send CMD41 */
            errorstate = SDMMC_CmdAppOperCommand(SDMMC2, SDMMC_HIGH_CAPACITY);
            if(errorstate != SDMMC_ERROR_NONE)
            {
                return errorstate;
            }

            /* Get command response */
            response = SDMMC_GetResponse(SDMMC2, SDMMC_RESP1);

            /* Get operating voltage*/
            validvoltage = (((response >> 31) == 1) ? 1 : 0);
        }

        if((response & SDMMC_HIGH_CAPACITY) == SDMMC_HIGH_CAPACITY) /* (response &= SD_HIGH_CAPACITY) */
        {
            SDCard.CardType = CARD_SDHC_SDXC;
        }
        else
        {
            SDCard.CardType = CARD_SDSC;
        }
    }

    return SDMMC_ERROR_NONE;
}

void BSP_SD_GetCardCID(uint32_t *CID,HAL_SD_CardCIDTypeDef *pCID)
{
  uint32_t tmp = 0;
  
  /* Byte 0 */
  tmp = (uint8_t)((CID[0] & 0xFF000000U) >> 24);
  pCID->ManufacturerID = tmp;
  
  /* Byte 1 */
  tmp = (uint8_t)((CID[0] & 0x00FF0000) >> 16);
  pCID->OEM_AppliID = tmp << 8;
  
  /* Byte 2 */
  tmp = (uint8_t)((CID[0] & 0x000000FF00) >> 8);
  pCID->OEM_AppliID |= tmp;
  
  /* Byte 3 */
  tmp = (uint8_t)(CID[0] & 0x000000FF);
  pCID->ProdName1 = tmp << 24;
  
  /* Byte 4 */
  tmp = (uint8_t)((CID[1] & 0xFF000000U) >> 24);
  pCID->ProdName1 |= tmp << 16;
  
  /* Byte 5 */
  tmp = (uint8_t)((CID[1] & 0x00FF0000) >> 16);
  pCID->ProdName1 |= tmp << 8;
  
  /* Byte 6 */
  tmp = (uint8_t)((CID[1] & 0x0000FF00) >> 8);
  pCID->ProdName1 |= tmp;
  
  /* Byte 7 */
  tmp = (uint8_t)(CID[1] & 0x000000FF);
  pCID->ProdName2 = tmp;
  
  /* Byte 8 */
  tmp = (uint8_t)((CID[2] & 0xFF000000U) >> 24);
  pCID->ProdRev = tmp;
  
  /* Byte 9 */
  tmp = (uint8_t)((CID[2] & 0x00FF0000) >> 16);
  pCID->ProdSN = tmp << 24;
  
  /* Byte 10 */
  tmp = (uint8_t)((CID[2] & 0x0000FF00) >> 8);
  pCID->ProdSN |= tmp << 16;
  
  /* Byte 11 */
  tmp = (uint8_t)(CID[2] & 0x000000FF);
  pCID->ProdSN |= tmp << 8;
  
  /* Byte 12 */
  tmp = (uint8_t)((CID[3] & 0xFF000000U) >> 24);
  pCID->ProdSN |= tmp;
  
  /* Byte 13 */
  tmp = (uint8_t)((CID[3] & 0x00FF0000) >> 16);
  pCID->Reserved1   |= (tmp & 0xF0) >> 4;
  pCID->ManufactDate = (tmp & 0x0F) << 8;
  
  /* Byte 14 */
  tmp = (uint8_t)((CID[3] & 0x0000FF00) >> 8);
  pCID->ManufactDate |= tmp;
  
  /* Byte 15 */
  tmp = (uint8_t)(CID[3] & 0x000000FF);
  pCID->CID_CRC   = (tmp & 0xFE) >> 1;
  pCID->Reserved2 = 1;
}

void BSP_SD_GetCardCSD(uint32_t *CSD,BSP_SD_CardCSDTypeDef *pCSD)
{
    uint32_t tmp = 0;

    /* Byte 0 */
    tmp = (CSD[0] & 0xFF000000U) >> 24;
    pCSD->CSDStruct      = (uint8_t)((tmp & 0xC0) >> 6);
    pCSD->SysSpecVersion = (uint8_t)((tmp & 0x3C) >> 2);
    pCSD->Reserved1      = tmp & 0x03;

    /* Byte 1 */
    tmp = (CSD[0] & 0x00FF0000) >> 16;
    pCSD->TAAC = (uint8_t)tmp;

    /* Byte 2 */
    tmp = (CSD[0] & 0x0000FF00) >> 8;
    pCSD->NSAC = (uint8_t)tmp;

    /* Byte 3 */
    tmp = CSD[0] & 0x000000FF;
    pCSD->MaxBusClkFrec = (uint8_t)tmp;

    /* Byte 4 */
    tmp = (CSD[1] & 0xFF000000U) >> 24;
    pCSD->CardComdClasses = (uint16_t)(tmp << 4);

    /* Byte 5 */
    tmp = (CSD[1] & 0x00FF0000U) >> 16;
    pCSD->CardComdClasses |= (uint16_t)((tmp & 0xF0) >> 4);
    pCSD->RdBlockLen       = (uint8_t)(tmp & 0x0F);

    /* Byte 6 */
    tmp = (CSD[1] & 0x0000FF00U) >> 8;
    pCSD->PartBlockRead   = (uint8_t)((tmp & 0x80) >> 7);
    pCSD->WrBlockMisalign = (uint8_t)((tmp & 0x40) >> 6);
    pCSD->RdBlockMisalign = (uint8_t)((tmp & 0x20) >> 5);
    pCSD->DSRImpl         = (uint8_t)((tmp & 0x10) >> 4);
    pCSD->Reserved2       = 0; /*!< Reserved */

    if(SDCard.CardType == CARD_SDSC)
    {
        pCSD->DeviceSize = (tmp & 0x03) << 10;

        /* Byte 7 */
        tmp = (uint8_t)(CSD[1] & 0x000000FFU);
        pCSD->DeviceSize |= (tmp) << 2;

        /* Byte 8 */
        tmp = (uint8_t)((CSD[2] & 0xFF000000U) >> 24);
        pCSD->DeviceSize |= (tmp & 0xC0) >> 6;

        pCSD->MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        pCSD->MaxRdCurrentVDDMax = (tmp & 0x07);

        /* Byte 9 */
        tmp = (uint8_t)((CSD[2] & 0x00FF0000U) >> 16);
        pCSD->MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        pCSD->MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        pCSD->DeviceSizeMul      = (tmp & 0x03) << 1;
        /* Byte 10 */
        tmp = (uint8_t)((CSD[2] & 0x0000FF00U) >> 8);
        pCSD->DeviceSizeMul |= (tmp & 0x80) >> 7;

        SDCard.BlockNbr  = (pCSD->DeviceSize + 1) ;
        SDCard.BlockNbr *= (1 << (pCSD->DeviceSizeMul + 2));
        SDCard.BlockSize = 1 << (pCSD->RdBlockLen);

        SDCard.LogBlockNbr =  (SDCard.BlockNbr) * ((SDCard.BlockSize) / 512);
        SDCard.LogBlockSize = 512;
    }
    else if(SDCard.CardType == CARD_SDHC_SDXC)
    {
        /* Byte 7 */
        tmp = (uint8_t)(CSD[1] & 0x000000FFU);
        pCSD->DeviceSize = (tmp & 0x3F) << 16;

        /* Byte 8 */
        tmp = (uint8_t)((CSD[2] & 0xFF000000U) >> 24);

        pCSD->DeviceSize |= (tmp << 8);

        /* Byte 9 */
        tmp = (uint8_t)((CSD[2] & 0x00FF0000U) >> 16);

        pCSD->DeviceSize |= (tmp);

        /* Byte 10 */
        tmp = (uint8_t)((CSD[2] & 0x0000FF00U) >> 8);

        SDCard.LogBlockNbr = SDCard.BlockNbr = (((uint64_t)pCSD->DeviceSize + 1) * 1024);
        SDCard.LogBlockSize = SDCard.BlockSize = 512;
    }

    pCSD->EraseGrSize = (tmp & 0x40) >> 6;
    pCSD->EraseGrMul  = (tmp & 0x3F) << 1;

    /* Byte 11 */
    tmp = (uint8_t)(CSD[2] & 0x000000FF);
    pCSD->EraseGrMul     |= (tmp & 0x80) >> 7;
    pCSD->WrProtectGrSize = (tmp & 0x7F);

    /* Byte 12 */
    tmp = (uint8_t)((CSD[3] & 0xFF000000U) >> 24);
    pCSD->WrProtectGrEnable = (tmp & 0x80) >> 7;
    pCSD->ManDeflECC        = (tmp & 0x60) >> 5;
    pCSD->WrSpeedFact       = (tmp & 0x1C) >> 2;
    pCSD->MaxWrBlockLen     = (tmp & 0x03) << 2;

    /* Byte 13 */
    tmp = (uint8_t)((CSD[3] & 0x00FF0000) >> 16);
    pCSD->MaxWrBlockLen      |= (tmp & 0xC0) >> 6;
    pCSD->WriteBlockPaPartial = (tmp & 0x20) >> 5;
    pCSD->Reserved3           = 0;
    pCSD->ContentProtectAppli = (tmp & 0x01);

    /* Byte 14 */
    tmp = (uint8_t)((CSD[3] & 0x0000FF00) >> 8);
    pCSD->FileFormatGrouop = (tmp & 0x80) >> 7;
    pCSD->CopyFlag         = (tmp & 0x40) >> 6;
    pCSD->PermWrProtect    = (tmp & 0x20) >> 5;
    pCSD->TempWrProtect    = (tmp & 0x10) >> 4;
    pCSD->FileFormat       = (tmp & 0x0C) >> 2;
    pCSD->ECC              = (tmp & 0x03);

    /* Byte 15 */
    tmp = (uint8_t)(CSD[3] & 0x000000FF);
    pCSD->CSD_CRC   = (tmp & 0xFE) >> 1;
    pCSD->Reserved4 = 1;

}

static uint32_t SD_InitCard(void)
{
    SDMMC_InitTypeDef Init;
    uint32_t CSD[4];
    uint32_t CID[4];	
    uint32_t errorstate = SDMMC_ERROR_NONE;
    uint16_t sd_rca = 1;

    /* Check the power State */
    if(SDMMC_GetPowerState(SDMMC2) == 0)
    {
        /* Power off */
        return BSP_SD_ERROR_REQUEST_NOT_APPLICABLE;
    }

    if(SDCard.CardType != CARD_SECURED)
    {
        /* Send CMD2 ALL_SEND_CID */
        errorstate = SDMMC_CmdSendCID(SDMMC2);
        if(errorstate != SDMMC_ERROR_NONE)
        {
            return errorstate;
        }
        else
        {
            /* Get Card identification number data */
            CID[0] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP1);
            CID[1] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP2);
            CID[2] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP3);
            CID[3] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP4);
        }
    }

		
		BSP_SD_GetCardCID(CID,&SDCard.CID);
		
    if(SDCard.CardType != CARD_SECURED)
    {
        /* Send CMD3 SET_REL_ADDR with argument 0 */
        /* SD Card publishes its RCA. */
        errorstate = SDMMC_CmdSetRelAdd(SDMMC2, &sd_rca);
        if(errorstate != SDMMC_ERROR_NONE)
        {
            return errorstate;
        }
    }
    if(SDCard.CardType != CARD_SECURED)
    {
        /* Get the SD card RCA */
        SDCard.RelCardAdd = sd_rca;

        /* Send CMD9 SEND_CSD with argument as card's RCA */
        errorstate = SDMMC_CmdSendCSD(SDMMC2, (uint32_t)(SDCard.RelCardAdd << 16U));
        if(errorstate != SDMMC_ERROR_NONE)
        {
            return errorstate;
        }
        else
        {
            /* Get Card Specific Data */
            CSD[0U] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP1);
            CSD[1U] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP2);
            CSD[2U] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP3);
            CSD[3U] = SDMMC_GetResponse(SDMMC2, SDMMC_RESP4);
        }
    }

    /* Get the Card Class */
    SDCard.Class = (SDMMC_GetResponse(SDMMC2, SDMMC_RESP2) >> 20);

    /* Get CSD parameters */
    BSP_SD_GetCardCSD(CSD,&SDCard.CSD);

    /* Select the Card */
    errorstate = SDMMC_CmdSelDesel(SDMMC2, (uint32_t)(((uint32_t)SDCard.RelCardAdd) << 16));
    if(errorstate != SDMMC_ERROR_NONE)
    {
        return errorstate;
    }

    /* Default SDMMC peripheral configuration for SD card initialization */
    Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    Init.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
    Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    Init.BusWide             = SDMMC_BUS_WIDE_1B;
    Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    Init.ClockDiv            = SDMMC_TRANSFER_CLK_DIV;

    /* Configure SDMMC peripheral interface */
    SDMMC_Init(SDMMC2, Init);

    /* All cards are initialized */
    return SDMMC_ERROR_NONE;
}

/**
  * @brief  Initializes the SD Card.
  * @param  Params : None
  * @note   This function initializes the SD card. It could be used when a card
            re-initialization is needed.
  * @retval status
  */
uint32_t SD_HighLevel_Init(void)
{
    uint32_t errorstate = SDMMC_ERROR_NONE;
    SDMMC_InitTypeDef Init;

    /* Default SDMMC peripheral configuration for SD card initialization */
    Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    Init.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
    Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    Init.BusWide             = SDMMC_BUS_WIDE_1B;
    Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    Init.ClockDiv            = SDMMC_INIT_CLK_DIV;

    /* Initialize SDMMC peripheral interface with default configuration */
    SDMMC_Init(SDMMC2, Init);

    /* Disable SDMMC Clock */
    SDMMC2->CLKCR &= ~SDMMC_CLKCR_CLKEN;

    /* Set Power State to ON */
    SDMMC2->POWER = SDMMC_POWER_PWRCTRL;

    /* Enable SDMMC Clock */
    SDMMC2->CLKCR |= SDMMC_CLKCR_CLKEN;

    /* Required power up waiting time before starting the SD initialization sequence */
    //BSP_Delay(2);

    /* Identify card operating voltage */
    errorstate = SD_PowerON();
    if(errorstate != SDMMC_ERROR_NONE)
    {
        return !SDMMC_ERROR_NONE;
    }

    /* Card initialization */
    errorstate = SD_InitCard();
    if(errorstate != SDMMC_ERROR_NONE)
    {
        return !SDMMC_ERROR_NONE;
    }

    return SDMMC_ERROR_NONE;
}

static uint32_t SD_WideBus_Disable(void)
{
    uint32_t scr[2] = {0, 0};
    uint32_t errorstate = SDMMC_ERROR_NONE;

    if((SDMMC_GetResponse(SDMMC2, SDMMC_RESP1) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
    {
        return BSP_SD_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Get SCR Register */
    errorstate = SD_FindSCR(scr);
    if(errorstate != MSD_OK)
    {
        return errorstate;
    }

    /* If requested card supports 1 bit mode operation */
    if((scr[1] & SDMMC_SINGLE_BUS_SUPPORT) != SDMMC_ALLZERO)
    {
        /* Send CMD55 APP_CMD with argument as card's RCA */
        errorstate = SDMMC_CmdAppCommand(SDMMC2, (uint32_t)(SDCard.RelCardAdd << 16));
        if(errorstate != MSD_OK)
        {
            return errorstate;
        }

        /* Send ACMD6 APP_CMD with argument as 0 for single bus mode */
        errorstate = SDMMC_CmdBusWidth(SDMMC2, 0);
        if(errorstate != MSD_OK)
        {
            return errorstate;
        }

        return SDMMC_ERROR_NONE;
    }
    else
    {
        return BSP_SD_ERROR_REQUEST_NOT_APPLICABLE;
    }
}


/**
  * @brief  Enables wide bus operation for the requested card if supported by
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SDMMC_BUS_WIDE_8B: 8-bit data transfer
  *            @arg SDMMC_BUS_WIDE_4B: 4-bit data transfer
  *            @arg SDMMC_BUS_WIDE_1B: 1-bit data transfer
  * @retval status
  */
uint32_t BSP_SD_ConfigWideBusOperation(uint32_t WideMode)
{
    SDMMC_InitTypeDef Init;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    if(SDCard.CardType != CARD_SECURED)
    {
        if(WideMode == SDMMC_BUS_WIDE_8B)
        {
            return !SDMMC_ERROR_NONE;
        }
        else if(WideMode == SDMMC_BUS_WIDE_4B)
        {
            errorstate = SD_WideBus_Enable();
        }
        else if(WideMode == SDMMC_BUS_WIDE_1B)
        {
            errorstate = SD_WideBus_Disable();
        }
        else
        {
            return !SDMMC_ERROR_NONE;
        }
    }
    else
    {
        return !SDMMC_ERROR_NONE;
    }

    if(errorstate != SDMMC_ERROR_NONE)
    {
        return !SDMMC_ERROR_NONE;
    }
    else
    {
        Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
        Init.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
        Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
        Init.BusWide             = WideMode;
        Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
        Init.ClockDiv            = SDMMC_TRANSFER_CLK_DIV;
        SDMMC_Init(SDMMC2, Init);
    }

    return SDMMC_ERROR_NONE;;
}

uint32_t BSP_SD_Erase(uint32_t BlockStartAdd, uint32_t BlockEndAdd)
{
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Check if the card command class supports erase command */
    if(((SDCard.Class) & SDMMC_CCCC_ERASE) == 0U)
    {
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;
        return MSD_ERROR;
    }

    if((SDMMC_GetResponse(SDMMC2, SDMMC_RESP1) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
    {
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;
        return MSD_ERROR;
    }

    /* Get start and end block for high capacity cards */
    if(SDCard.CardType != CARD_SDHC_SDXC)
    {
        BlockStartAdd *= 512U;
        BlockEndAdd   *= 512U;
    }

    /* According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
    if(SDCard.CardType != CARD_SECURED)
    {
        /* Send CMD32 SD_ERASE_GRP_START with argument as addr  */
        errorstate = SDMMC_CmdSDEraseStartAdd(SDMMC2, BlockStartAdd);
        if(errorstate != SDMMC_ERROR_NONE)
        {
            /* Clear all the static flags */
            SDMMC2->ICR = SDMMC_STATIC_FLAGS;
            return MSD_ERROR;
        }

        /* Send CMD33 SD_ERASE_GRP_END with argument as addr  */
        errorstate = SDMMC_CmdSDEraseEndAdd(SDMMC2, BlockEndAdd);
        if(errorstate != SDMMC_ERROR_NONE)
        {
            /* Clear all the static flags */
            SDMMC2->ICR = SDMMC_STATIC_FLAGS;
            return MSD_ERROR;
        }
    }

    /* Send CMD38 ERASE */
    errorstate = SDMMC_CmdErase(SDMMC2);
    if(errorstate != SDMMC_ERROR_NONE)
    {
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;
        return MSD_ERROR;
    }

    return MSD_OK;

}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t BlockAdd, uint32_t NumOfBlocks)
{
    SDMMC_DataInitTypeDef config;
    uint32_t errorstate = SDMMC_ERROR_NONE;


    /* Initialize data control register */
    SDMMC2->DCTRL = 0U;

    if(SDCard.CardType != CARD_SDHC_SDXC)
    {
        BlockAdd *= 512U;
    }

    /* Set Block Size for Card */
    errorstate = SDMMC_CmdBlockLength(SDMMC2, SDMMC_BLOCKSIZE);
    if(errorstate != SDMMC_ERROR_NONE)
    {
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;
        return MSD_ERROR;
    }

    /* Write Blocks in Polling mode */
    if(NumOfBlocks > 1U)
    {
        SDCard.Context = SD_CONTEXT_WRITE_MULTIPLE_BLOCK | SD_CONTEXT_DMA;
        /* Write Multi Block command */
        errorstate = SDMMC_CmdWriteMultiBlock(SDMMC2, BlockAdd);
    }
    else
    {
        SDCard.Context = (SD_CONTEXT_WRITE_SINGLE_BLOCK | SD_CONTEXT_DMA);
        /* Write Single Block command */
        errorstate = SDMMC_CmdWriteSingleBlock(SDMMC2, BlockAdd);
    }
    if(errorstate != SDMMC_ERROR_NONE)
    {
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;
        return MSD_ERROR;
    }

    /* Enable SDMMC DMA transfer */
    // __HAL_SD_DMA_ENABLE(hsd);

    /* Enable the DMA Channel */
    SDMMC2->DCTRL |= SDMMC_DCTRL_DMAEN;
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, (uint32_t)(SDMMC_BLOCKSIZE * NumOfBlocks) / 4);
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_5, (uint32_t)&SDMMC2->FIFO, (uint32_t)pData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);//LL_DMA_DIRECTION_MEMORY_TO_PERIPH
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);

    /* Configure the SD DPSM (Data Path State Machine) */
    config.DataTimeOut   = SDMMC_DATATIMEOUT;
    config.DataLength    = SDMMC_BLOCKSIZE * NumOfBlocks;
    config.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
    config.TransferDir   = SDMMC_TRANSFER_DIR_TO_CARD;
    config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
    config.DPSM          = SDMMC_DPSM_ENABLE;
    SDMMC_ConfigData(SDMMC2, &config);

    return MSD_OK;
}

uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t BlockAdd, uint32_t NumOfBlocks)
{
    SDMMC_DataInitTypeDef config;
    uint32_t errorstate = SDMMC_ERROR_NONE;
	
    if((BlockAdd + NumOfBlocks) > (SDCard.LogBlockNbr))
    {
        return MSD_ERROR;
    }

    /* Initialize data control register */
    SDMMC2->DCTRL = 0U;

    SDMMC2->DCTRL |= SDMMC_DCTRL_DMAEN;

    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, (uint32_t)(SDMMC_BLOCKSIZE * NumOfBlocks) / 4);

    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, (uint32_t)&SDMMC2->FIFO, (uint32_t)pData, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);


    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

    if(SDCard.CardType != CARD_SDHC_SDXC)
    {
        BlockAdd *= 512U;
    }

    /* Configure the SD DPSM (Data Path State Machine) */
    config.DataTimeOut   = SDMMC_DATATIMEOUT;
    config.DataLength    = SDMMC_BLOCKSIZE * NumOfBlocks;
    config.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
    config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
    config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
    config.DPSM          = SDMMC_DPSM_ENABLE;
    SDMMC_ConfigData(SDMMC2, &config);

    /* Set Block Size for Card */
    errorstate = SDMMC_CmdBlockLength(SDMMC2, SDMMC_BLOCKSIZE);
    if(errorstate != SDMMC_ERROR_NONE)
    {
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;
        return MSD_ERROR;
    }

    /* Read Blocks in DMA mode */
    if(NumOfBlocks > 1U)
    {
        SDCard.Context = (SD_CONTEXT_READ_MULTIPLE_BLOCK | SD_CONTEXT_DMA);
        /* Read Multi Block command */
        errorstate = SDMMC_CmdReadMultiBlock(SDMMC2, BlockAdd);
    }
    else
    {
        SDCard.Context = (SD_CONTEXT_READ_SINGLE_BLOCK | SD_CONTEXT_DMA);

        /* Read Single Block command */
        errorstate = SDMMC_CmdReadSingleBlock(SDMMC2, BlockAdd);
    }
    if(errorstate != SDMMC_ERROR_NONE)
    {
        /* Clear all the static flags */
        SDMMC2->ICR = SDMMC_STATIC_FLAGS;
        return MSD_ERROR;
    }

    return MSD_OK;

}


/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{
    uint8_t sd_state = MSD_OK;

    /* Msp SD Detect pin initialization */
    BSP_SD_Detect_MspInit();
    if(BSP_SD_IsDetected() != SD_PRESENT)   /* Check if SD card is present */
    {
        return MSD_ERROR_SD_NOT_PRESENT;
    }

    /* Msp SD initialization */
    BSP_SD_MspInit();

    /* HL SD initialization */
    if(SD_HighLevel_Init() != SDMMC_ERROR_NONE)
    {
        sd_state = MSD_ERROR;
    }

    /* Configure SD Bus width */
    if(sd_state == MSD_OK)
    {
        /* Enable wide operation */
        if(BSP_SD_ConfigWideBusOperation(SDMMC_BUS_WIDE_4B) != MSD_OK)
        {
            sd_state = MSD_ERROR;
        }
        else
        {
            sd_state = MSD_OK;
        }
    }

    return  sd_state;

}

static uint32_t SD_SendStatus(uint32_t *pCardStatus)
{
    /* Send Status command */
    SDMMC_CmdSendStatus(SDMMC2, (uint32_t)(SDCard.RelCardAdd << 16));

    /* Get SD card status */
    *pCardStatus = SDMMC_GetResponse(SDMMC2, SDMMC_RESP1);

    return SDMMC_ERROR_NONE;
}

BSP_SD_CardStateTypeDef HAL_SD_GetCardState(void)
{
    BSP_SD_CardStateTypeDef cardstate =  BSP_SD_CARD_TRANSFER;
    uint32_t resp1 = 0;

    SD_SendStatus(&resp1);

    cardstate = (BSP_SD_CardStateTypeDef)((resp1 >> 9) & 0x0F);

    return cardstate;
}

uint8_t BSP_SD_GetCardState(void)
{
    return((HAL_SD_GetCardState() == BSP_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}
