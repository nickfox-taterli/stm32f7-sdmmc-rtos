#ifndef __STM32F769I_DISCOVERY_SD_H
#define __STM32F769I_DISCOVERY_SD_H

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"

#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_crc.h"
#include "stm32f7xx_ll_dac.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_dma2d.h"
#include "stm32f7xx_ll_i2c.h"
#include "stm32f7xx_ll_iwdg.h"
#include "stm32f7xx_ll_rtc.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_usart.h"
#include "stm32f7xx_ll_wwdg.h"
#include "stm32f7xx_ll_sdmmc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"


typedef struct
{
		__IO uint8_t  CSDStruct;            /*!< CSD structure                         */
		__IO uint8_t  SysSpecVersion;       /*!< System specification version          */
		__IO uint8_t  Reserved1;            /*!< Reserved                              */
		__IO uint8_t  TAAC;                 /*!< Data read access time 1               */
		__IO uint8_t  NSAC;                 /*!< Data read access time 2 in CLK cycles */
		__IO uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency              */
		__IO uint16_t CardComdClasses;      /*!< Card command classes                  */
		__IO uint8_t  RdBlockLen;           /*!< Max. read data block length           */
		__IO uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed       */
		__IO uint8_t  WrBlockMisalign;      /*!< Write block misalignment              */
		__IO uint8_t  RdBlockMisalign;      /*!< Read block misalignment               */
		__IO uint8_t  DSRImpl;              /*!< DSR implemented                       */
		__IO uint8_t  Reserved2;            /*!< Reserved                              */
		__IO uint32_t DeviceSize;           /*!< Device Size                           */
		__IO uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min           */
		__IO uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max           */
		__IO uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min          */
		__IO uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max          */
		__IO uint8_t  DeviceSizeMul;        /*!< Device size multiplier                */
		__IO uint8_t  EraseGrSize;          /*!< Erase group size                      */
		__IO uint8_t  EraseGrMul;           /*!< Erase group size multiplier           */
		__IO uint8_t  WrProtectGrSize;      /*!< Write protect group size              */
		__IO uint8_t  WrProtectGrEnable;    /*!< Write protect group enable            */
		__IO uint8_t  ManDeflECC;           /*!< Manufacturer default ECC              */
		__IO uint8_t  WrSpeedFact;          /*!< Write speed factor                    */
		__IO uint8_t  MaxWrBlockLen;        /*!< Max. write data block length          */
		__IO uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed      */
		__IO uint8_t  Reserved3;            /*!< Reserved                              */
		__IO uint8_t  ContentProtectAppli;  /*!< Content protection application        */
		__IO uint8_t  FileFormatGrouop;     /*!< File format group                     */
		__IO uint8_t  CopyFlag;             /*!< Copy flag (OTP)                       */
		__IO uint8_t  PermWrProtect;        /*!< Permanent write protection            */
		__IO uint8_t  TempWrProtect;        /*!< Temporary write protection            */
		__IO uint8_t  FileFormat;           /*!< File format                           */
		__IO uint8_t  ECC;                  /*!< ECC code                              */
		__IO uint8_t  CSD_CRC;              /*!< CSD CRC                               */
		__IO uint8_t  Reserved4;            /*!< Always 1                              */

} BSP_SD_CardCSDTypeDef;

typedef struct
{
  __IO uint8_t  ManufacturerID;  /*!< Manufacturer ID       */
  __IO uint16_t OEM_AppliID;     /*!< OEM/Application ID    */
  __IO uint32_t ProdName1;       /*!< Product Name part1    */
  __IO uint8_t  ProdName2;       /*!< Product Name part2    */
  __IO uint8_t  ProdRev;         /*!< Product Revision      */
  __IO uint32_t ProdSN;          /*!< Product Serial Number */
  __IO uint8_t  Reserved1;       /*!< Reserved1             */
  __IO uint16_t ManufactDate;    /*!< Manufacturing Date    */
  __IO uint8_t  CID_CRC;         /*!< CID CRC               */
  __IO uint8_t  Reserved2;       /*!< Always 1              */

}HAL_SD_CardCIDTypeDef;

/**
* @brief  SD Card Information Structure definition
*/
typedef struct
{
		uint32_t CardType;                     /*!< Specifies the card Type                         */

		uint32_t CardVersion;                  /*!< Specifies the card version                      */

		uint32_t Class;                        /*!< Specifies the class of the card class           */

		uint32_t RelCardAdd;                   /*!< Specifies the Relative Card Address             */

		uint32_t BlockNbr;                     /*!< Specifies the Card Capacity in blocks           */

		uint32_t BlockSize;                    /*!< Specifies one block size in bytes               */

		uint32_t LogBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */

		uint32_t LogBlockSize;                 /*!< Specifies logical block size in bytes           */

		BSP_SD_CardCSDTypeDef                     CSD;           /*!< SD card specific data table         */

		HAL_SD_CardCIDTypeDef                     CID;           /*!< SD card identification number table */

		__IO uint32_t                 Context;          /*!< SD transfer context                 */

		__IO SemaphoreHandle_t                SDWriteStatus;

		__IO SemaphoreHandle_t                SDReadStatus;
} BSP_SD_CardInfo;

typedef enum
{
		BSP_SD_CARD_READY                  = ((uint32_t)0x00000001U),  /*!< Card state is ready                     */
		BSP_SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002U),  /*!< Card is in identification state         */
		BSP_SD_CARD_STANDBY                = ((uint32_t)0x00000003U),  /*!< Card is in standby state                */
		BSP_SD_CARD_TRANSFER               = ((uint32_t)0x00000004U),  /*!< Card is in transfer state               */
		BSP_SD_CARD_SENDING                = ((uint32_t)0x00000005U),  /*!< Card is sending an operation            */
		BSP_SD_CARD_RECEIVING              = ((uint32_t)0x00000006U),  /*!< Card is receiving operation information */
		BSP_SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007U),  /*!< Card is in programming state            */
		BSP_SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008U),  /*!< Card is disconnected                    */
		BSP_SD_CARD_ERROR                  = ((uint32_t)0x000000FFU)   /*!< Card response Error                     */
} BSP_SD_CardStateTypeDef;

/** @defgroup SD_Exported_Constansts_Group3 SD Supported Memory Cards
	* @{
	*/
#define CARD_SDSC                  ((uint32_t)0x00000000U)
#define CARD_SDHC_SDXC             ((uint32_t)0x00000001U)
#define CARD_SECURED               ((uint32_t)0x00000003U)

/**
	* @}
	*/

/** @defgroup SD_Exported_Constansts_Group4 SD Supported Version
	* @{
	*/
#define CARD_V1_X                  ((uint32_t)0x00000000U)
#define CARD_V2_X                  ((uint32_t)0x00000001U)

#define   MSD_OK                        ((uint8_t)0x00)
#define   MSD_ERROR                     ((uint8_t)0x01)
#define   MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)

/**
	* @brief  SD transfer state definition
	*/
#define   SD_TRANSFER_OK                ((uint8_t)0x00)
#define   SD_TRANSFER_BUSY              ((uint8_t)0x01)

/** @defgroup STM32F769I_DISCOVERY_SD_Exported_Constants SD Exported Constants
	* @{
	*/
#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

#define SD_DATATIMEOUT           ((uint32_t)100000000)

#define   SD_CONTEXT_NONE                 ((uint32_t)0x00000000U)  /*!< None                             */
#define   SD_CONTEXT_READ_SINGLE_BLOCK    ((uint32_t)0x00000001U)  /*!< Read single block operation      */
#define   SD_CONTEXT_READ_MULTIPLE_BLOCK  ((uint32_t)0x00000002U)  /*!< Read multiple blocks operation   */
#define   SD_CONTEXT_WRITE_SINGLE_BLOCK   ((uint32_t)0x00000010U)  /*!< Write single block operation     */
#define   SD_CONTEXT_WRITE_MULTIPLE_BLOCK ((uint32_t)0x00000020U)  /*!< Write multiple blocks operation  */
#define   SD_CONTEXT_IT                   ((uint32_t)0x00000008U)  /*!< Process in Interrupt mode        */
#define   SD_CONTEXT_DMA                  ((uint32_t)0x00000080U)  /*!< Process in DMA mode              */


#define BSP_SD_ERROR_NONE                     SDMMC_ERROR_NONE                    /*!< No error                                                      */
#define BSP_SD_ERROR_CMD_CRC_FAIL             SDMMC_ERROR_CMD_CRC_FAIL            /*!< Command response received (but CRC check failed)              */
#define BSP_SD_ERROR_DATA_CRC_FAIL            SDMMC_ERROR_DATA_CRC_FAIL           /*!< Data block sent/received (CRC check failed)                   */
#define BSP_SD_ERROR_CMD_RSP_TIMEOUT          SDMMC_ERROR_CMD_RSP_TIMEOUT         /*!< Command response timeout                                      */
#define BSP_SD_ERROR_DATA_TIMEOUT             SDMMC_ERROR_DATA_TIMEOUT            /*!< Data timeout                                                  */
#define BSP_SD_ERROR_TX_UNDERRUN              SDMMC_ERROR_TX_UNDERRUN             /*!< Transmit FIFO underrun                                        */
#define BSP_SD_ERROR_RX_OVERRUN               SDMMC_ERROR_RX_OVERRUN              /*!< Receive FIFO overrun                                          */
#define BSP_SD_ERROR_ADDR_MISALIGNED          SDMMC_ERROR_ADDR_MISALIGNED         /*!< Misaligned address                                            */
#define BSP_SD_ERROR_BLOCK_LEN_ERR            SDMMC_ERROR_BLOCK_LEN_ERR           /*!< Transferred block length is not allowed for the card or the 
number of transferred bytes does not match the block length   */
#define BSP_SD_ERROR_ERASE_SEQ_ERR            SDMMC_ERROR_ERASE_SEQ_ERR           /*!< An error in the sequence of erase command occurs              */
#define BSP_SD_ERROR_BAD_ERASE_PARAM          SDMMC_ERROR_BAD_ERASE_PARAM         /*!< An invalid selection for erase groups                         */
#define BSP_SD_ERROR_WRITE_PROT_VIOLATION     SDMMC_ERROR_WRITE_PROT_VIOLATION    /*!< Attempt to program a write protect block                      */
#define BSP_SD_ERROR_LOCK_UNLOCK_FAILED       SDMMC_ERROR_LOCK_UNLOCK_FAILED      /*!< Sequence or password error has been detected in unlock 
command or if there was an attempt to access a locked card    */
#define BSP_SD_ERROR_COM_CRC_FAILED           SDMMC_ERROR_COM_CRC_FAILED          /*!< CRC check of the previous command failed                      */
#define BSP_SD_ERROR_ILLEGAL_CMD              SDMMC_ERROR_ILLEGAL_CMD             /*!< Command is not legal for the card state                       */
#define BSP_SD_ERROR_CARD_ECC_FAILED          SDMMC_ERROR_CARD_ECC_FAILED         /*!< Card internal ECC was applied but failed to correct the data  */
#define BSP_SD_ERROR_CC_ERR                   SDMMC_ERROR_CC_ERR                  /*!< Internal card controller error                                */
#define BSP_SD_ERROR_GENERAL_UNKNOWN_ERR      SDMMC_ERROR_GENERAL_UNKNOWN_ERR     /*!< General or unknown error                                      */
#define BSP_SD_ERROR_STREAM_READ_UNDERRUN     SDMMC_ERROR_STREAM_READ_UNDERRUN    /*!< The card could not sustain data reading in stream rmode       */
#define BSP_SD_ERROR_STREAM_WRITE_OVERRUN     SDMMC_ERROR_STREAM_WRITE_OVERRUN    /*!< The card could not sustain data programming in stream mode    */
#define BSP_SD_ERROR_CID_CSD_OVERWRITE        SDMMC_ERROR_CID_CSD_OVERWRITE       /*!< CID/CSD overwrite error                                       */
#define BSP_SD_ERROR_WP_ERASE_SKIP            SDMMC_ERROR_WP_ERASE_SKIP           /*!< Only partial address space was erased                         */
#define BSP_SD_ERROR_CARD_ECC_DISABLED        SDMMC_ERROR_CARD_ECC_DISABLED       /*!< Command has been executed without using internal ECC          */
#define BSP_SD_ERROR_ERASE_RESET              SDMMC_ERROR_ERASE_RESET             /*!< Erase sequence was cleared before executing because an out 
of erase sequence command was received                        */
#define BSP_SD_ERROR_AKE_SEQ_ERR              SDMMC_ERROR_AKE_SEQ_ERR             /*!< Error in sequence of authentication                           */
#define BSP_SD_ERROR_INVALID_VOLTRANGE        SDMMC_ERROR_INVALID_VOLTRANGE       /*!< Error in case of invalid voltage range                        */
#define BSP_SD_ERROR_ADDR_OUT_OF_RANGE        SDMMC_ERROR_ADDR_OUT_OF_RANGE       /*!< Error when addressed block is out of range                    */
#define BSP_SD_ERROR_REQUEST_NOT_APPLICABLE   SDMMC_ERROR_REQUEST_NOT_APPLICABLE  /*!< Error when command request is not applicable                  */
#define BSP_SD_ERROR_PARAM                    SDMMC_ERROR_INVALID_PARAMETER       /*!< the used parameter is not valid                               */
#define BSP_SD_ERROR_UNSUPPORTED_FEATURE      SDMMC_ERROR_UNSUPPORTED_FEATURE     /*!< Error when feature is not insupported                         */
#define BSP_SD_ERROR_BUSY                     SDMMC_ERROR_BUSY                    /*!< Error when transfer process is busy                           */
#define BSP_SD_ERROR_DMA                      SDMMC_ERROR_DMA                     /*!< Error while DMA transfer                                      */
#define BSP_SD_ERROR_TIMEOUT                  SDMMC_ERROR_TIMEOUT                 /*!< Timeout error                                                 */

#define SDMMC_BLOCKSIZE   ((uint32_t)512U)

extern BSP_SD_CardInfo SDCard;

uint8_t BSP_SD_Init(void);
uint32_t BSP_SD_Erase(uint32_t BlockStartAdd, uint32_t BlockEndAdd);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t BlockAdd, uint32_t NumOfBlocks);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t BlockAdd, uint32_t NumOfBlocks);
uint8_t BSP_SD_GetCardState(void);
#endif
