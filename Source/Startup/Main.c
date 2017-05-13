#include "stm32f7xx.h"

#include "SDMMC.h"
#include "System.h"

#include "ff.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

void FATFS_Test(void const *argument)
{
    UINT bw;
    BYTE data[64] = "Hello World!\r\n";
    BYTE buff[64] = "";
    FATFS *SDCard_Fatfs;
    FIL *fil;
    FRESULT res;

    SDCard_Fatfs = ( FATFS * ) pvPortMalloc( sizeof( FATFS ) );
    fil = ( FIL * ) pvPortMalloc( sizeof( FIL ) );
    res = f_mount(SDCard_Fatfs, "0:", 1);
    if(res == FR_OK)
        res = f_open(fil, "0:hello.txt", FA_WRITE | FA_READ | FA_OPEN_APPEND);
    for(;;)
    {
        vTaskDelay(100);
        if(res == FR_OK)
            res = f_write(fil, data, sizeof(data), &bw);
        if(res == FR_OK)
            res = f_read(fil, buff, 64, &bw);

        f_sync(fil);
    }
}


int main(void)
{

    SystemConfig();

    SDCard.SDReadStatus = xSemaphoreCreateBinary();
    SDCard.SDWriteStatus = xSemaphoreCreateBinary();

    xTaskCreate((TaskFunction_t)FATFS_Test, "FATFS_Test", 640, NULL, 2, NULL);
    vTaskStartScheduler();

    while (1)
    {

    }
}
