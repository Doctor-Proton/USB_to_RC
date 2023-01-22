#include "FreeRTOS.h" /* Must come first. */
//
#include <stdio.h>
//
#include "pico/stdlib.h"
//
#include "ff_headers.h"
#include "ff_sddisk.h"
#include "ff_stdio.h"
#include "ff_utils.h"
//
#include "hw_config.h"

#include "sdcard.h"

bool sd_mounted=false;
FF_Disk_t *pxDisk;

int sd_card_init(void)
{
    pxDisk = FF_SDDiskInit(SD_CARD_PATH);
    configASSERT(pxDisk);
    FF_Error_t xError = FF_SDDiskMount(pxDisk);
    if (FF_isERR(xError) != pdFALSE) {
        FF_PRINTF("FF_SDDiskMount: %s\n",
                  (const char *)FF_GetErrMessage(xError));
        //stop();
    }
    FF_FS_Add(MOUNT_POINT, pxDisk);
    sd_mounted=true;
}

void sd_unmount(void)
{
if(sd_mounted==true)
    {
    FF_FS_Remove(MOUNT_POINT);
    FF_Unmount(pxDisk);
    FF_SDDiskDelete(pxDisk);
    printf("sd card unmounted\r\n");
    sd_mounted=false;            
    }
}

bool is_sd_mounted(void)
{
return sd_mounted;
}