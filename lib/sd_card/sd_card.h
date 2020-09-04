#ifndef __SD_CARD_h
#define __SD_CARD_h

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
// #include "sdkconfig.h"

// #ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
// #endif

#ifdef __cplusplus
extern "C"
{
#endif
    //--------------------------------------------------------------------------------------------
    // Function declarations
    void write_a_file(void);
    void task_write_file(void *ignore);

#ifdef __cplusplus
}
#endif

#endif /* */