#include "main.h"
#include "flash.h"
#include "lwrb.h"
#include "app_debug.h"
#include "tusb.h"
#include "fatfs.h"
#include <string.h>
#include <stdbool.h>

//#define OTA_ADDR 0x08080000
//#define UPDATE_CHECK_ADDR 0x08008000 //sector 2
//#define CHECK_UPDATE_VALUE 0xAAAAAAAA
//#define SIZE_OF_FIRM_ADDR 0x0800C000 // sector 3
#define INFO_OF_FILE_ADDR 0x08010000

void poll_data_to_process_flash (uint8_t* buff, uint32_t len);
void write_data_to_ota_memory (void);
void check_version_and_update_firmware(void);
