#include "main.h"
#include "flash.h"
#include "lwrb.h"
#include "app_debug.h"
#include "tusb.h"
#include <string.h>
#include <stdbool.h>

#define OTA_ADDR 0x08020000
#define UPDATE_CHECK_ADDR 0x08008000
#define CHECK_UPDATE_VALUE 0xAAAAAAAA
#define SIZE_OF_FIRM_ADDR 0x0800C000
//#define INFO_OF_FILE_ADDR 0x08004000

void poll_data_to_process_flash (uint8_t* buff, uint32_t len);
void write_data_to_ota_memory (void);
void write_info_of_binfile_to_memory (void);
