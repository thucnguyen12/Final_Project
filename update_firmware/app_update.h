#include "main.h"
#include "flash.h"
#include "lwrb.h"
#include "app_debug.h"
#include "tusb.h"
#include "fatfs.h"
#include <string.h>
#include <stdbool.h>

#define INFO_OF_FILE_ADDR 0x08080000

void poll_data_to_process_flash (uint8_t* buff, uint32_t len);
void write_data_to_ota_memory (void);
void check_version_and_update_firmware(void);
