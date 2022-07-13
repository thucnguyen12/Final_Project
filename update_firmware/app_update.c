#include "app_update.h"
#include "version.h"
typedef union
{
    struct
    {
        char header[3];
        char firmware_version[3];
        char hardware_version[3];
        uint32_t firmware_size;         // fw size =  image_size + 4/16 byte md5, fw excluded header
        uint8_t release_year;           // From 2000
        uint8_t release_month;
        uint8_t release_date;
    } __attribute__((packed)) name;
    uint8_t raw[16];
} __attribute__((packed)) ota_image_header_t;
uint8_t info_buff [16];
ota_image_header_t  info_of_bin;
//ota_image_header_t* info_of_bin_ptr = &info_of_bin;
ota_image_header_t* info_current_version_ptr;
ota_image_header_t* update_info_ptr;
//static const char *update_file = "0:/jig_update.bin";
uint8_t ringbuffer_data_to_flash [512];
uint8_t buff_data_to_write_to_flash [512];
bool in_flash_process = false;
bool start_flash_data = false;
//bool need_update = false;
uint32_t word_wrote = 0;
uint32_t read_crc = 0;
uint32_t calculated_crc = 0;
#if 1
static lwrb_t lwrb_data =
		{
				.buff = NULL,
				.r = 0,
				.w = 0
		};

void poll_data_to_process_flash (uint8_t* buff, uint32_t len)
{
	if (lwrb_data.buff == NULL)
	{
		lwrb_init (&lwrb_data, ringbuffer_data_to_flash, sizeof (ringbuffer_data_to_flash));
	}

	if (!buff || len == 0)
	{
		return;
	}

	if (lwrb_get_free (&lwrb_data))
	{
		lwrb_write (&lwrb_data, buff, len);
	}
	else
	{
		// this would be expected never reach
		lwrb_read (&lwrb_data, buff_data_to_write_to_flash, sizeof (buff_data_to_write_to_flash));
	}
	// find start update string
	if (strstr((char*)ringbuffer_data_to_flash, "check version"))
	{
		DEBUG_INFO ("Hardware version: %s \r\n", HardwareVersion);
		DEBUG_INFO ("Firmware version: %s \r\n", FirmwareVersion);
	}
}
#endif

void send_version_info(void)
{
	DEBUG_INFO ("Hardware version: %s \r\n", HardwareVersion);
	DEBUG_INFO ("Firmware version: %s \r\n", FirmwareVersion);
}
