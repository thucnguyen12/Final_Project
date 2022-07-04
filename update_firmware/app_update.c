#include "app_update.h"

typedef union
{
    struct
    {
        char header[3];
        char firmware_version[3];
        char hardware_version[3];
        uint32_t firmware_size;         // fw size =  image_size + 16 byte md5, fw excluded header
        uint8_t release_year;           // From 2000
        uint8_t release_month;
        uint8_t release_date;
    } __attribute__((packed)) name;
    uint8_t raw[16];
} __attribute__((packed)) ota_image_header_t;
char info_string [16];
ota_image_header_t  info_of_bin;
ota_image_header_t* info_of_bin_ptr = &info_of_bin;
uint8_t ringbuffer_data_to_flash [512];
uint8_t buff_data_to_write_to_flash [512];
bool in_flash_process = false;
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
	if (lwrb_get_full (&lwrb_data) > 512)
	{
		lwrb_read (&lwrb_data, buff_data_to_write_to_flash, sizeof (buff_data_to_write_to_flash));
		if (strstr ((char *)buff_data_to_write_to_flash, "start update:")
			&& (in_flash_process == false))
		{
			in_flash_process = true;
			strncpy (info_string, (char *)buff_data_to_write_to_flash, sizeof (info_string));
			info_of_bin_ptr = (ota_image_header_t*)info_string;
			// check info file hoac tinh crc roi nhet vao header
			/* header contain size
			 * header [0] header[1] header[2]
			 * size count by byte
			 * */
			uint32_t size_of_firm = 0;
			size_of_firm |= info_of_bin_ptr->name.header[0] << 16;
			size_of_firm |= info_of_bin_ptr->name.header[1] << 8;
			size_of_firm |= info_of_bin_ptr->name.header[2];

			uint8_t sector_to_erase = (GetSector (OTA_ADDR + size_of_firm) - GetSector (OTA_ADDR)) + 1;
			Flash_Erase (OTA_ADDR, sector_to_erase);
			//send tin hieu ok lai may tinh
//			Flash_Write_Array_32bit (buff_data_to_write_to_flash, OTA_ADDR,);
		}
	}
}
