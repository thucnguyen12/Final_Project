#include "app_update.h"

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
static const char *update_file = "jig_update.bin";
uint8_t ringbuffer_data_to_flash [528];
uint8_t buff_data_to_write_to_flash [512];
bool in_flash_process = false;
bool start_flash_data = false;
//bool need_update = false;
uint32_t word_wrote = 0;
uint32_t read_crc = 0;
uint32_t calculated_crc = 0;
#if 0
static lwrb_t lwrb_data =
		{
				.buff = NULL,
				.r = 0,
				.w = 0
		};

void poll_data_to_process_flash (uint8_t* buff, uint32_t len)
{
	uint32_t size_of_firm = 0;
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

	if (strstr ((char *)ringbuffer_data_to_flash, "start Upload\r\n")
				&& (in_flash_process == false))
	{
		in_flash_process = true;
		DEBUG_RAW ("OK to Update\r\n");
		memset (buff_data_to_write_to_flash, 0, sizeof(buff_data_to_write_to_flash));
	}
	//ensure that will found the string before receive
	if (lwrb_get_full (&lwrb_data) > 512)
	{
		lwrb_read (&lwrb_data, buff_data_to_write_to_flash, sizeof (buff_data_to_write_to_flash));
		if (in_flash_process)
		{
			if (start_flash_data == false)
			{
				strncpy (info_string, (char *)buff_data_to_write_to_flash, sizeof (info_string));
				info_of_bin_ptr = (ota_image_header_t*)info_string;
				//check if need update
				size_of_firm = info_of_bin_ptr->name.firmware_size - 4; //minus 4 byte sum crc
				Flash_Erase (SIZE_OF_FIRM_ADDR, 1); //erase sector
				Flash_Write_Uin32t (size_of_firm, SIZE_OF_FIRM_ADDR); //write size of firm to flash
				uint8_t sector_to_erase = (GetSector (OTA_ADDR + size_of_firm) - GetSector (OTA_ADDR)) + 1;
				Flash_Erase (OTA_ADDR, sector_to_erase);
				for (uint16_t i = 0; i < 496; i++)
				{
					calculated_crc += (uint32_t) buff_data_to_write_to_flash [i];
				}
				if (Flash_Write_Array_32bit ((uint32_t*)buff_data_to_write_to_flash, OTA_ADDR, 124))
				{//flash 124 word because 1 word contain information
					word_wrote += 124;
					memset (buff_data_to_write_to_flash, 0, sizeof(buff_data_to_write_to_flash));
					DEBUG_RAW ("Read OK\r\n");
				}
				else
				{
					DEBUG_RAW ("Write Fail\r\n");
				}
			}
			else
			{

				uint32_t end_of_file_arr [2]; //contain the rest of firm ware and crc value
				if (word_wrote < (size_of_firm / 4))
				{
					for (uint16_t i = 0; i < 512; i++)
					{
						calculated_crc += (uint32_t) buff_data_to_write_to_flash [i];
					}
					if (Flash_Write_Array_32bit ((uint32_t*)buff_data_to_write_to_flash, OTA_ADDR + word_wrote * 4, 128))
					{
						word_wrote += 128;
						memset (buff_data_to_write_to_flash, 0, sizeof(buff_data_to_write_to_flash));
						DEBUG_RAW ("Read OK\r\n");
					}
					else
					{
						DEBUG_RAW ("Write Fail\r\n");
					}
				}
				else
				{
					// would reach here  when word_wrote == size_of_firm / 4
					if (Flash_Write_Array_32bit ((uint32_t*)buff_data_to_write_to_flash, OTA_ADDR + word_wrote * 4, 2))
					{
						DEBUG_RAW ("Read OK\r\n");
					}
					else
					{
						DEBUG_RAW ("Write Fail\r\n");
					}
					if ((size_of_firm % 4) == 1)
					{
						for (uint16_t i = 0; i < 4; i++)
						{
							calculated_crc += (uint32_t) buff_data_to_write_to_flash [i];
						}
						Flash_Read_Array_32bit (end_of_file_arr, OTA_ADDR + word_wrote * 4, 2);
						read_crc |= end_of_file_arr [0] << 8;
						read_crc |= end_of_file_arr [1] >> 24;
					}
					else if ((size_of_firm % 4) == 2)
					{
						for (uint16_t i = 0; i < 8; i++)
						{
							calculated_crc += (uint32_t) buff_data_to_write_to_flash [i];
						}
						Flash_Read_Array_32bit (end_of_file_arr, OTA_ADDR + word_wrote * 4, 2);
						read_crc |= end_of_file_arr [0] << 16;
						read_crc |= end_of_file_arr [1] >> 16;
					}
					else if ((size_of_firm % 4) == 3)
					{
						for (uint16_t i = 0; i < 12; i++)
						{
							calculated_crc += (uint32_t) buff_data_to_write_to_flash [i];
						}
						read_crc |= end_of_file_arr [0] << 24;
						read_crc |= end_of_file_arr [1] >> 8;
					}
					if (read_crc == calculated_crc)
					{
						Flash_Erase (UPDATE_CHECK_ADDR, 1); //erase sector
						Flash_Write_Uin32t (CHECK_UPDATE_VALUE, UPDATE_CHECK_ADDR); //write size of firm to flash
						NVIC_SystemReset();
					}
					else
					{
						DEBUG_RAW ("CRC Error\r\n"); // gui thong tin lai may tinh
						in_flash_process = false;
						start_flash_data = false;
					}

				}

			}

		}
		else
		{
			memset (buff_data_to_write_to_flash, 0, sizeof(buff_data_to_write_to_flash));
		}

	}
}
#endif
void check_version_and_update_firmware(void)
{
	if (fatfs_is_file_or_folder_existed (update_file) != FILE_EXISTED)
	{
		DEBUG_ERROR ("NO FILE UPDATE OR SOMETHING WENT WRONG\r\n");
		return;
	}
	if (fatfs_read_file (update_file, info_buff, 16))
	{
		update_info_ptr = (ota_image_header_t*) info_buff;
	}

	uint32_t size_of_firmware = update_info_ptr->name.firmware_size - 4;
	uint32_t info_current_version = Flash_Read_Uint (INFO_OF_FILE_ADDR);
	uint16_t info_current_version_buff = (uint16_t)info_current_version;
	info_current_version_ptr = (ota_image_header_t*) &info_current_version_buff;
	if (memcmp (info_current_version_ptr->name.firmware_version, update_info_ptr->name.firmware_version, 3) == 0)
	{
		DEBUG_INFO ("VERSION STILL THE SAME, NO NEED UPDATE\r\n");
		return;
	}
	// now need check crc
	uint32_t size_of_update_file = fatfs_get_file_size (update_file);
	uint32_t crc_value_read;
	uint32_t crc_calculate = 0;
	fatfs_read_file_at_pos (update_file, (uint8_t*) &crc_value_read, 4, size_of_update_file - 4);
	uint32_t total_byte_read = 0;
	uint32_t byte_read;
	uint8_t check_data_buffer [1024];
	while (total_byte_read < (size_of_update_file - 16 - 4)) //exclude header and crc
	{
		memset (check_data_buffer, 0, sizeof (check_data_buffer));
		byte_read = fatfs_read_file_at_pos (update_file, check_data_buffer, sizeof (check_data_buffer), 16 + total_byte_read);//skip 16 first byte
		if (byte_read < 1024)
		{
			for (uint16_t i = 0; i < byte_read - 4; i++)
			{
				crc_calculate += check_data_buffer [i];
			}
			total_byte_read += byte_read;
			break;
		}
		for (uint16_t i = 0; i < byte_read; i++)
		{
			crc_calculate += check_data_buffer [i];
		}
		total_byte_read += byte_read;
	}
	if (crc_calculate == crc_value_read)
	{
		DEBUG_INFO ("CRC OK PREPARE TO RESET \r\n");
		NVIC_SystemReset();
	}
	else
	{
		DEBUG_INFO ("CRC check fail, please retry again\r\n");
		return;
	}
}
