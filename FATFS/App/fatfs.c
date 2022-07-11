/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "app_debug.h"
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
FRESULT fresult;
#define SEEK_SET 0
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
uint32_t fatfs_read_file(const char *file, uint8_t *data, uint32_t size)
{
    UINT byte_read = 0;

    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("Seek file %s failed\r\n", file);
        f_close(&USERFile);
        goto end;
    }

    fresult = f_read(&USERFile, data, size, &byte_read);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("Read file %s failed %d\r\n", file, fresult);
        f_close(&USERFile);
        goto end;
    }

    f_close(&USERFile);

end:
    return byte_read;
}

int32_t fatfs_read_file_at_pos(const char *file, uint8_t *data, uint32_t size, uint32_t pos)
{
    UINT byte_read = 0;

    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }
    DEBUG_VERBOSE("File %s opened\r\n", file);

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("[0] Seek file %s failed\r\n", file);
        f_close(&USERFile);
        goto end;
    }
    DEBUG_VERBOSE("File %s seek set\r\n", file);
    fresult = f_lseek(&USERFile, pos);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("[1] Seek file %s failed, %d\r\n", file, fresult);
//        HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
        f_close(&USERFile);
        goto end;
    }
    DEBUG_VERBOSE("File %s seek pos\r\n", file);
    fresult = f_read(&USERFile, data, size, &byte_read);
    if (FR_OK != fresult)
    {
//    	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
//    	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
        DEBUG_ERROR("Read file %s failed %d\r\n", file, fresult);
        f_close(&USERFile);
        goto end;
    }
    DEBUG_VERBOSE("File %s read\r\n", file);
    fresult = f_close(&USERFile);
    if (FR_OK != fresult)
       {
     	 DEBUG_INFO ("CLOSE FILE ERROR: %u", fresult);
       }

    DEBUG_VERBOSE("File %s closed\r\n", file);
end:
    return byte_read;
}

int32_t fatfs_get_file_size(const char *file)
{
    int32_t size = -1;

    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
    	if (fresult == 4) size = 0;
        DEBUG_ERROR("[%s] Open file %s failed %d\r\n", __FUNCTION__, file, fresult);
        goto end;
    }

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        f_close(&USERFile);
        DEBUG_ERROR("Seek file %s failed %d\r\n", file, fresult);
        goto end;
    }

    size = f_size(&USERFile);
    f_close(&USERFile);

end:
    return size;
}

void fatfs_delete_a_file (const char * file)
{
	fresult = f_unlink (file);
}

FRESULT fatfs_create_directory (const char * path)
{
	fresult = f_mkdir (path);
	if (fresult != FR_OK)
	{
		DEBUG_ERROR ("ERROR IN CREAT DIR: %d\r\n", fresult);
	}
	return fresult;
}
fatfs_file_info_t fatfs_is_file_or_folder_existed (const char* file)
{
	FRESULT fr;
	FILINFO fno;
	fr = f_stat (file, &fno);
	switch (fr)
	{
	case FR_OK:
		DEBUG_VERBOSE("THERE IS FILE %s\r\n", file);
		return FILE_EXISTED;

	case FR_NO_FILE:
//		DEBUG_INFO ("NO FILE %s\r\n", file);
		return FILE_NOT_EXISTED;
	default:
		DEBUG_ERROR ("AN ERROR OCCURED, %d\r\n", fr);
		return FILE_ERROR;
	}
}

uint32_t fatfs_write_to_a_file_at_pos (const char* file, char* buff, uint32_t size, uint32_t pos)
{
	UINT byte_write = 0;
	fresult = f_open(&USERFile, file, FA_CREATE_ALWAYS);
	if (fresult != FR_OK)
	{
		DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
		goto end;
	}
	f_close(&USERFile);
	fresult = f_open(&USERFile, file, 	 FA_WRITE);
	if (fresult != FR_OK)
	{
		DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
		goto end;
	}

	fresult = f_lseek(&USERFile, (DWORD)pos);
	if (FR_OK != fresult)
	{
		DEBUG_ERROR(" Seek file %s at 0 failed\r\n", file);
		f_close(&USERFile);
		goto end;
	}

	fresult = f_write (&USERFile, buff, size, &byte_write);
	if (fresult != FR_OK)
	{
		DEBUG_INFO ("ERROR %d", fresult);
		DEBUG_ERROR ("WRITE FILE %s FAIL", file);
		f_close(&USERFile);
		goto end;

	}
	f_close(&USERFile);
	end:
	    return byte_write;
}
/* USER CODE END Application */
