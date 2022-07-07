/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include <stdbool.h>
#include <string.h>
#include "app_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ADDR 0x08040000 //sector 6
//#define OTA_ADDR 0x08080000 //sector 8
//#define UPDATE_CHECK_ADDR 0x08008000 //sector 2
//#define CHECK_UPDATE_VALUE 0xAAAAAAAA
//#define SIZE_OF_FIRM_ADDR 0x0800C000 // sector 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t check_update_value;
static const char *update_file = "jig_update.bin";
//ota_image_header_t update_info;
ota_image_header_t* update_info_ptr;
uint8_t info_buff[16];
/*************************** 	  FLASH DISK VARIABLE        *************************************************************/
BYTE gFSWork[_MAX_SS];
UINT br, bw;   // File read/write count
UINT fbr, fbw; // File read/write count
FRESULT flash_res;
bool m_disk_is_mounted = false;
/***********************************************************************************************************************/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef void (*jump_func)(void);
jump_func jump_to_app;
void update_firmware (void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
#if 0
  //test flash
  uint32_t test_temp[12] = {1,2,3,4,5,6,7,8,9,1,2,3};
  uint32_t word_wrote = 0;
  uint32_t word_to_write = 12;
  uint32_t word_temp[3];
  Flash_Erase (UPDATE_CHECK_ADDR, 1);
  Flash_Erase (APP_ADDR, 1);
  Flash_Write_Array_32bit (test_temp, UPDATE_CHECK_ADDR, 12);
	while (word_wrote < word_to_write)
	{
		Flash_Read_Array_32bit (word_temp, UPDATE_CHECK_ADDR + word_wrote * 4, 3);
		if (Flash_Write_Array_32bit (word_temp, APP_ADDR + word_wrote * 4, 3) != 1)
		{
			Error_Handler();
		}
		word_wrote += 3;
	}
#endif
	/// mount flash to use fs
	flash_res = f_mount(&USERFatFS, USERPath, 1);
	if (flash_res != FR_OK)
	{
		DEBUG_WARN("Mount flash fail\r\n");
//		flash_res = f_mkfs(USERPath, 1, sizeof gFSWork);
		flash_res = f_mount(&USERFatFS, USERPath, 1);
		if (flash_res == FR_OK)
		{
			m_disk_is_mounted = true;
			DEBUG_INFO("format disk and mount again\r\n");
		}
		else
		{
			DEBUG_ERROR("Mount flash error\r\n");
		}
	}
	else
	{
		m_disk_is_mounted = true;
		DEBUG_INFO("Mount flash ok\r\n");
	}
	// Set label
	TCHAR label[32];
	f_getlabel(USERPath, label, 0);
	if (strcmp(label, "BSAFE JIG"))
	{
		DEBUG_INFO("Set label\r\n");
		f_setlabel("BSAFE JIG");
	}


	//Check firmware update
//	check_update_value = Flash_Read_Uint (UPDATE_CHECK_ADDR);
//	if (check_update_value == CHECK_UPDATE_VALUE)
	if (fatfs_is_file_or_folder_existed (update_file) == FILE_EXISTED
		&& m_disk_is_mounted)
	{
	  update_firmware ();
	  flash_res = f_mount(NULL, USERPath, 1);//unmount before go to app
	  HAL_RCC_DeInit();
	  SysTick->CTRL = 0;
	  SysTick->LOAD = 0;
	  SysTick->VAL = 0;
	  SYSCFG->MEMRMP = 0x01; //choose the flash memory
	  HAL_DeInit();
	  SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk |\
	  SCB_SHCSR_BUSFAULTENA_Msk | \
	  SCB_SHCSR_MEMFAULTENA_Msk ) ;
	  __disable_irq();
	  jump_to_app = (jump_func)(*(volatile unsigned int *)(APP_ADDR + 4));
	  __set_MSP(*(volatile unsigned int*)APP_ADDR);
	  jump_to_app();
	  while (1);
	}
	else
	{
	  if (m_disk_is_mounted)
	  {
		  flash_res = f_mount(NULL, USERPath, 1);//unmount before go to app
	  }
	  HAL_RCC_DeInit();
	  SysTick->CTRL = 0;
	  SysTick->LOAD = 0;
	  SysTick->VAL = 0;
	  SYSCFG->MEMRMP = 0x01; //choose the flash memory
	  HAL_DeInit();
	  SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk |\
	  SCB_SHCSR_BUSFAULTENA_Msk | \
	  SCB_SHCSR_MEMFAULTENA_Msk ) ;
	  __disable_irq();
	  jump_to_app = (jump_func)(*(volatile unsigned int *)(APP_ADDR + 4));
	  __set_MSP(*(volatile unsigned int*)APP_ADDR);
	  jump_to_app();
	  while (1);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void update_firmware (void)
{

	if (fatfs_read_file (update_file, info_buff, 16))
	{
		update_info_ptr = (ota_image_header_t*) info_buff;
	}

	uint32_t size_of_firmware = update_info_ptr->name.firmware_size;
//	uint32_t word_to_write = size_of_firmware / 4; // size_of_firmware count by byte to word (4byte)
//	uint32_t word_wrote = 0;
//	uint32_t word_temp[1024]; //buffer contain data to write to flash
	uint32_t byte_wrote;
	uint8_t update_data [4096];
	uint8_t sector_to_erase = (GetSector (APP_ADDR + size_of_firmware) - GetSector(APP_ADDR)); //sectors to erase at app addr
	sector_to_erase += 1;
	Flash_Erase(APP_ADDR, sector_to_erase); //Erase before write
	uint32_t size_of_update_file = fatfs_get_file_size (update_file);
	uint32_t crc_value_read;
	uint32_t crc_calculate = 0;
	fatfs_read_file_at_pos (update_file, (uint8_t*) &crc_value_read, 4, size_of_update_file - 4);
	while (byte_wrote < size_of_firmware)
	{

//		Flash_Read_Array_32bit (word_temp, OTA_ADDR + word_wrote * 4, 1024);
		memset (update_data, 255, sizeof (update_data));
		uint32_t byte_read = fatfs_read_file_at_pos (update_file, update_data, sizeof (update_data), 16 + byte_wrote);
		if (!(byte_read / 4))	// byte read = 0/ 1 /2 / 3
		{
			if(byte_read == 0)
			{
				break;
			}
			if (Flash_Write_Array_32bit ((uint32_t*)update_data, APP_ADDR + byte_wrote, 1) != 1)
			{
				Error_Handler();
			}
			if (byte_read == 1)
			{
				crc_calculate += update_data [0];
				break;
			}
			else if (byte_read == 2)
			{
				crc_calculate = crc_calculate + update_data [0]+ update_data [1];
				break;
			}
			else if (byte_read == 3)
			{
				crc_calculate = crc_calculate + update_data [0]+ update_data [1]+ update_data [2];
				break;
			}

		}
		if (Flash_Write_Array_32bit ((uint32_t*)update_data, APP_ADDR + byte_wrote, byte_read / 4) != 1)
		{
			Error_Handler();
		}
		for (uint32_t i = 0; i < 4096; i++)
		{
			crc_calculate += update_data [i];
		}
		byte_wrote += 4096;
	}
	if (crc_calculate != crc_value_read)
	{
		NVIC_SystemReset();
	}
	fatfs_delete_a_file (update_file); //delete file after update

//	if (size_of_firmware % 4)
//	{
//		//flash the rest of file
//		if ((size_of_firmware % 4) == 1)
//		{
//			uint32_t the_rest_word = Flash_Read_Uint (OTA_ADDR + word_wrote * 4);
//			the_rest_word |= 0x00FFFFFF;
//			if (Flash_Write_Uin32t (the_rest_word, APP_ADDR + word_wrote * 4))
//			{
//				Error_Handler();
//			}
//		}
//		else if ((size_of_firmware % 4) == 2)
//		{
//			uint32_t the_rest_word = Flash_Read_Uint (OTA_ADDR + word_wrote * 4);
//			the_rest_word |= 0x0000FFFF;
//			if (Flash_Write_Uin32t (the_rest_word, APP_ADDR + word_wrote * 4))
//			{
//				Error_Handler();
//			}
//		}
//		else if ((size_of_firmware % 4) == 3)
//		{
//			uint32_t the_rest_word = Flash_Read_Uint (OTA_ADDR + word_wrote * 4);
//			the_rest_word |= 0x000000FF;
//			if (Flash_Write_Uin32t (the_rest_word, APP_ADDR + word_wrote * 4))
//			{
//				Error_Handler();
//			}
//			the_rest_word = the_rest_word << 16;
//		}
//	}
//	Flash_Erase (UPDATE_CHECK_ADDR, 1);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	NVIC_SystemReset ();
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
