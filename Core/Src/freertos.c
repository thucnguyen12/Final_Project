/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_storage_if.h"
#include "tusb.h"
#include "app_debug.h"
#include "fatfs.h"
#include "lwrb.h"
#include <stdbool.h>
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
//#include "esp_loader.h"
//#include "example_common.h"
//#include "md5_hash.h"
#include "app_btn.h"
#include "utilities.h"
#include "usart.h"
//#include "stm32_port.h"
#include "app_ethernet.h"
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip.h"
#include "stdio.h"
#include "app_http.h"
#include "lwip/dns.h"
#include "min.h"
#include "min_id.h"
#include "ringBuffer.h"
#include "adc.h"
#include "sntp.h"
#include "rtc.h"
#include "time.h"
#include "app_cli.h"
#include "iwdg.h"
#include "ff.h"
#include "spi.h"
#include "app_spi_flash.h"
#include "app_drv_spi.h"
#include "u8g2.h"
#include "indicator.h"
#include "flash.h"
#include "app_update.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t weekday;
} date_time_t;

typedef enum
{
	ETHERNET_NOT_CONNECTED,
	ETHERNET_CONNECTED,
	ETHERNET_COMPLETED
} ETHERNET_STATE;

typedef union
{
	struct
	{
		uint8_t led1_ok : 1;
		uint8_t led2_ok : 1;
		uint8_t led3_ok : 1;
		uint8_t led4_ok : 1;
		uint8_t led5_ok : 1;
		uint8_t led6_ok : 1;
		uint8_t reserve : 2;
	} res;
	uint8_t value;
} __attribute__((packed)) led_result_t;

typedef union
{
	struct
	{
		uint32_t rs485 : 1;
		uint32_t rs232 : 1;
		uint32_t test_wd_ok : 1;
		uint32_t sim_ok : 1;
		uint32_t temper_ok : 1;
		uint32_t vbat_ok : 1;
		uint32_t vbatrf_ok : 1;
		uint32_t v1v8_ok : 1;
		uint32_t v3v3_ok : 1;
		uint32_t vgsm4v2_ok : 1;
		uint32_t charge_ok : 1;
		uint32_t alarm_ok : 1;
		uint32_t fault_ok : 1;
		uint32_t relay0_ok : 1;
		uint32_t relay1_ok : 1;
		uint32_t vsys_ok : 1;
		uint32_t eth_ok : 1;
		uint32_t reserved : 15;
	} result;
	uint32_t value;
} __attribute__((packed)) func_test_t;

typedef union
{
	struct
	{
		uint16_t eth : 1;
		uint16_t wifi : 1;
		uint16_t gsm : 1;
		uint16_t server : 1;
		uint16_t input0_pass : 1;
		uint16_t input1_pass : 1;
		uint16_t input2_pass : 1;
		uint16_t input3_pass : 1;
		uint16_t button_pass : 1;
		uint16_t main_power_pass : 1;
		uint16_t backup_power_pass : 1;
		uint16_t reserve : 5;
	} name;
	uint16_t value;
} __attribute__((packed)) jig_peripheral_t;

typedef struct

{
	char gsm_imei[16];
	char sim_imei[16];
	uint8_t mac[6];
	uint16_t gsm_voltage;
	jig_peripheral_t peripheral;
	uint8_t temperature;
	uint8_t device_type; // 1 = "BST01", 0 = "BST01-Pro"
	uint8_t fw_version[3]; // Major.minor.build
	uint8_t hw_version[3]; // Major.minor.build
	uint32_t timestamp;
} __attribute__((packed)) jig_value_t;

typedef struct
{
	uint16_t vbatrf_max;
	uint16_t vbatrf_min;
	uint16_t vbat_max;
	uint16_t vbat_min;
	uint16_t v1v8_max;
	uint16_t v1v8_min;
	uint16_t v3v3_max;
	uint16_t v3v3_min;
	uint16_t vsys_max;
	uint16_t vsys_min;
} __attribute__((packed)) test_info;

typedef struct
{
	char *data;
	uint32_t len;
	bool need_free;
} http_msq_t;

#define RESET_CDC_DEBUG_BUFFER_WHEN_USB_DISCONNECT	0

#define DEVICE_TYPE_INVALID		0xFF

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_CDC_TX_RING_BUFFER_SIZE 1024
#define BIT_EVENT_GROUP_BT_IN1_PRESSED (1 << 0)
#define BIT_EVENT_GROUP_BT_IN2_PRESSED (1 << 1)
#define HW_BTN_CONFIG                              \
	{                                              \
		/* PinName       Last state   Idle level*/ \
		{0, 1, 1},                                 \
		{                                          \
			1, 1, 1                                \
		}                                          \
	}

static const uint8_t day_in_month[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
#define FIRSTYEAR 2000 // start year
#define FIRSTDAY 6	   // 0 = Sunday

#define defaultTaskB (1 << 0)
#define cdcTaskB (1 << 1)
#define netTaskB (1 << 2)
/*lcd define*/
#define LCD_HORIZONTAL 128
#define LCD_VERTICAL 64

#define LCD_MEASURE_X_CENTER(x) ((LCD_HORIZONTAL - x) / 2)
#define LCD_MEASURE_Y_CENTER(x) ((LCD_VERTICAL - x) / 2)
#define LCD_4G_CSQ_X_OFFSET (116)

#define LED1_R_ON() HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_G_Pin, 1)
#define LED1_R_OFF() HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_G_Pin, 0)
#define LED1_R_TOGGLE() HAL_GPIO_TogglePin(LED1_R_GPIO_Port, LED1_G_Pin)

#define LED1_G_ON() HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, 1)
#define LED1_G_OFF() HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, 0)
#define LED1_G_TOGGLE() HAL_GPIO_TogglePin(LED1_G_GPIO_Port, LED1_G_Pin)

#define LED2_R_ON() HAL_GPIO_WritePin(LED2_R_GPIO_Port, LED2_G_Pin, 1)
#define LED2_R_OFF() HAL_GPIO_WritePin(LED2_R_GPIO_Port, LED2_G_Pin, 0)
#define LED2_R_TOGGLE() HAL_GPIO_TogglePin(LED2_R_GPIO_Port, LED2_G_Pin)

#define LED2_G_ON() HAL_GPIO_WritePin(LED2_G_GPIO_Port, LED2_G_Pin, 1)
#define LED2_G_OFF() HAL_GPIO_WritePin(LED2_G_GPIO_Port, LED2_G_Pin, 0)
#define LED2_G_TOGGLE() HAL_GPIO_TogglePin(LED2_G_GPIO_Port, LED2_G_Pin)

#define LR1 0
#define LR2 1
#define LR3 2
#define LR4 3
#define LR5 4
#define LR6 5
#define LB1 6
#define LB2 7
#define LB3 8
#define LB4 9
#define LB5 10
#define LB6 11

#define HTTP_TIMESTAMP_TIMEOUT_MS 5000
#define WAIT_HTTP_GET_TIMESTAMP_COMPLETE() xSemaphoreTake(m_sem_sync_time, HTTP_TIMESTAMP_TIMEOUT_MS)
#define HTTP_GET_TIMESTAMP_COMPLETE()	xSemaphoreGive(m_sem_sync_time)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
EventGroupHandle_t m_wdg_event_group = NULL;
static EventBits_t uxBits;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/**************        BUTTON VAR               ***********/
static app_btn_hw_config_t m_button_cfg[] = HW_BTN_CONFIG;
static EventGroupHandle_t m_button_event_group = NULL;
/**********************************************************/

/*************************** 	  FLASH DISK VARIABLE        *************************************************************/
BYTE gFSWork[_MAX_SS];
UINT br, bw;   // File read/write count
UINT fbr, fbw; // File read/write count
FRESULT flash_res;
bool m_disk_is_mounted = false;
/***********************************************************************************************************************/
/********************************  	 CDC TASK VAR     *****************************************************************/
StackType_t cdc_stack[configMINIMAL_STACK_SIZE];
StaticTask_t cdc_taskdef;
/**********************************************************************************************************************/

//********************************    tiny USB TASK VAR     ********************************************************//
static TaskHandle_t m_USB_handle = NULL;
bool tusb_init_flag = false;

//********************************************************************************************************************//

/*********************8*************       NET VAR    ******************************************************************/
struct netif g_netif;
static TaskHandle_t m_task_handle_protocol = NULL; // NET APP TASK
osThreadId DHCP_id;
SemaphoreHandle_t hHttpStart;
static QueueHandle_t m_http_msq;
bool send_offline_file = false;
static SemaphoreHandle_t m_sem_signal_send_file, m_sem_sync_time, m_sem_send_current_test_result;
static char ip_addr_file_content[128];
static char *local_ip_addr;
static uint16_t m_server_port = 80;

static http_msq_t rx_msq =
{
	.data = 0,
	.len = 0,
};

static const char *ip_file = "ip_local.txt";
/*********************************************************************************************************************/

//***************************  TESTING TASK VAR   *********************************//
static TaskHandle_t mTest_Handle_t = NULL;
char gsm_imei[16];
char sim_imei[16];
static uint8_t rs232_incomming_mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t m_last_remember_rs232_mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//static jig_value_t *to_send_value;
//static jig_value_t temporary_test_result;
jig_value_t current_jig_test_value;
jig_value_t final_jig_value;
static bool m_found_jig_plugged = false;
static func_test_t function_test_result;
static bool all_peripheral_is_passed = false;
static const char *test_info_file = "test_info.txt";
//static char json_send_to_sever[1024];
static test_info voltage_info;
uint8_t config_param_buffer[512];
uint8_t relay0_toggle_counter;
uint8_t relay1_toggle_counter;
static bool new_data_need_send_to_server = false;
static uint16_t m_adc_result[5];
static uint8_t rs485_idle_line_detect;

static uint32_t test_process_begin_timestamp = 0;
static bool is_test_running = false;


static GPIO_PinState led_status[12];
static uint32_t test_timeout_s = 60;

static led_result_t led_detect =
{
	.res.led1_ok = 0,
	.res.led2_ok = 0,
	.res.led3_ok = 0,
	.res.led4_ok = 0,
	.res.led5_ok = 0,
	.res.led6_ok = 0,
	.res.reserve = 0b11
};

//*****************************************************************************//

//********************************* MIN PROTOCOL VAR**********************//
lwrb_t m_ringbuffer_host_rx;
uint8_t m_rs232_host_buffer[512];
static uint8_t m_min_rx_buffer[256];
static min_context_t m_min_context;
static min_frame_cfg_t m_min_setting = MIN_DEFAULT_CONFIG();
//*******************************************************************//
// ******************** RING BUFFER********************************//
static uint8_t rs485_ringbuffer[256];
static uint16_t rs485_buffer_index = 0;
//******************************************************************//

//************************* TIME PROTOCOL PRO AND VAR***********************************//
void lwip_sntp_recv_cb(uint32_t time);
//*******************************************************************************************//

//************************** RTC VAR*******************************//

RTC_TimeTypeDef m_rtc_time = {0};
RTC_DateTypeDef m_rtc_date = {0};
static RTC_TimeTypeDef m_last_time_sync_timestamp_data = {0};
RTC_TimeTypeDef sTimeToSend = {0};
RTC_DateTypeDef sDateToSend = {0};
//  RTC_HandleTypeDef hrtc1;
static date_time_t date_time;
//********************************************************************//
// ********************* APP CLI VARIABLE****************************//
uint32_t cdc_tx(const void *buffer, uint32_t size);
int32_t USB_puts(char *msg);

void cli_cdc_tx(uint8_t *buffer, uint32_t size);
int cli_cdc_puts(const char *msg);

static app_cli_cb_t m_tcp_cli =
{
	.puts = cli_cdc_tx,
	.printf = cli_cdc_puts,
	.terminate = NULL
};


static bool m_cli_started = false;
// ******************************************************************//
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//***************************************   TASK PFT *****************************************************************//
void cdc_task(void *params);
void usb_task(void *params);
void testing_task(void *arg);
void set_rtc_date_time(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
static void initialize_stnp(void);
static uint32_t convert_date_time_to_second(date_time_t *t);
static void convert_second_to_date_time(uint32_t sec, date_time_t *t, uint8_t Calyear);
static uint8_t get_weekday(date_time_t time);
void button_initialize(uint32_t button_num);

uint32_t btn_read(uint32_t pin);
void on_btn_pressed(int number, int event, void *pData);
void on_btn_release(int number, int event, void *pData);
void on_btn_hold(int number, int event, void *pData);
static void on_btn_hold_so_long(int index, int event, void *pData);

void Netif_Config(bool restart);
void net_task(void *argument);


int16_t build_json_msg_send_to_server(jig_value_t *value,
									func_test_t *test,
									led_result_t *led,
									bool all_passed,
									char *output);


// void make_string_from_mac(char* str);
void min_rx_callback(void *min_context, min_msg_t *frame);

bool RS232_tx(void *ctx, uint8_t byte);

void test_point_voltage_monitor(void);

static bool is_peripheral_testcase_passed(jig_value_t *value);

void send_test_command(min_msg_t *test_command);

bool RS232_tx(void *ctx, uint8_t byte);

void monitor_led_test_point(void);

void display_test_result_on_lcd(func_test_t *res, jig_value_t *value, led_result_t *led_result);
void sys_delay_ms(uint32_t ms);
static void delay_ns(uint32_t ns);
uint8_t u8g2_gpio_8080_update_and_delay(U8X8_UNUSED u8x8_t *u8x8,
										U8X8_UNUSED uint8_t msg,
										U8X8_UNUSED uint8_t arg_int,
										U8X8_UNUSED void *arg_ptr);

void lcd_clr_screen(void);
void lcd_display_header(const char *msg);
void lcd_display_error_at_pos(char *msg, u8g2_uint_t x, u8g2_uint_t y, const uint8_t *font);
void lcd_display_content(const char *msg);
void lcd_display_content_at_pos(const char *msg, u8g2_uint_t x, u8g2_uint_t y);
u8g2_t m_u8g2;
//********************************************************************************************************************//
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

extern void MX_LWIP_Init(void);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
									StackType_t **ppxIdleTaskStackBuffer,
									uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
	/* init code for LWIP */
	//  MX_LWIP_Init();

	/* init code for USB_DEVICE */
	//  MX_USB_DEVICE_Init();
	/* USER CODE BEGIN StartDefaultTask */
	HAL_IWDG_Refresh(&hiwdg); // FEED WDT

	/* Renum USB */
	HAL_GPIO_WritePin(JIG_CS_GPIO_Port, JIG_CS_Pin, 1); // pull up cs pin to mount flash

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	uint32_t now = xTaskGetTickCount();
	MX_FATFS_Init();


	flash_res = f_mount(&USERFatFS, USERPath, 1);
	if (flash_res != FR_OK)
	{
		DEBUG_WARN("Mount flash fail\r\n");
		flash_res = f_mkfs(USERPath, FM_ANY, 0, gFSWork, sizeof gFSWork);
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

	vTaskDelayUntil(&now, 500); // time for usb renum

	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	MX_USB_DEVICE_Init();

	//***********************************************************************//

	//*************************** INIT BUTTON APP**********************//
	app_btn_config_t btn_conf;
	btn_conf.config = m_button_cfg;
	btn_conf.btn_count = 2;
	btn_conf.get_tick_cb = xTaskGetTickCount;
	btn_conf.btn_initialize = button_initialize;
	btn_conf.btn_read = btn_read;
	btn_conf.scan_interval_ms = 50;
	app_btn_initialize(&btn_conf);
	//    app_btn_initialize(&btn_conf);
	app_btn_register_callback(APP_BTN_EVT_HOLD, on_btn_hold, NULL);
	app_btn_register_callback(APP_BTN_EVT_HOLD_SO_LONG, on_btn_hold_so_long, NULL);
	app_btn_register_callback(APP_BTN_EVT_PRESSED, on_btn_pressed, NULL);
	app_btn_register_callback(APP_BTN_EVT_RELEASED, on_btn_release, NULL);

	m_button_event_group = xEventGroupCreate(); //>>>>>>> CREATE BUTTON EVENT VAR

	//************************* INIT BUTTON END **********************//
	m_wdg_event_group = xEventGroupCreate(); // creat group of wdt

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)m_adc_result, sizeof(m_adc_result)/sizeof(m_adc_result[0])); //>>>Start ADC

	vTaskDelay(100);

	if (m_USB_handle == NULL)
	{
		xTaskCreate(usb_task, "usb_task", 512, NULL, 4, &m_USB_handle); // pio =1
	}
	if (mTest_Handle_t == NULL)
	{
		xTaskCreate(testing_task, "testing_task", 512, NULL, 5, &mTest_Handle_t);
	}

	if (m_task_handle_protocol == NULL)
	{
		xTaskCreate(net_task, "net_task", 1024, NULL, 0, &m_task_handle_protocol);
	}

	m_http_msq = xQueueCreate(2, sizeof(http_msq_t));

	if (m_cli_started == false)
	{
		m_cli_started = true;
		app_cli_start(&m_tcp_cli);
		DEBUG_WARN("APP CLI STARTED \r\n");
	}

	/* Infinite loop */
	for (;;)
	{
		app_btn_scan(NULL);
		xEventGroupSetBits(m_wdg_event_group, defaultTaskB);
		osDelay(10);
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void on_http_post_file_event_callback(app_http_event_t event, void * data)
{
	if (event == APP_HTTP_EVENT_FINISH_SUCCESS)
	{
		xSemaphoreGive(m_sem_signal_send_file);
	}
}

static void on_http_event_timestamp_callback(app_http_event_t event, void * data)
{
	if (event == APP_HTTP_EVENT_FINISH_SUCCESS)
	{
		HTTP_GET_TIMESTAMP_COMPLETE();
	}
}

static void on_http_post_test_result_data(app_http_event_t event, void * data)
{
	if (event == APP_HTTP_EVENT_FINISH_SUCCESS)
	{
		xSemaphoreGive(m_sem_send_current_test_result);
	}
	else if (event == APP_HTTP_EVENT_DATA)
	{
		app_http_data_t *require = (app_http_data_t*)data;
		if (rx_msq.len)
		{
			require->data = (uint8_t*)rx_msq.data;
			require->length = rx_msq.len;
			rx_msq.len = 0;		// nore more data for the next time
		}
		else
		{
			require->data = NULL;
			require->length = 0;
		}
	}
}

void save_offline_data(char *data, uint32_t size)
{
	if (fatfs_is_file_or_folder_existed("jig") == FILE_NOT_EXISTED)
	{
		fatfs_create_directory("jig");
	}

	DIR dir;
	char file_name[64];
	FRESULT fre = f_opendir(&dir, "jig");
	if (fre == FR_OK)
	{
		// Get Date and time
		RTC_DateTypeDef date = {0};
		RTC_TimeTypeDef time = {0};
		HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);

		// Make test filename
		sprintf(file_name, "jig/%02u-%02u-%02u-%04u",
				time.Hours,
				date.Date,
				date.Month,
				date.Year + 2000);

		// Get file size
		int32_t file_size = fatfs_get_file_size(file_name);

		DEBUG_INFO("File size %d bytes\r\n", file_size);
		uint32_t byte_write = fatfs_write_json_to_a_file_at_pos(file_name, data, size, file_size);

		if (byte_write == size)
		{
			DEBUG_INFO("Write test result to file %s success\r\n", file_name);
		}
		else
		{
			DEBUG_ERROR("Write test result to file %s failed\r\n", file_name);
		}
		f_closedir(&dir);
	}
}


FRESULT fre; // result
DIR dir;
static uint32_t ip_readbyte;
static ETHERNET_STATE m_ethernet_state = ETHERNET_NOT_CONNECTED;
static char file_name[256+64];
void net_task(void *argument)
{
#if LWIP_DHCP
	/* Start DHCPClient */
	osThreadDef(DHCP, DHCP_Thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
	DHCP_id = osThreadCreate(osThread(DHCP), &g_netif);
	vTaskDelay(1);
#endif

	initialize_stnp();
	app_http_config_t http_send_json_payload_to_server;
	app_http_config_t http_config_send_file_to_server;
	app_http_config_t http_get_timestamp_configuration;

	m_sem_signal_send_file = xSemaphoreCreateBinary();
	if (m_sem_signal_send_file == NULL)
	{
		DEBUG_ERROR("Create semaphore error\r\n");
		NVIC_SystemReset();
	}

	m_sem_sync_time = xSemaphoreCreateBinary();
	if (m_sem_sync_time == NULL)
	{
		DEBUG_ERROR("Create semaphore error\r\n");
		NVIC_SystemReset();
	}

	m_sem_send_current_test_result = xSemaphoreCreateBinary();
	if (m_sem_send_current_test_result == NULL)
	{
		DEBUG_ERROR("Create semaphore error\r\n");
		NVIC_SystemReset();
	}



	if (m_disk_is_mounted)
	{
		if (fatfs_is_file_or_folder_existed(ip_file) == FILE_EXISTED)
		{
			DEBUG_INFO("IP FILE OK\r\n");
		}
		else if (fatfs_is_file_or_folder_existed(ip_file) == FILE_NOT_EXISTED)
		{
			DEBUG_ERROR("IP FILE DOESN'T EXIST \r\n");
			lcd_display_content("IP LỖI");
			indicator_buzzer_beeps(10);
		}
		else
		{
			DEBUG_ERROR("FATFS ERROR \r\n");
		}

		/**
		 *
		 *	{
		 *		"ip":"192.168.1.65",
		 *		"port":8080
			}
		 */

		ip_readbyte = fatfs_read_file(ip_file, (uint8_t *)ip_addr_file_content, sizeof(ip_addr_file_content));
		DEBUG_INFO("Server configuration:\r\n%s\r\n", ip_addr_file_content);

		char *port_str = strstr(ip_addr_file_content, "\"port\":");
		local_ip_addr = strstr(ip_addr_file_content, "\"ip\":\"");
		local_ip_addr += strlen("\"ip\":\"");
		local_ip_addr = strtok(local_ip_addr, "\"");

		m_server_port = utilities_get_number_from_string(strlen("\"port\":"), port_str);

		DEBUG_INFO("IP: %s:%u\r\n", local_ip_addr, m_server_port);
		ip_readbyte = strlen(local_ip_addr);
		if (ip_readbyte < 15)
		{
			DEBUG_INFO("VALID ADDR \r\n");
		}
		else
		{
			lcd_display_content("IP LỖI");
			DEBUG_ERROR("INVALID ADDR\r\n");
		}
	}

	uint32_t sync_time_interval = 5;

	for (;;)
	{
		switch (m_ethernet_state)
		{
		case ETHERNET_NOT_CONNECTED:
		{
			if (m_ip_assigned)
			{
				m_ethernet_state = ETHERNET_CONNECTED;
			}
			break;
		}


		case ETHERNET_CONNECTED:
			/* If folder test is not existed =>> Create test folder*/
			if (fatfs_is_file_or_folder_existed("jig") == FILE_NOT_EXISTED)
			{
				fre = fatfs_create_directory("jig");
			}
			vTaskDelay(1000);		// Delay after network connect

			// Scan all file in test folder and send to server
			// Then delete it
			FRESULT res;
			DIR dir;
			static FILINFO fno;
			static const char *folder = "0:/jig";
			res = f_opendir(&dir, folder);                       /* Open the directory */
			if (res == FR_OK)
			{
				uint32_t wait_time = HTTPC_POLL_TIMEOUT*1000;
				for (;;)
				{
					res = f_readdir(&dir, &fno);                   /* Read a directory item */
					if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
					if (fno.fattrib & AM_DIR) 	/* It is a directory, dont care about it */
					{
//			                i = strlen(path);
//			                sprintf(&path[i], "/%s", fno.fname);
//			                res = scan_files(path);                    /* Enter the directory */
//			                if (res != FR_OK) break;
//			                path[i] = 0;
					}
					else
					{                                       /* It is a file. */
						DEBUG_INFO("Send file %s/%s to server\r\n", folder, fno.fname);

						snprintf(file_name, sizeof(file_name), "%s/%s", folder, fno.fname);
						int32_t file_size = fatfs_get_file_size(file_name);
						if (file_size > 0)
						{
							DEBUG_INFO("Size %u bytes\r\n", file_size);
							if (file_size)
							{
								sprintf(http_config_send_file_to_server.url, local_ip_addr);
								sprintf(http_config_send_file_to_server.file, "%s", "/fact/api/firesafe/test-result");
								http_config_send_file_to_server.port = m_server_port;
								http_config_send_file_to_server.on_event_cb = on_http_post_file_event_callback;
								http_config_send_file_to_server.method = APP_HTTP_POST;
								http_config_send_file_to_server.transfile = TRANS_FILE;
								http_config_send_file_to_server.len = file_size;
								http_config_send_file_to_server.local_file = file_name;

								app_http_start(&http_config_send_file_to_server);
								if (xSemaphoreTake(m_sem_signal_send_file, wait_time) == pdTRUE)
								{
									DEBUG_INFO("Send data to server success, delete file %s\r\n", file_name);
									fatfs_delete_a_file(file_name);
								}
								else
								{
									DEBUG_ERROR("Send %s/%s failed\r\n", folder, fno.fname);
								}
							}
						}
					}
				}
				f_closedir(&dir);
			}
			else
			{
				DEBUG_ERROR("Open dir %s failed\r\n", folder);
			}
			m_ethernet_state = ETHERNET_COMPLETED;
			break;

		case ETHERNET_COMPLETED:
			if (!eth_is_cable_connected(&g_netif))
			{
				m_ethernet_state = ETHERNET_NOT_CONNECTED;
				m_ip_assigned = false;
			}

			{
				RTC_TimeTypeDef TimeNow;
				HAL_RTC_GetTime(&hrtc, &TimeNow, RTC_FORMAT_BIN);
				if ((TimeNow.Minutes - m_last_time_sync_timestamp_data.Minutes) >= sync_time_interval)
				{
					/* Set http config parameters*/
					sprintf(http_get_timestamp_configuration.url, "%s", local_ip_addr);
					http_get_timestamp_configuration.port = m_server_port;
					sprintf(http_get_timestamp_configuration.file, "%s", "/fact/api/firesafe/get-timestamp");
					http_get_timestamp_configuration.on_event_cb = on_http_event_timestamp_callback;
					http_get_timestamp_configuration.method = APP_HTTP_GET;

					app_http_start(&http_get_timestamp_configuration);

					DEBUG_INFO("Get timestamp from local server\r\n");

					// Wait until device received timestamp
					if (WAIT_HTTP_GET_TIMESTAMP_COMPLETE())
					{
						uint8_t data[64];
						memset(data, 0, sizeof(data));
						http_get_body(data);
						DEBUG_INFO("http data get %s\r\n", data);
						uint32_t timestamp = utilities_get_number_from_string(0, (char *)data);
						lwip_sntp_recv_cb(timestamp);
						sync_time_interval = 5;
					}
					else
					{
						DEBUG_ERROR("Wait timestamp error\r\n");
						sync_time_interval = 1;
					}
					m_last_time_sync_timestamp_data = TimeNow;
				}
			}
			break;
		default:
			break;
		}

		if (xQueueReceive(m_http_msq, &rx_msq, 10))
		{
			DEBUG_INFO("%.*s\r\n", rx_msq.len, rx_msq.data);
			if (eth_is_cable_connected(&g_netif))
			{
				//				sprintf(http_cfg.url, "%s", "192.168.1.27");
				memcpy(http_send_json_payload_to_server.url, (char *)local_ip_addr, ip_readbyte);
				DEBUG_INFO("%s\r\n", local_ip_addr);
				DEBUG_INFO("%s\r\n", http_send_json_payload_to_server.url);
				http_send_json_payload_to_server.port = m_server_port;
				sprintf(http_send_json_payload_to_server.file, "%s", "/fact/api/firesafe/test-result");
				http_send_json_payload_to_server.on_event_cb = on_http_post_test_result_data;
				http_send_json_payload_to_server.method = APP_HTTP_POST;
				http_send_json_payload_to_server.transfile = TRANS_STRING;
				http_send_json_payload_to_server.len = rx_msq.len;

//				trans_content_to_body(rx_msq.data, rx_msq.len);

				app_http_start(&http_send_json_payload_to_server);

				// Wait for http complete
				if (!xSemaphoreTake(m_sem_send_current_test_result, 5000))
				{
					DEBUG_ERROR("Send test data error, save test result to memory\r\n");

					// Trim []
					char *p = strstr(rx_msq.data, "[");		// never fail
					p++;
					char *q = strstr(p, "]");
					save_offline_data(p, q-p);
				}
				else
				{
					DEBUG_INFO("Http post cplt\r\n");
				}

				if (rx_msq.need_free)
				{
					DEBUG_INFO("Free http rx memory\r\n");
					vPortFree(rx_msq.data);
					rx_msq.data = NULL;
					rx_msq.len = 0;
				}

			}
			else
			{
				DEBUG_INFO("ETH cable is not plugged, save file to storage\r\n");
				// Trim []
				char *p = strstr(rx_msq.data, "[");		// never fail
				p++;
				char *q = strstr(p, "]");
				save_offline_data(p, q-p);

				if (rx_msq.need_free)
				{
					vPortFree(rx_msq.data);
					rx_msq.data = NULL;
					rx_msq.len = 0;
				}
			}
			xEventGroupSetBits(m_wdg_event_group, netTaskB);
		}
		else
		{
			xEventGroupSetBits(m_wdg_event_group, netTaskB);
		}
	}
}
//***********************************************************************************************************//

//*********************************** TEST TASK******************************************************//

void testing_task(void *arg)
{
	// init min protocol
	lwrb_init(&m_ringbuffer_host_rx, &m_rs232_host_buffer, sizeof(m_rs232_host_buffer));
	m_min_setting.get_ms = sys_get_ms;
	m_min_setting.last_rx_time = 0x00;
	m_min_setting.rx_callback = min_rx_callback;
	m_min_setting.timeout_not_seen_rx = 5000;
	m_min_setting.tx_byte = RS232_tx;
	m_min_setting.use_timeout_method = 1;

	m_min_context.callback = &m_min_setting;
	m_min_context.rx_frame_payload_buf = m_min_rx_buffer;
	min_init_context(&m_min_context);
	uint32_t last_tick = 0;
	uint32_t last_vol_tick = 0;
	//******************************* LCD INIT******************************//
#if 1 // paralle mode
	u8g2_Setup_st7920_p_128x64_f(&m_u8g2, U8G2_R2, u8x8_byte_8bit_8080mode, u8g2_gpio_8080_update_and_delay);
#else
#warning "do nothing"
//	software_spi_cfg_t spi_cfg;
//	spi_cfg.cs_out = software_spi_set_cs;
//	spi_cfg.miso_in = software_spi_read_miso;
//	spi_cfg.mosi_out = software_spi_set_mosi;
//	spi_cfg.sck_out = software_spi_set_sck;
//	software_spi_inititalize(&spi_cfg);
//	u8g2_Setup_st7920_s_128x64_f(&m_u8g2, U8G2_R0, u8x8_byte_3wire_sw_spi, u8g2_gpio_3w_spi_update_and_delay);
#endif
	u8g2_InitDisplay(&m_u8g2);	   // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&m_u8g2, 0); // wake up display
	u8g2_ClearBuffer(&m_u8g2);
	u8g2_ClearDisplay(&m_u8g2);

	u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1);
	u8g2_FirstPage(&m_u8g2);
	do
	{
		const char *str_begin = "JIG TEST";
		u8g2_DrawUTF8(&m_u8g2,
					  LCD_MEASURE_X_CENTER(u8g2_GetUTF8Width(&m_u8g2, str_begin)),
					  LCD_MEASURE_Y_CENTER(u8g2_GetMaxCharHeight(&m_u8g2)),
					  str_begin);
	} while (u8g2_NextPage(&m_u8g2));
	u8g2_SendBuffer(&m_u8g2); // transfer data from uC memory to display

	bool enter_test_mode = false;

	//**********************************************************************//
	const min_msg_t test_cmd =
	{
		.id = MIN_ID_RS232_ENTER_TEST_MODE,
		.len = 0,
		.payload = NULL
	};

	const min_msg_t reset_cmd_wd =
	{
		.id = MIN_ID_TEST_WATCHDOG,
		.len = 0,
		.payload = NULL
	};

	min_msg_t time_tested;
	if (m_disk_is_mounted)
	{
		fatfs_file_info_t fatfs_err = fatfs_is_file_or_folder_existed(test_info_file);
		if (fatfs_err == FILE_NOT_EXISTED)
		{
			DEBUG_ERROR("Configuration info not exited in flash\r\n");
			lcd_display_content("KHÔNG CÓ FILE");
		}
		else if (fatfs_err == FILE_ERROR)
		{
			DEBUG_ERROR("FATFS ERROR\r\n");
		}

		uint32_t file_size = fatfs_read_file(test_info_file, (uint8_t *)config_param_buffer, sizeof(config_param_buffer) - 1);
		/*
		 * {vbatmax:4.3,
		 * 	vbatmin:4.5,
		 *
		 *
		 * }
		 * */
		DEBUG_VERBOSE("READ %d byte size\r\n", file_size);
		if (file_size > 0)
		{
			DEBUG_INFO("\r\n%s\r\n", config_param_buffer);
			char *ptr = strstr((char *)config_param_buffer, "\"vbat_max\":");
			if (ptr)
			{
				ptr += strlen("\"vbat_max\":");
				voltage_info.vbat_max = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"vbat_min\":");
			if (ptr)
			{
				ptr += strlen("\"vbat_min\":");
				voltage_info.vbat_min = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"vbatrf_max\":");
			if (ptr)
			{
				ptr += strlen("\"vbatrf_max\":");
				voltage_info.vbatrf_max = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"vbatrf_min\":");
			if (ptr)
			{
				ptr += strlen("\"vbatrf_min\":");
				voltage_info.vbatrf_min = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"v1v8_max\":");
			if (ptr)
			{
				ptr += strlen("\"v1v8_max\":");
				voltage_info.v1v8_max = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"v1v8_min\":");
			if (ptr)
			{
				ptr += strlen("\"v1v8_min\":");
				voltage_info.v1v8_min = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"v3v3_max\":");
			if (ptr)
			{
				ptr += strlen("\"v3v3_max\":");
				voltage_info.v3v3_max = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"v3v3_min\":");
			if (ptr)
			{
				ptr += strlen("\"v3v3_min\":");
				voltage_info.v3v3_min = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"vsys_max\":");
			if (ptr)
			{
				ptr += strlen("\"vsys_max\":");
				voltage_info.vsys_max = utilities_get_number_from_string(0, ptr);
			}
			ptr = strstr((char *)config_param_buffer, "\"vsys_min\":");
			if (ptr)
			{
				ptr += strlen("\"vsys_min\":");
				voltage_info.vsys_min = utilities_get_number_from_string(0, ptr);
			}

			ptr = strstr((char *)config_param_buffer, "\"test_timeout\":");
			if (ptr)
			{
				ptr += strlen("\"test_timeout\":");
				test_timeout_s = utilities_get_number_from_string(0, ptr);
				if (test_timeout_s == 0)
				{
					test_timeout_s = 60;
				}
				test_timeout_s *= 1000;
			}

			DEBUG_INFO("%d < vbat < %d (mV) \r\n", voltage_info.vbat_min, voltage_info.vbat_max);
			DEBUG_INFO("%d < vbatrf < %d (mV) \r\n", voltage_info.vbatrf_min, voltage_info.vbatrf_max);
			DEBUG_INFO("%d < v1v8 < %d (mV) \r\n", voltage_info.v1v8_min, voltage_info.v1v8_max);
			DEBUG_INFO("%d < v3v3 < %d (mV) \r\n", voltage_info.v3v3_min, voltage_info.v3v3_max);
			//			DEBUG_INFO ("%d < v4v2 < %d (mV) \r\n", voltage_info.v4v2_min, voltage_info.v4v2_max);
			DEBUG_INFO("%d < vsys < %d (mV) \r\n", voltage_info.vsys_min, voltage_info.vsys_max);
		}
	}

	for (;;)
	{
		//		DEBUG_INFO ("ENTER TESTING LOOP \r\n");
		//		if (HAL_GPIO_ReadPin(MODE1_GPIO_Port, MODE1_Pin))//xet trang thai bit gat
		//		{

		if (enter_test_mode == false)
		{
			//				u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1);
			lcd_display_content("ĐANG CHỜ");
			enter_test_mode = true;
		}

		// Wait for button test pressed
		if (xEventGroupWaitBits(m_button_event_group,
								BIT_EVENT_GROUP_BT_IN1_PRESSED,
								pdTRUE,
								pdTRUE,
								0) &&
			(is_test_running == false))

		{
			indicator_buzzer_beeps(1);
			lcd_clr_screen();

			// Reset main board
			send_test_command((min_msg_t *)&reset_cmd_wd);

			// Flush serial ringbuffer
			while (1)
			{
				uint8_t ch;
				if (lwrb_read(&m_ringbuffer_host_rx, &ch, 1))
				{
				//	min_rx_feed(&m_min_context, &ch, 1);
				}
				else
				{
					min_timeout_poll(&m_min_context);
					break;
				}
			}


			__disable_irq();

			// Reset rs485 ringbuffer
			memset(rs485_ringbuffer, 0, sizeof(rs485_ringbuffer));
			rs485_buffer_index = 0;

			led_detect.value = 0;
			memset(&current_jig_test_value, 0, sizeof(current_jig_test_value));
			memset(&final_jig_value, 0, sizeof(final_jig_value));
			relay0_toggle_counter = 0;
			relay1_toggle_counter = 0;
			function_test_result.value = 0;
			new_data_need_send_to_server = false;
			all_peripheral_is_passed = 0;
			current_jig_test_value.device_type = DEVICE_TYPE_INVALID;		// invalid device type
			is_test_running = true;

			__enable_irq();

			test_process_begin_timestamp = HAL_GetTick();
		}

		// Feed all data in rs232 ringbuffer
		while (1)
		{
			uint8_t ch;
			if (lwrb_read(&m_ringbuffer_host_rx, &ch, 1))
			{
				min_rx_feed(&m_min_context, &ch, 1);
			}
			else
			{
				min_timeout_poll(&m_min_context);
				break;
			}
		}


		// Check RS485 data
		if (rs485_idle_line_detect)
		{
			rs485_idle_line_detect--;
			// Only check when rs485 is failed and idle line detected
			if (rs485_idle_line_detect == 0 && function_test_result.result.rs485 == false)
			{
				DEBUG_VERBOSE("RS485 =>> \r\n%s", rs485_ringbuffer);
				char *testptr = strstr((char *)rs485_ringbuffer, "fire_safe:");
				if (testptr)
				{
					DEBUG_INFO("RS485 passed\r\n");
					function_test_result.result.rs485 = true;
					memset(rs485_ringbuffer, 0, sizeof(rs485_ringbuffer));
				}
			}
			else if (function_test_result.result.rs485)
			{
				rs485_buffer_index = 0;
			}
		}

		uint32_t now = HAL_GetTick();
		if (is_test_running && ((now - test_process_begin_timestamp) >= test_timeout_s))
		{
			DEBUG_ERROR("Test timeout is over\r\n");
			is_test_running = false;
			new_data_need_send_to_server = true;
		}


		uint32_t time_now = sys_get_ms();
		static uint32_t test_blink = 0;
		if (is_test_running)
		{
			if ((time_now - test_blink) > (uint32_t)1000)
			{
				test_blink = time_now;

				if (!final_jig_value.peripheral.name.button_pass)
				{
					lcd_display_content_at_pos("SWITCH...", 32, 38);
				}
				else
				{
					/**
					 * 	12:55:13
					 * TESTING (55s)
					 * IMEI : 123
					 */
					char counter[24];
 					u8g2_ClearBuffer(&m_u8g2);

					// Display header
					RTC_TimeTypeDef time;
					HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
					sprintf(counter, "%02u:%02u:%02u", time.Hours, time.Minutes, time.Seconds);
					u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1); // can chon font cho header
					do
					{
						u8g2_DrawUTF8(&m_u8g2,
									  LCD_MEASURE_X_CENTER(u8g2_GetUTF8Width(&m_u8g2, counter)),
									  15,
									  counter);
					} while (u8g2_NextPage(&m_u8g2));

					u8g2_SetFont(&m_u8g2, u8g2_font_6x13_tf);
					sprintf(counter, "TESTING %lus", (HAL_GetTick() -  test_process_begin_timestamp + 999) / 1000);
					u8g2_DrawUTF8(&m_u8g2, 32, 38, counter);
					sprintf(counter, "IMEI %s", final_jig_value.gsm_imei);
					u8g2_DrawUTF8(&m_u8g2, 5,55, counter);
					u8g2_SendBuffer(&m_u8g2);
				}
			}
		}


		if (m_found_jig_plugged && is_test_running)
		{
			if (memcmp(rs232_incomming_mac_addr, m_last_remember_rs232_mac_addr, 6) != 0)
			{
				DEBUG_INFO("MAC changed to: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
								rs232_incomming_mac_addr[0],
								rs232_incomming_mac_addr[1],
								rs232_incomming_mac_addr[2],
								rs232_incomming_mac_addr[3],
								rs232_incomming_mac_addr[4],
								rs232_incomming_mac_addr[5]);

				memcpy(m_last_remember_rs232_mac_addr, rs232_incomming_mac_addr, 6);
				memset(&current_jig_test_value, 0, sizeof(current_jig_test_value));
				memcpy(&final_jig_value, &current_jig_test_value, sizeof(current_jig_test_value));
				send_test_command((min_msg_t *)&reset_cmd_wd);
			}
			else
			{
				DEBUG_VERBOSE("Same mac address\r\n");
				if (all_peripheral_is_passed)
				{
					break;
				}

				if (current_jig_test_value.temperature)
				{
					final_jig_value.temperature = current_jig_test_value.temperature;
				}

				if (current_jig_test_value.peripheral.name.eth)
				{
					if (!final_jig_value.peripheral.name.eth)
					{
						DEBUG_INFO("Ethernet passed\r\n");
					}
					final_jig_value.peripheral.name.eth = 1;
				}

				if (current_jig_test_value.peripheral.name.wifi)
				{
					if (!final_jig_value.peripheral.name.wifi)
					{
						DEBUG_INFO("Wifi passed\r\n");
					}
					final_jig_value.peripheral.name.wifi = 1;
				}

				if (current_jig_test_value.peripheral.name.gsm)
				{
					if (!final_jig_value.peripheral.name.gsm)
					{
						DEBUG_INFO("GSM passed\r\n");
					}
					final_jig_value.peripheral.name.gsm = 1;
				}

				if (current_jig_test_value.peripheral.name.server)
				{
					if (!final_jig_value.peripheral.name.server)
					{
						DEBUG_INFO("Server passed\r\n");
					}
					final_jig_value.peripheral.name.server = 1;
				}

				if (current_jig_test_value.peripheral.name.input0_pass)
				{
					if (!final_jig_value.peripheral.name.input0_pass)
					{
						DEBUG_INFO("Input0 passed\r\n");
					}
					final_jig_value.peripheral.name.input0_pass = 1;
					function_test_result.result.alarm_ok = 1;
				}

				if (current_jig_test_value.peripheral.name.input1_pass)
				{
					if (!final_jig_value.peripheral.name.input1_pass)
					{
						DEBUG_INFO("Input1 passed\r\n");
					}
					function_test_result.result.fault_ok = 1;
					final_jig_value.peripheral.name.input1_pass = 1;
				}


				if (current_jig_test_value.peripheral.name.input2_pass)
				{
					if (!final_jig_value.peripheral.name.input2_pass)
					{
						DEBUG_INFO("Input2 passed\r\n");
					}
					final_jig_value.peripheral.name.input2_pass = 1;
				}

				if (current_jig_test_value.peripheral.name.input3_pass)
				{
					if (!final_jig_value.peripheral.name.input3_pass)
					{
						DEBUG_INFO("Input3 passed\r\n");
					}
					final_jig_value.peripheral.name.input3_pass = 1;
				}

				if (current_jig_test_value.peripheral.name.button_pass)
				{
					if (!final_jig_value.peripheral.name.button_pass)
					{
						DEBUG_INFO("Button passed\r\n");
					}
					final_jig_value.peripheral.name.button_pass = 1;
				}

				if (current_jig_test_value.peripheral.name.main_power_pass)
				{
					if (!final_jig_value.peripheral.name.main_power_pass)
					{
						DEBUG_INFO("Main power passed\r\n");
					}
					final_jig_value.peripheral.name.main_power_pass = 1;
				}

				if (current_jig_test_value.peripheral.name.backup_power_pass)
				{
					if (!final_jig_value.peripheral.name.backup_power_pass)
					{
						DEBUG_INFO("backup power passed\r\n");
					}
					final_jig_value.peripheral.name.backup_power_pass = 1;
				}

				if (current_jig_test_value.peripheral.name.eth)
				{
					if (!final_jig_value.peripheral.name.eth)
					{
						DEBUG_INFO("ETH OK\r\n");
					}
					function_test_result.result.eth_ok = 1;
				}

//				if (((final_jig_value.peripheral.value) == 0xFFFF) && (function_test_result.result.test_wd_ok != 1))
//				{
//					send_test_command((min_msg_t *)&reset_cmd_wd);
//				}

				if (((final_jig_value.peripheral.value) == 0xFFFF) && (function_test_result.result.test_wd_ok != 1))
				{
					DEBUG_INFO("Send watchdog test cmd\r\n");
					static uint32_t reset_wdt_timeout = 0;
					if (xTaskGetTickCount() - reset_wdt_timeout >= 2000)
					{
						send_test_command((min_msg_t *)&reset_cmd_wd);
						reset_wdt_timeout = xTaskGetTickCount();
					}
				}
			}

			all_peripheral_is_passed = is_peripheral_testcase_passed(&final_jig_value);
			if (is_test_running && all_peripheral_is_passed) // check whether pass test
			{
				DEBUG_INFO("All test case passed\r\n");
				new_data_need_send_to_server = true;
			}
			else
			{
				if (((now - test_process_begin_timestamp) > test_timeout_s) && is_test_running)
				{
					DEBUG_ERROR("Test timeout is over, send data to server\r\n");
					new_data_need_send_to_server = true;
					is_test_running = false;
				}
			}

			// Reset jig flag, no device plugged, ready for new devices
			m_found_jig_plugged = false;
		}

		if (new_data_need_send_to_server)
		{
			memset(m_last_remember_rs232_mac_addr, 0, 6);

			date_time_t date_time_buff;
			// Get current time
			HAL_RTC_GetTime(&hrtc, &sTimeToSend, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDateToSend, RTC_FORMAT_BIN);
			date_time_buff.day = sDateToSend.Date;
			date_time_buff.month = sDateToSend.Month;
			date_time_buff.year = sDateToSend.Year;
			date_time_buff.hour = sTimeToSend.Hours - 7;
			date_time_buff.minute = sTimeToSend.Minutes;
			date_time_buff.second = sTimeToSend.Seconds;

			// Set jig timestamp
			final_jig_value.timestamp = convert_date_time_to_second(&date_time_buff);
			final_jig_value.timestamp += 946684800; // add time from 1970 to 2000


			DEBUG_INFO("GET TIME: %d: %d: %d\r\n", (uint8_t)(sTimeToSend.Hours), (uint8_t)(sTimeToSend.Minutes), (uint8_t)(sTimeToSend.Seconds));
			DEBUG_INFO("GET date: %d: %d: %d\r\n", (uint8_t)(sDateToSend.Date), (uint8_t)(sDateToSend.Month), (uint8_t)(sDateToSend.Year));
			DEBUG_WARN("CALCULATED TIME : %u\r\n", final_jig_value.timestamp);

			// SEND TEST TIME TO MAIN BOARD
			time_tested.id = MIN_ID_SEND_TIMESTAMP;
			time_tested.len = 4;
			time_tested.payload = &(final_jig_value.timestamp);

			send_test_command(&time_tested);

			final_jig_value.device_type = current_jig_test_value.device_type;
			memcpy(final_jig_value.fw_version, current_jig_test_value.fw_version, 3);
			memcpy(final_jig_value.hw_version, current_jig_test_value.hw_version, 3);
			memcpy(final_jig_value.gsm_imei, current_jig_test_value.gsm_imei, 16);
			memcpy(final_jig_value.mac, current_jig_test_value.mac, 6);
			if (strlen(current_jig_test_value.sim_imei) > 10)
			{
				memcpy(final_jig_value.sim_imei, current_jig_test_value.sim_imei, 16);
			}
			else
			{
				memset(final_jig_value.sim_imei, 0, 16);
			}

			char *payload = pvPortMalloc(1024);
			if (payload)
			{
				/**
				 *
				 *
				 * [{
						"timestamp": "1656497320",
						"DeviceType": "BST01",
						"FirmwareVersion": "0.0.1",
						"HardwareVersion": "0.0.2",
						"GsmIMEI": "",
						"MacJIG": "485519B38B70",
						"ErrorResults": {
							"sim": false,
							"vGsm4V2": false,
							"eth": false,
							"wifi": true,
							"server": false,
							"mainPower": true,
							"backupPower": true,
							"buttonTest": false,
							"input1": true,
							"input2": true,
							"input3": true,
							"input4": true,
							"temperature": true,
							"alarmIn": true,
							"faultIn": true,
							"relay0": true,
							"relay1": true,
							"watchdog": true,
							"vbat": true,
							"v1v8": true,
							"v3v3": true,
							"vsys": true,
							"led1": true,
							"led2": true,
							"led3": true,
							"led4": true,
							"led5": true,
							"led6": true,
							"allPassed": false
						}}]
				*/
				uint32_t msg_size = build_json_msg_send_to_server(&final_jig_value,
																	&function_test_result,
																	&led_detect,
																	all_peripheral_is_passed,
																	payload);
				DEBUG_VERBOSE("%s\r\n", payload);

				http_msq_t http_message =
				{
					.data = payload,
					.len = msg_size,
					.need_free = true
				};

				if (!xQueueSend(m_http_msq, &http_message, 1))
				{
					DEBUG_WARN("No memory in http queue, save test data to flash\r\n");
					// Trim []
					char *p = strstr(http_message.data, "[");		// never fail
					p++;
					char *q = strstr(p, "]");

					save_offline_data(p, q-p);
					vPortFree(http_message.data);
				}
			}
			else
			{
				DEBUG_ERROR("No memory\r\n");
				vTaskDelay(1000);
				NVIC_SystemReset();
			}

			display_test_result_on_lcd(&function_test_result, &final_jig_value, &led_detect);
			is_test_running = false;
			new_data_need_send_to_server = false;

		}

		// Continuous send test command every 1s
		if ((now - last_tick) > 1000)
		{
			send_test_command((min_msg_t *)&test_cmd);
			last_tick = now;
		}

		if (is_test_running)
		{
			if ((now - last_vol_tick) > 1500)
			{
				test_point_voltage_monitor();
				last_vol_tick = now;
				monitor_led_test_point();
			}
		}

		uxBits = xEventGroupWaitBits(m_wdg_event_group,
									 defaultTaskB | cdcTaskB,
									 pdTRUE,
									 pdTRUE,
									 10);

		if ((uxBits & (defaultTaskB | cdcTaskB)) == (defaultTaskB | cdcTaskB))
		{
			HAL_IWDG_Refresh(&hiwdg);
		}

		osDelay(1);
	}
}
//*********************** LCD FUNCTION *******************************************//
void lcd_clr_screen(void)
{
	u8g2_ClearBuffer(&m_u8g2);
	u8g2_ClearDisplay(&m_u8g2);
	//	u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1);
	u8g2_FirstPage(&m_u8g2);
}
void lcd_display_header(const char *msg)
{
	//	snprintf(simulation_header_lcd, sizeof(simulation_header_lcd), "%s", msg);
	u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1); // can chon font cho header
	do
	{
		u8g2_DrawUTF8(&m_u8g2,
					  LCD_MEASURE_X_CENTER(u8g2_GetUTF8Width(&m_u8g2, msg)),
					  15,
					  msg);
	} while (u8g2_NextPage(&m_u8g2));
	u8g2_SendBuffer(&m_u8g2);
}

void lcd_display_error_at_pos(char *msg, u8g2_uint_t x, u8g2_uint_t y, const uint8_t *font)
{
	//	u8g2_ClearBuffer(&m_u8g2);
	u8g2_SetFont(&m_u8g2, font); // can chon font hien thi loi
	do
	{
		u8g2_DrawUTF8(&m_u8g2, x, y, msg);
	} while (u8g2_NextPage(&m_u8g2));
	u8g2_SendBuffer(&m_u8g2);
}

void lcd_display_content(const char *msg)
{
	lcd_clr_screen();

	lcd_display_header("JIG TEST");
	do
	{
		u8g2_DrawUTF8(&m_u8g2,
					  LCD_MEASURE_X_CENTER(u8g2_GetUTF8Width(&m_u8g2, msg)),
					  20+LCD_MEASURE_Y_CENTER(u8g2_GetMaxCharHeight(&m_u8g2)),
					  msg);
	} while (u8g2_NextPage(&m_u8g2));
	u8g2_SendBuffer(&m_u8g2);
}

void lcd_display_content_at_pos(const char *msg, u8g2_uint_t x, u8g2_uint_t y)
{
	//	lcd_clr_screen();
	//	u8g2_ClearBuffer(&m_u8g2);
	//	u8g2_FirstPage(&m_u8g2);
	char tmp[24];

	RTC_TimeTypeDef time;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);

	sprintf(tmp, "%02u:%02u:%02u", time.Hours, time.Minutes, time.Seconds);
	lcd_display_header(tmp);

	u8g2_SetFont(&m_u8g2, u8g2_font_6x13_tf);
	do
	{
		u8g2_DrawUTF8(&m_u8g2, x, y, msg);
	} while (u8g2_NextPage(&m_u8g2));
	u8g2_SendBuffer(&m_u8g2);
}

void display_test_result_on_lcd(func_test_t *res, jig_value_t *value, led_result_t *led_result)
{
	DEBUG_VERBOSE("DISPLAY ON LCD\r\n");
	uint32_t error_counter = 0;
	uint32_t index = 0;
	char tmp[256];
	lcd_clr_screen();
	if (all_peripheral_is_passed)
	{
		uint32_t test_time = HAL_GetTick() - test_process_begin_timestamp + 999;
		test_time /= 1000;

		char tmp[32];
		sprintf(tmp, "OK : %s", final_jig_value.gsm_imei);
		lcd_display_error_at_pos(tmp, 2, 32, u8g2_font_6x13_tf);


		LED1_R_OFF();
		LED1_G_ON();
		LED2_R_OFF();
		LED2_G_ON();
		return;
	}

	if (res->result.sim_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E1 ");
		error_counter++;
	}

	if (value->peripheral.name.eth == 0)
	{
		index += sprintf(tmp + index, "%s", "E2 ");
		error_counter++;
	}
	if (value->peripheral.name.server == 0)
	{
		index += sprintf(tmp + index, "%s", "E3 ");
		error_counter++;
	}
	if (value->peripheral.name.wifi == 0)
	{
		index += sprintf(tmp + index, "%s", "E4 ");
		error_counter++;
	}
	if (value->peripheral.name.gsm == 0)
	{
		index += sprintf(tmp + index, "%s", "E5 ");
		error_counter++;
	}
	if (value->peripheral.name.main_power_pass == 0)
	{
		index += sprintf(tmp + index, "%s", "E6 ");
		error_counter++;
	}
	if (value->peripheral.name.backup_power_pass == 0)
	{
		index += sprintf(tmp + index, "%s", "E7 ");
		error_counter++;
	}

	if (value->peripheral.name.button_pass == 0)
	{
		index += sprintf(tmp + index, "%s", "E8 ");
		error_counter++;
	}

	if (res->result.rs232 == 0)
	{
		index += sprintf(tmp+index, "%s", "E9 ");
		error_counter++;
	}
	if (res->result.rs485 == 0)
	{
		index += sprintf(tmp + index, "%s", "E10 ");
		error_counter++;
	}

	if ((res->result.relay0_ok == 0))
	{
		index += sprintf(tmp + index, "%s", "E11 ");
		error_counter++;
	}

	if ((res->result.relay1_ok == 0))
	{
		index += sprintf(tmp + index, "%s", "E12 ");
		error_counter++;
	}

	if (!led_result->res.led1_ok)
	{
		index += sprintf(tmp + index, "%s", "E13 ");
		error_counter++;
	}
	if (!led_result->res.led2_ok)
	{
		index += sprintf(tmp + index, "%s", "E14 ");
		error_counter++;
	}
	if (!led_result->res.led3_ok)
	{
		index += sprintf(tmp + index, "%s", "E15 ");
		error_counter++;
	}

	if (!led_result->res.led4_ok)
	{
		index += sprintf(tmp + index, "%s", "E16 ");
		error_counter++;
	}

	if (!led_result->res.led5_ok)
	{
		index += sprintf(tmp + index, "%s", "E17 ");
		error_counter++;
	}

	if (!led_result->res.led6_ok)
	{
		index += sprintf(tmp + index, "%s", "E18 ");
		error_counter++;
	}


	if (res->result.test_wd_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E19 ");
		error_counter++;
	}

	if ((res->result.v1v8_ok == 0))
	{
		index += sprintf(tmp + index, "%s", "E20 ");
		error_counter++;
	}

	if (res->result.v3v3_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E21 ");
		error_counter++;
	}

	if (res->result.vbat_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E22 ");
		error_counter++;
	}

	if (res->result.vbatrf_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E23 ");
		error_counter++;
	}

	if (res->result.vgsm4v2_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E24 ");
		error_counter++;
	}

	if (res->result.vsys_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E25 ");
		error_counter++;
	}

	if (res->result.charge_ok == 0)
	{
		index += sprintf(tmp + index, "%s", "E26 ");
		error_counter++;
	}

	if (value->peripheral.name.input0_pass == 0)
	{
		index += sprintf(tmp + index, "%s", "E27 ");
		error_counter++;
	}

	if (value->peripheral.name.input1_pass == 0)
	{
		index += sprintf(tmp + index, "%s", "E28 ");
		error_counter++;
	}

	if (value->peripheral.name.input2_pass == 0)
	{
		index += sprintf(tmp + index, "%s", "E29 ");
		error_counter++;
	}

	if (value->peripheral.name.input3_pass == 0)
	{
		index += sprintf(tmp + index, "%s", "E30 ");
		error_counter++;
	}

	DEBUG_INFO("Total %u error\r\n", error_counter);

	char error_code[7][48];
	memset(error_code, 0, sizeof(error_code));
	char *p = tmp;
	uint32_t display_row = 0;
	for (uint32_t row = 0; row < (index + 5) / 6; row++)		// 4 row * 6 error code
	{
		// Count number of space
		uint8_t column = 0;
		uint8_t char_count = 0;
		while (column < 6 && *p)
		{
			if (*p == ' ')
			{
				column++;
			}
			error_code[display_row][char_count++] = *p;
			p++;
		}
		if (strlen(error_code[display_row]))
		{
			DEBUG_INFO("[%d] %s\r\n", display_row, error_code[display_row]);
			display_row++;
		}
	}

	char header[24];
	sprintf(header, "CÓ %lu LỖI", error_counter);
	lcd_display_header(header);

	const uint8_t *font = u8g2_font_5x7_tf;
	uint32_t space_per_row = 12;
	uint32_t offset = 25;
	if (display_row < 4)
	{
		font = u8g2_font_5x8_tf;
		space_per_row = 14;
		offset = 30;
	}
	for (uint32_t i = 0; i < display_row; i++)
	{
		lcd_display_error_at_pos(error_code[i], 5 , offset + space_per_row*i, font);
	}

	LED1_R_ON();
	LED1_G_OFF();
	LED2_R_ON();
	LED2_G_OFF();
	indicator_buzzer_beeps(2);
}


void send_test_command(min_msg_t *test_command)
{
	min_send_frame(&m_min_context, test_command);
}

void min_rx_callback(void *min_context, min_msg_t *frame)
{
	function_test_result.result.rs232 = 1;
	m_found_jig_plugged = true;

	switch (frame->id)
	{
	case MIN_ID_RS232_ENTER_TEST_MODE:
		DEBUG_VERBOSE("ENTER TEST MODE");

		memcpy(&current_jig_test_value, (jig_value_t *)frame->payload, sizeof(jig_value_t));
		memcpy(rs232_incomming_mac_addr, current_jig_test_value.mac, 6);
		if (is_test_running)
		{
			if (strlen(current_jig_test_value.sim_imei) > 10)
			{
				memcpy(final_jig_value.sim_imei, current_jig_test_value.sim_imei, 16);
			}

			if (strlen(current_jig_test_value.gsm_imei) > 10 && strlen(final_jig_value.gsm_imei) < 10)
			{
				memcpy(final_jig_value.gsm_imei, current_jig_test_value.gsm_imei, 15);
				DEBUG_INFO("IMEI %s\r\n", final_jig_value.gsm_imei);
			}
		}

		if (!m_found_jig_plugged)
		{
			DEBUG_INFO("MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
						rs232_incomming_mac_addr[0],
						rs232_incomming_mac_addr[1],
						rs232_incomming_mac_addr[2],
						rs232_incomming_mac_addr[3],
						rs232_incomming_mac_addr[4],
						rs232_incomming_mac_addr[5]);
		}
		break;

	case MIN_ID_RS232_ESP32_RESET:
		if (is_test_running)
		{
			if (!function_test_result.result.test_wd_ok)
			{
				DEBUG_INFO("Watchdog passed\r\n");
			}
			function_test_result.result.test_wd_ok = 1;
		}
		else
		{
			DEBUG_WARN("Main board reset\r\n");
		}
		break;

	default:
		break;
	}
}
bool RS232_tx(void *ctx, uint8_t byte)
{
	(void)ctx;
	putChar(USART6, byte);
	return true;
}

void test_point_voltage_monitor(void)
{
	uint8_t error_counter;
	uint32_t voltage[5];

	// Get ADC data
	for (uint8_t i = 0; i < 5; i++)
	{
		voltage[i] = (m_adc_result[i] * 3300 / 4095);
		voltage[i] = voltage[i] * 2;		// resistor div
	}

	DEBUG_VERBOSE("1V8 : %d mV\r\n", voltage[1]);
	DEBUG_VERBOSE("3V3 : %d mV\r\n", voltage[2]);
	DEBUG_VERBOSE("VSYS : %d mV\r\n", voltage[3]);
	DEBUG_VERBOSE("VBAT : %d mV\r\n", voltage[4]);

	error_counter = 0;
	if (voltage_info.vbatrf_min <= voltage[0] && voltage[0] <= voltage_info.vbatrf_max)
	{
		if (!function_test_result.result.vbatrf_ok)
		{
			DEBUG_INFO("VBAT_RF OK\r\n");
		}
		function_test_result.result.vbatrf_ok = 1;
		error_counter++;
	}
	else
	{
		DEBUG_INFO("VBATRF : %dmV, range [%u - %u]\r\n", voltage[0], voltage_info.vbatrf_min, voltage_info.vbatrf_max);
	}

	if (voltage_info.v1v8_min <= voltage[1] && voltage[1] <= voltage_info.v1v8_max)
	{
		if (!function_test_result.result.v1v8_ok)
		{
			DEBUG_INFO("1V8 OK\r\n");
		}
		function_test_result.result.v1v8_ok = 1;
		error_counter++;
	}
	else
	{
		DEBUG_INFO("1V8 : %d mV, range [%u - %u]\r\n", voltage[1], voltage_info.v1v8_min, voltage_info.v1v8_max);
	}

	if (voltage_info.v3v3_min <= voltage[2] && voltage[2] <= voltage_info.v3v3_max)
	{
		if (!function_test_result.result.v3v3_ok)
		{
			DEBUG_INFO("3V3 OK\r\n");
		}
		function_test_result.result.v3v3_ok = 1;
		error_counter++;
	}
	else
	{
		DEBUG_INFO("3V3 : %d mV, range [%u - %u]\r\n", voltage[2], voltage_info.v3v3_min, voltage_info.v3v3_max);
	}

	if (voltage_info.vsys_min <= voltage[3] && voltage[3] <= voltage_info.vsys_max)
	{
		if (!function_test_result.result.vsys_ok)
		{
			DEBUG_INFO("VSYS OK\r\n");
		}
		function_test_result.result.vsys_ok = 1;
		error_counter++;
	}
	else
	{
		DEBUG_INFO("VSYS : %d mV, range [%u - %u]\r\n", voltage[3], voltage_info.vsys_min, voltage_info.vsys_max);
	}


	if (voltage_info.vbat_min <= voltage[4] && voltage[4] <= voltage_info.vbat_max)
	{
		if (!function_test_result.result.vbat_ok)
		{
			DEBUG_INFO("VBAT OK\r\n");
		}
		function_test_result.result.vbat_ok = 1;
		error_counter++;
	}
	else
	{
		DEBUG_INFO("VBAT : %d mV, range [%u - %u]\r\n", voltage[4], voltage_info.vbat_min, voltage_info.vbat_max);
	}

	 function_test_result.result.vgsm4v2_ok = function_test_result.result.vbatrf_ok;

	if (error_counter == 5)
	{
		DEBUG_VERBOSE("VOLTAGE OK\r\n");
	}
	else
	{
		if (!(function_test_result.result.vsys_ok
			&& function_test_result.result.vbat_ok
			&& function_test_result.result.vbatrf_ok
			&& function_test_result.result.v1v8_ok
			&& function_test_result.result.v3v3_ok
			&& function_test_result.result.vgsm4v2_ok))
		{
			DEBUG_WARN("VOLTAGE FAIL\r\n");
		}
	}

	// Read charger status pin
	if (HAL_GPIO_ReadPin(STATUS_GPIO_Port, STATUS_Pin) == 0)
	{
		if (!function_test_result.result.charge_ok)
		{
			DEBUG_INFO("CHARGE OK");
		}
		function_test_result.result.charge_ok = 1;
	}
	else
	{
		if (function_test_result.result.charge_ok)
		{
			DEBUG_WARN("CHARGE failed");
		}
		function_test_result.result.charge_ok = 0;
	}
}

static bool is_peripheral_testcase_passed(jig_value_t *value)
{
	/* Get sim status */
	if (strlen(value->sim_imei) >= 10)
	{
		if (!function_test_result.result.sim_ok)
		{
			DEBUG_INFO("Sim IMEI %s\r\n", value->sim_imei);
		}
		function_test_result.result.sim_ok = 1;
	}
	else
	{
		if (function_test_result.result.sim_ok)
		{
			DEBUG_INFO("Sim error\r\n");
		}
	}

	/* Get temperature status */
	if (25 <= value->temperature && value->temperature <= 50)
	{
		if (!function_test_result.result.temper_ok)
		{
			DEBUG_INFO("TEMPER IS OK \r\n");
		}
		function_test_result.result.temper_ok = 1;
	}
	else
	{
		if (function_test_result.result.temper_ok)
		{
			DEBUG_INFO("TEMPER IS not OK \r\n");
		}
		function_test_result.result.temper_ok = 0;
	}

	// Check if all testcase is passed
	if (function_test_result.result.rs232
		&& function_test_result.result.rs485
		&& function_test_result.result.relay0_ok
		&& function_test_result.result.relay1_ok
		&& function_test_result.result.v1v8_ok
		&& function_test_result.result.v3v3_ok
		&& function_test_result.result.vbat_ok
		&& function_test_result.result.vbatrf_ok
		&& function_test_result.result.vsys_ok
		&& function_test_result.result.sim_ok
		&& function_test_result.result.test_wd_ok
		&& function_test_result.result.temper_ok
		&& function_test_result.result.charge_ok
		&& function_test_result.result.eth_ok)
	{
		DEBUG_INFO("Test peripheral passed\r\n");
		return true;
	}
	else
	{
		return false;
	}
	return false;
}

// copy lien tuc gia tri tu led_detect sang led_detect_final
//  kiem tra sau 3s neu chua dat thi coi nhu la chua dat

void monitor_led_test_point(void)
{
	led_status[LR1] = HAL_GPIO_ReadPin(IN_LR1_GPIO_Port, IN_LR1_Pin);
	led_status[LR2] = HAL_GPIO_ReadPin(IN_LR2_GPIO_Port, IN_LR2_Pin);
	led_status[LR3] = HAL_GPIO_ReadPin(IN_LR3_GPIO_Port, IN_LR3_Pin);
	led_status[LR4] = HAL_GPIO_ReadPin(IN_LR4_GPIO_Port, IN_LR4_Pin);
	led_status[LR5] = HAL_GPIO_ReadPin(IN_LR5_GPIO_Port, IN_LR5_Pin);
	led_status[LR6] = HAL_GPIO_ReadPin(IN_LR6_GPIO_Port, IN_LR6_Pin);
	led_status[LB1] = HAL_GPIO_ReadPin(IN_LB1_GPIO_Port, IN_LB1_Pin);
	led_status[LB2] = HAL_GPIO_ReadPin(IN_LB2_GPIO_Port, IN_LB2_Pin);
	led_status[LB3] = HAL_GPIO_ReadPin(IN_LB3_GPIO_Port, IN_LB3_Pin);
	led_status[LB4] = HAL_GPIO_ReadPin(IN_LB4_GPIO_Port, IN_LB4_Pin);
	led_status[LB5] = HAL_GPIO_ReadPin(IN_LB5_GPIO_Port, IN_LB5_Pin);
	led_status[LB6] = HAL_GPIO_ReadPin(IN_LB6_GPIO_Port, IN_LB6_Pin);

	for (uint8_t i = LR1; i < 6; i++)
	{
		if ((led_status[i] == 0) && (led_status[i + 6] == 0))
		{
			if (!(led_detect.value & (1 << i)))
			{
				DEBUG_INFO("LED%d passed\r\n", i + 1);
			}
			led_detect.value |= (1 << i);
		}
	}
}

void button_initialize(uint32_t button_num)
{
}

uint32_t btn_read(uint32_t pin)
{
	if (pin == 0)
	{
		return HAL_GPIO_ReadPin(BT_IN_1_GPIO_Port, BT_IN_1_Pin);
	}
	else if (pin == 1)
	{
		return HAL_GPIO_ReadPin(BT_IN_2_GPIO_Port, BT_IN_2_Pin);
	}
	return 1;
}

void on_btn_pressed(int number, int event, void *pData)
{
	DEBUG_INFO("On button %d pressed\r\n", number);
	if (number == 0)
	{
		xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_BT_IN1_PRESSED);
	}
	else if (number == 1)
	{
		xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_BT_IN2_PRESSED);
	}
	//    else
	//    {
	//        //xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_BUTTON_1_PRESSED);
	//    }
}

void on_btn_release(int number, int event, void *pData)
{
	DEBUG_VERBOSE("On button %d release\r\n", number);
	if (number == 0)
	{
		xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_BT_IN1_PRESSED);
	}
	else if (number == 1)
	{
		xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_BT_IN2_PRESSED);
	}
	//    else if (number == 2)
	//    {
	//      xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_BUTTON_2_PRESSED);
	//    }
}

void on_btn_hold(int number, int event, void *pData)
{
	DEBUG_INFO("On button %d pair hold, enter pair mode\r\n", number);
}

static void on_btn_hold_so_long(int index, int event, void *pData)
{
	DEBUG_INFO("Button hold so long\r\n");
}

static void convert_second_to_date_time(uint32_t sec, date_time_t *t, uint8_t Calyear)
{
	uint16_t day;
	uint8_t year;
	uint16_t days_of_year;
	uint8_t leap400;
	uint8_t month;

	t->second = sec % 60;
	sec /= 60;
	t->minute = sec % 60;
	sec /= 60;
	t->hour = sec % 24;

	if (Calyear == 0)
		return;

	day = (uint16_t)(sec / 24);

	year = FIRSTYEAR % 100;					   // 0..99
	leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3); // 4, 3, 2, 1

	for (;;)
	{
		days_of_year = 365;
		if ((year & 3) == 0)
		{
			days_of_year = 366; // leap year
			if (year == 0 || year == 100 || year == 200)
			{ // 100 year exception
				if (--leap400)
				{ // 400 year exception
					days_of_year = 365;
				}
			}
		}
		if (day < days_of_year)
		{
			break;
		}
		day -= days_of_year;
		year++; // 00..136 / 99..235
	}
	t->year = year + FIRSTYEAR / 100 * 100 - 2000; // + century
	if (days_of_year & 1 && day > 58)
	{		   // no leap year and after 28.2.
		day++; // skip 29.2.
	}

	for (month = 1; day >= day_in_month[month - 1]; month++)
	{
		day -= day_in_month[month - 1];
	}

	t->month = month; // 1..12
	t->day = day + 1; // 1..31
}

static uint8_t get_weekday(date_time_t time)
{
	time.weekday = (time.day +=
					time.month < 3 ? time.year-- : time.year - 2,
					23 * time.month / 9 + time.day + 4 + time.year / 4 -
						time.year / 100 + time.year / 400);
	return time.weekday % 7;
}

static uint32_t convert_date_time_to_second(date_time_t *t)
{
	uint8_t i;
	uint32_t result = 0;
	uint16_t idx, year;

	year = t->year + 2000;

	/* Calculate days of years before */
	result = (uint32_t)year * 365;
	if (t->year >= 1)
	{
		result += (year + 3) / 4;
		result -= (year - 1) / 100;
		result += (year - 1) / 400;
	}

	/* Start with 2000 a.d. */
	result -= 730485UL;

	/* Make month an array index */
	idx = t->month - 1;

	/* Loop thru each month, adding the days */
	for (i = 0; i < idx; i++)
	{
		result += day_in_month[i];
	}

	/* Leap year? adjust February */
	if (!(year % 400 == 0 || (year % 4 == 0 && year % 100 != 0)))
	{
		if (t->month > 2)
		{
			result--;
		}
	}

	/* Add remaining days */
	result += t->day;

	/* Convert to seconds, add all the other stuff */
	result = (result - 1) * 86400L + (uint32_t)t->hour * 3600 +
			 (uint32_t)t->minute * 60 + t->second;
	return result;
}


static void initialize_stnp(void)
{
	static bool sntp_start = false;
	if (sntp_start == false)
	{
		sntp_start = true;
		sntp_setoperatingmode(SNTP_OPMODE_POLL);
		sntp_setservername(0, "pool.ntp.org");
		sntp_init();
		DEBUG_INFO("Initialize stnp\r\n");
	}
}

void lwip_sntp_recv_cb(uint32_t time)
{
	if (time == 0)
	{
		DEBUG_ERROR("NTP ERROR\r\n");
	}
	else
	{
		//		set_rtc_date_time()
		DEBUG_INFO("It's been %u second from 1970\r\n", time);

		time_t rawtime = time;
		struct tm ts;

		// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
		ts = *localtime(&rawtime);

		DEBUG_INFO("Time now: %02d:%02d:%02d  %02d-%02d-%04d\r\n",
					ts.tm_hour, ts.tm_min, ts.tm_sec,
					ts.tm_mday, ts.tm_mon + 1,
					ts.tm_year + 1900);

		date_time.year = (ts.tm_year + 1900) % 2000; // year - 1900
		date_time.month = ts.tm_mon + 1;			 // month, where 0 = jan
		date_time.day = ts.tm_mday;					 // day of the month
		date_time.hour = ts.tm_hour;
		date_time.minute = ts.tm_min;
		date_time.second = ts.tm_sec;

		uint32_t time_buff = convert_date_time_to_second(&date_time) + 25200; // time for gmt +7

		convert_second_to_date_time(time_buff, &date_time, 1);


		m_rtc_time.Hours = date_time.hour;
		m_rtc_time.Minutes = date_time.minute;
		m_rtc_time.Seconds = date_time.second;
		m_rtc_date.Year = date_time.year;
		m_rtc_date.Month = date_time.month;
		m_rtc_date.Date = date_time.day;
		m_rtc_date.WeekDay = get_weekday(date_time);

		m_last_time_sync_timestamp_data = m_rtc_time;

		set_rtc_date_time(&m_rtc_time, &m_rtc_date);

		// Set time and date to RTC
		HAL_RTC_GetTime(&hrtc, &m_rtc_time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &m_rtc_date, RTC_FORMAT_BIN);
	}
}

void set_rtc_date_time(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
	if (HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN) != HAL_OK)
	{
		DEBUG_ERROR("Set time failed\r\n");
		vTaskDelay(1000);
		Error_Handler();
	}

	if (HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN) != HAL_OK)
	{
		DEBUG_ERROR("Set date failed\r\n");
		vTaskDelay(1000);
		Error_Handler();
	}
}


static lwrb_t m_ringbuffer_usb_cdc_tx;
static uint8_t m_lwrb_tx_raw_buffer[USB_CDC_TX_RING_BUFFER_SIZE];
void tusb_read_callback(void)
{
	while (1)
	{
		uint8_t usb_ch;
		uint32_t count = tud_cdc_read(&usb_ch, 1);
		if (count)
		{
			app_cli_poll(usb_ch);
		}
		else
		{
			break;
		}
	}
	return;
}

/* Send data to CDC port */
uint32_t cdc_tx(const void *buffer, uint32_t size)
{
	uint32_t written = lwrb_write(&m_ringbuffer_usb_cdc_tx, buffer, size);
	if (written < size)
	{
		lwrb_skip(&m_ringbuffer_usb_cdc_tx, size - written);
		lwrb_write(&m_ringbuffer_usb_cdc_tx, buffer+written, size - written);
	}
	return size;
}


void usb_task(void *params)
{
	tusb_init_flag = tusb_init();
	// Create CDC task
	(void)xTaskCreateStatic(cdc_task, "cdc", 256, NULL, 1, cdc_stack, &cdc_taskdef); // pio =2

	while (1)
	{
		tud_task();
	}
}

void cdc_task(void *params)
{
#if RESET_CDC_DEBUG_BUFFER_WHEN_USB_DISCONNECT == 0
	app_debug_register_callback_print(cdc_tx);
#else
	static bool m_cdc_debug_register = false;
#endif

	DEBUG_INFO("ENTER CDC TASK\r\n");
	lwrb_init(&m_ringbuffer_usb_cdc_tx, m_lwrb_tx_raw_buffer, USB_CDC_TX_RING_BUFFER_SIZE);
	for (;;)
	{
		//	    // connected() check for DTR bit
		//	    // Most but not all terminal client set this when making connection
		//		tusb_read_callback();
		if (tud_cdc_connected())
		{
#if RESET_CDC_DEBUG_BUFFER_WHEN_USB_DISCONNECT
			if (m_cdc_debug_register == false)
			{
				m_cdc_debug_register = true;
				app_debug_register_callback_print(cdc_tx);
			}
#endif
			if (tud_cdc_available())
			{
				tusb_read_callback();
			}
		}
		else
		{
#if RESET_CDC_DEBUG_BUFFER_WHEN_USB_DISCONNECT
			if (m_cdc_debug_register)
			{
				m_cdc_debug_register = false;
				app_debug_unregister_callback_print(cdc_tx);
				// Flush all cdc tx buffer
				char tmp[1];
				while (lwrb_read(&m_ringbuffer_usb_cdc_tx, tmp, 1))
				{
				}
			}
#endif
		}

		char buffer[(TUD_OPT_HIGH_SPEED ? 512 : 64)];
		uint32_t size;
		while (tud_cdc_connected())
		{
			uint32_t avai = tud_cdc_write_available();
			if (avai >= sizeof(buffer))
			{
				avai = sizeof(buffer);
			}
			size = lwrb_read(&m_ringbuffer_usb_cdc_tx, buffer, avai);
			if (size)
			{
				tud_cdc_write(buffer, size);
				tud_cdc_write_flush();
			}
			else
			{
				break;
			}
		}
		xEventGroupSetBits(m_wdg_event_group, cdcTaskB);
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
//*************************************************************************************************************************/

//*****************************************  ETHERNET function ***********************************************************//
void Netif_Config(bool restart)
{
	ip4_addr_t ipaddr;
	ip4_addr_t netmask;
	ip4_addr_t gw;
	/* IP addresses initialization with DHCP (IPv4) */
	ipaddr.addr = 0;
	netmask.addr = 0;
	gw.addr = 0;
	if (restart)
	{
		netif_remove(&g_netif);
		DEBUG_INFO("NET IF REMOVE \r\n");
		/* Start DHCP negotiation for a network interface (IPv4) */
	}
	/* add the network interface (IPv4/IPv6) with RTOS */
	netif_add(&g_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input); //&tcpip_input =>null

	/* Registers the default network interface */
	netif_set_default(&g_netif);
	netif_set_link_callback(&g_netif, ethernet_link_status_updated);
	if (netif_is_link_up(&g_netif))
	{
		/* When the netif is fully configured this function must be called */
		netif_set_up(&g_netif);
	}
	else
	{
		/* When the netif link is down this function must be called */
		netif_set_down(&g_netif);
	}
	app_ethernet_notification(&g_netif);
	/* Set the link callback function, this function is called on change of link status*/
}

//***************************************************************************************************//

//*********************** json encode*********************************//
int16_t build_json_msg_send_to_server(jig_value_t *value,
									func_test_t *test,
									led_result_t *led,
									bool all_passed,
									char *output)
{
	int16_t index = 0;

	char *passed = "true";
	char *failed = "false";

	index += sprintf(output + index, "[{\"timestamp\":\"%lu\",", value->timestamp);

	// Version control
	const char *device_type[] = {"BST01-Pro", "BST01", "SF68", "NA"};
	const char *p = device_type[3];

	if (value->device_type < 3)
	{
		p = device_type[value->device_type];
	}

	index += sprintf(output + index, "\"DeviceType\":\"%s\",", p);

	index += sprintf(output + index, "\"FirmwareVersion\":\"%d.%d.%d\",",
					value->fw_version[0],
					value->fw_version[1],
					value->fw_version[2]);
	index += sprintf(output + index, "\"HardwareVersion\":\"%d.%d.%d\",",
					value->hw_version[0],
					value->hw_version[1],
					value->hw_version[2]);
	// IMEI
	index += sprintf(output + index, "\"GsmIMEI\":\"%s\",", value->gsm_imei);

	// MAC
	index += sprintf(output + index, "\"MacJIG\":\"%02X%02X%02X%02X%02X%02X\",",
					value->mac[0],
					value->mac[1],
					value->mac[2],
					value->mac[3],
					value->mac[4],
					value->mac[5]);
	index += sprintf(output + index, "\"ErrorResults\":{");
	index += sprintf(output + index, "\"sim\":%s,", test->result.sim_ok ? passed : failed);
	index += sprintf(output + index, "\"vGsm4V2\":%s,", test->result.vgsm4v2_ok ? passed : failed);
	index += sprintf(output + index, "\"eth\":%s,", value->peripheral.name.eth ? passed : failed);
	index += sprintf(output + index, "\"wifi\":%s,", value->peripheral.name.wifi ? passed : failed);
	index += sprintf(output + index, "\"server\":%s,", value->peripheral.name.server ? passed : failed);
	index += sprintf(output + index, "\"mainPower\":%s,", value->peripheral.name.main_power_pass ? passed : failed);
	index += sprintf(output + index, "\"backupPower\":%s,", value->peripheral.name.backup_power_pass ? passed : failed);
	index += sprintf(output + index, "\"buttonTest\":%s,", value->peripheral.name.button_pass ? passed : failed);
	index += sprintf(output + index, "\"input1\":%s,", value->peripheral.name.input0_pass ? passed : failed);
	index += sprintf(output + index, "\"input2\":%s,", value->peripheral.name.input1_pass ? passed : failed);
	index += sprintf(output + index, "\"input3\":%s,", value->peripheral.name.input2_pass ? passed : failed);
	index += sprintf(output + index, "\"input4\":%s,", value->peripheral.name.input3_pass ? passed : failed);
	index += sprintf(output + index, "\"temperature\":%s,", test->result.temper_ok ? passed : failed);
	index += sprintf(output + index, "\"alarmIn\":%s,", test->result.alarm_ok ? passed : failed);
	index += sprintf(output + index, "\"faultIn\":%s,", test->result.fault_ok ? passed : failed);
	index += sprintf(output + index, "\"relay0\":%s,", test->result.relay0_ok ? passed : failed);
	index += sprintf(output + index, "\"relay1\":%s,", test->result.relay1_ok ? passed : failed);
	index += sprintf(output + index, "\"watchdog\":%s,", test->result.test_wd_ok ? passed : failed);
	index += sprintf(output + index, "\"vbat\":%s,", test->result.vbat_ok ? passed : failed);
	index += sprintf(output + index, "\"v1v8\":%s,", test->result.v1v8_ok ? passed : failed);
	index += sprintf(output + index, "\"v3v3\":%s,", test->result.v3v3_ok ? passed : failed);
	index += sprintf(output + index, "\"vsys\":%s,", test->result.vsys_ok ? passed : failed);
	index += sprintf(output + index, "\"led1\":%s,", led->res.led1_ok ? passed : failed);
	index += sprintf(output + index, "\"led2\":%s,", led->res.led2_ok ? passed : failed);
	index += sprintf(output + index, "\"led3\":%s,", led->res.led3_ok ? passed : failed);
	index += sprintf(output + index, "\"led4\":%s,", led->res.led4_ok ? passed : failed);
	index += sprintf(output + index, "\"led5\":%s,", led->res.led5_ok ? passed : failed);
	index += sprintf(output + index, "\"led6\":%s,", led->res.led6_ok ? passed : failed);
	index += sprintf(output + index, "\"allPassed\":%s}}]", all_passed ? passed : failed);

	return index;
}


int32_t USB_puts(char *msg)
{
	uint32_t len = strlen(msg);

	cdc_tx((uint8_t *)msg, len);
	return len;
}

void cli_cdc_tx(uint8_t *buffer, uint32_t size)
{
	cdc_tx(buffer, size);
}

int cli_cdc_puts(const char *msg)
{
	uint32_t len = strlen(msg);
	cdc_tx((uint8_t *)msg, len);
	return len;
}

void send_test_cmd(void)
{
	min_msg_t reset_cmd_wd =
	{
		.id = MIN_ID_TEST_WATCHDOG,
		.len = 0,
		.payload = NULL
	};
	send_test_command(&reset_cmd_wd);
	DEBUG_INFO("TEST WD\r\n");
}

void rs232_rx_callback(uint8_t ch)
{
	lwrb_write(&m_ringbuffer_host_rx, &ch, 1);
}

void rs485_rx_callback(uint8_t tmp)
{
	rs485_idle_line_detect = 5;
	rs485_ringbuffer[rs485_buffer_index++] = tmp;
	if (rs485_buffer_index >= 255)
	{
		rs485_buffer_index = 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
	/* NOTE: This function Should not be modified, when the callback is needed,
			 the HAL_GPIO_EXTI_Callback could be implemented in the user file
	 */
	if (GPIO_Pin == RELAY_NO_Pin)
	{
		relay0_toggle_counter++;
		if (relay0_toggle_counter > 1)
		{
			// Chi co 1 relay
			function_test_result.result.relay0_ok = 1;
			function_test_result.result.relay1_ok = 1;
			relay0_toggle_counter = 0;
		}
	}
	else if (GPIO_Pin == RELAY_NC_Pin)
	{
		// Chi co 1 relay
		relay0_toggle_counter++;
		if (relay0_toggle_counter > 1)
		{
			relay1_toggle_counter = 0;
			function_test_result.result.relay0_ok = 1;
			function_test_result.result.relay1_ok = 1;
		}
	}
}

void sys_delay_ms(uint32_t ms)
{
	DEBUG_INFO("Delay %ums\r\n", ms);
	vTaskDelay(ms);
}
static void delay_ns(uint32_t ns)
{
	for (uint32_t i = 0; i < ns * 2; i++)
	{
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}
uint8_t u8g2_gpio_8080_update_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
	switch (msg)
	{
	// Initialize SPI peripheral
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
	{
		HAL_GPIO_WritePin(PWM_LCD_GPIO_Port, PWM_LCD_Pin, 1);
	}
	break;

	case U8X8_MSG_DELAY_MILLI:
	{
		sys_delay_ms(arg_int);
	}
	break;

	// Function which delays 10us
	case U8X8_MSG_DELAY_10MICRO:
	{
		for (volatile uint32_t n = 0; n < 600 * arg_int; n++)
		{
			__NOP();
			__NOP();
		}
	}
	break;
	// Function which delays 100ns
	case U8X8_MSG_DELAY_100NANO:
	{
		delay_ns(100 * arg_int);
	}
	break;

	case U8X8_MSG_DELAY_NANO:
	{
		delay_ns(arg_int);
	}
	break;

	// Function to define the logic level of the clockline
	case U8X8_MSG_GPIO_D0:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D0_GPIO_Port, LCD_D0_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D0_GPIO_Port, LCD_D0_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_D1:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_D2:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D2_GPIO_Port, LCD_D2_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D2_GPIO_Port, LCD_D2_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_D3:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D3_GPIO_Port, LCD_D3_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D3_GPIO_Port, LCD_D3_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_D4:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_D5:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_D6:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_D7:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_E:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_CS:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, 1);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, 0);
		}
	}
	break;

	case U8X8_MSG_GPIO_DC:
	{
		if (arg_int)
		{
			HAL_GPIO_WritePin(LCD_DI_GPIO_Port, LCD_DI_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(LCD_DI_GPIO_Port, LCD_DI_Pin, 1);
		}
	}
	break;

	case U8X8_MSG_GPIO_RESET:
	{
	}
	break;

	case U8X8_MSG_GPIO_MENU_SELECT:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
	}
	break;

	case U8X8_MSG_GPIO_MENU_NEXT:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
	}
	break;

	case U8X8_MSG_GPIO_MENU_PREV:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
	}
	break;

	case U8X8_MSG_GPIO_MENU_HOME:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
	}
	break;

	default:
		DEBUG_ERROR("Unhandled case %d\r\n", msg);
		return 0; // A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}


/* USER CODE END Application */
