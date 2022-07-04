#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdint.h>
#include <stdbool.h>
//#include "gd32e23x.h"
#include "main.h"
//
//#define BOARD_HW_LED_OFF         1
//#define BOARD_HW_LED_ON          0
//
//#define BOARD_HW_SW_OFF         0
//#define BOARD_HW_SW_ON          1
//
///* External wdt reset */
//#define BOARD_HW_EXT_WDT_PORT				GPIOA
//#define BOARD_HW_EXT_WDT_PIN				GPIO_PIN_4
//
/* LCD Pin*/
#define BOARD_HW_LCD_RS_PORT				GPIOE
#define BOARD_HW_LCD_RS_PIN					GPIO_PIN_1

#define BOARD_HW_LCD_RW_PORT				GPIOE
#define BOARD_HW_LCD_RW_PIN					GPIO_PIN_0

#define BOARD_HW_LCD_EN_PORT				GPIOB
#define BOARD_HW_LCD_EN_PIN					GPIO_PIN_9

#define BOARD_HW_LCD_D0_PORT				GPIOE
#define BOARD_HW_LCD_D0_PIN					GPIO_PIN_2

#define BOARD_HW_LCD_D1_PORT				GPIOE
#define BOARD_HW_LCD_D1_PIN					GPIO_PIN_3

#define BOARD_HW_LCD_D2_PORT				GPIOE
#define BOARD_HW_LCD_D2_PIN					GPIO_PIN_4

#define BOARD_HW_LCD_D3_PORT				GPIOE
#define BOARD_HW_LCD_D3_PIN					GPIO_PIN_5

#define BOARD_HW_LCD_D4_PORT				GPIOE
#define BOARD_HW_LCD_D4_PIN					GPIO_PIN_6

#define BOARD_HW_LCD_D5_PORT				GPIOC
#define BOARD_HW_LCD_D5_PIN					GPIO_PIN_13

#define BOARD_HW_LCD_D6_PORT				GPIOC
#define BOARD_HW_LCD_D6_PIN					GPIO_PIN_14

#define BOARD_HW_LCD_D7_PORT				GPIOC
#define BOARD_HW_LCD_D7_PIN					GPIO_PIN_15

//#define BOARD_HW_LCD_LED_PORT				GPIOB
//#define BOARD_HW_LCD_LED_PIN				GPIO_PIN_0

#define BOARD_HW_LCD_RESET_PORT				GPIOF
#define BOARD_HW_LCD_RESET_PIN				GPIO_PIN_0

//
///* LCD power */
#define BOARD_HW_LCD_POWER_CONTROL_PORT		    GPIOB
#define BOARD_HW_LCD_POWER_CONTROL_PIN		    GPIO_PIN_8
//
///* LCD_BRIGHTNESS */
#define BOARD_HW_LCD_BRIGHTNESS_CONTROL_PORT	GPIOA
#define BOARD_HW_LCD_BRIGHTNESS_CONTROL_PIN		GPIO_PIN_3
//
///* Button pin */
//#define BOARD_HW_BTN_STT0_PORT				GPIOF
//#define BOARD_HW_BTN_STT0_PIN				GPIO_PIN_6
//
//#define BOARD_HW_BTN_STT1_PORT				GPIOA
//#define BOARD_HW_BTN_STT1_PIN				GPIO_PIN_10
//
//
///* Buzzer */
//#define BOARD_HW_BUZZER_PORT			GPIOF
//#define BOARD_HW_BUZZER_PIN				GPIO_PIN_7
//#define BUZZER_ON()                     gpio_bit_write(BOARD_HW_BUZZER_PORT, BOARD_HW_BUZZER_PIN, (bit_status)1)
//#define BUZZER_OFF()                    gpio_bit_write(BOARD_HW_BUZZER_PORT, BOARD_HW_BUZZER_PIN, (bit_status)0)
//
///* LED */
//#define BOARD_HW_LED1_R_PORT			GPIOA
//#define BOARD_HW_LED1_R_PIN				GPIO_PIN_6
//#define BOARD_HW_LED1_G_PORT			GPIOB
//#define BOARD_HW_LED1_G_PIN				GPIO_PIN_12
//#define BOARD_HW_LED2_R_PORT			GPIOA
//#define BOARD_HW_LED2_R_PIN				GPIO_PIN_11
//#define BOARD_HW_LED2_G_PORT			GPIOB
//#define BOARD_HW_LED2_G_PIN				GPIO_PIN_13
//
//#define LED1_R_ON()             	gpio_bit_write(BOARD_HW_LED1_R_PORT, BOARD_HW_LED1_R_PIN, (bit_status)BOARD_HW_LED_ON)
//#define LED1_R_OFF()                gpio_bit_write(BOARD_HW_LED1_R_PORT, BOARD_HW_LED1_R_PIN, (bit_status)BOARD_HW_LED_OFF)
//#define LED1_R_TOGGLE()             gpio_bit_toggle(BOARD_HW_LED1_R_PORT, BOARD_HW_LED1_R_PIN)
//
//#define LED1_G_ON()                 gpio_bit_write(BOARD_HW_LED1_G_PORT, BOARD_HW_LED1_G_PIN, (bit_status)BOARD_HW_LED_ON)
//#define LED1_G_OFF()                gpio_bit_write(BOARD_HW_LED1_G_PORT, BOARD_HW_LED1_G_PIN, (bit_status)BOARD_HW_LED_OFF)
//#define LED1_G_TOGGLE()             gpio_bit_toggle(BOARD_HW_LED1_G_PORT, BOARD_HW_LED1_G_PIN)
//
//#define LED2_R_ON()                 gpio_bit_write(BOARD_HW_LED2_R_PORT, BOARD_HW_LED2_R_PIN, (bit_status)BOARD_HW_LED_ON)
//#define LED2_R_OFF()                gpio_bit_write(BOARD_HW_LED2_R_PORT, BOARD_HW_LED2_R_PIN, (bit_status)BOARD_HW_LED_OFF)
//#define LED2_R_TOGGLE()             gpio_bit_toggle(BOARD_HW_LED2_R_PORT, BOARD_HW_LED2_R_PIN)
//
//#define LED2_G_ON()                 gpio_bit_write(BOARD_HW_LED2_G_PORT, BOARD_HW_LED2_G_PIN, (bit_status)BOARD_HW_LED_ON)
//#define LED2_G_OFF()                gpio_bit_write(BOARD_HW_LED2_G_PORT, BOARD_HW_LED2_G_PIN, (bit_status)BOARD_HW_LED_OFF)
//#define LED2_G_TOGGLE()             gpio_bit_toggle(BOARD_HW_LED2_G_PORT, BOARD_HW_LED2_G_PIN)
//

//typedef struct
//{
//	uint32_t year;
//	uint8_t month;
//	uint8_t	day;
//	uint8_t hour;
//	uint8_t min;
//	uint8_t sec;
//	uint32_t unix_timestamp;
//} rtc_time_t;
//
///**
// * @brief		Get current timestamp
// * @retval		Current timestamp
// */
//rtc_time_t *board_hw_rtc_get(void);
//
//
///**
// * @brief               Initialize all board clock, gpio and peripheral
// */
//void board_hw_initialize(void);
//
///**
// * @brief               Reset system
// */
//void board_hw_reset(void);
//
//
///**
// * @brief               Initialize hardware uart debug
// */
//void board_hw_uart_debug_initialize(void);
//
///**
// * @brief               Start RTC
// */
//void board_hw_rtc_start(void);
//
///**
// * @brief               Set current timestamp
// */
//void board_hw_rtc_set_timestamp(uint32_t timestamp);
//
///**
// * @brief               Set lcd brightness
// * @param[in]           LCD brightness in precent (0-100)
// */
//void board_hw_set_lcd_brightness(uint8_t percent);
//
///**
// *  @brief              Generate PWM signal on Vsys pin
// *  @param[in]          percent PWM value, range (0-100)
// */
//void board_hw_pwm_vsys_out(uint8_t percent);
//
///**
// *  @brief              Generate PWM signal on Vin pin
// *  @param[in]          percent PWM value, range (0-100)
// */
//void board_hw_pwm_vin_out(uint8_t percent);
//
///**
// *  @brief              Generate PWM signal on VbatRF pin
// *  @param[in]          percent PWM value, range (0-100)
// */
//void board_hw_pwm_vbatRF_out(uint8_t percent);
//
///**
// *  @brief              Generate PWM signal on V3V3 pin
// *  @param[in]          percent PWM value, range (0-100)
// */
//void board_hw_pwm_3v3_out(uint8_t percent);
//
///**
// *  @brief              Get Vsys
// *  @retval             Vsys voltage in mV
// */
//uint32_t board_hw_get_vsys(void);
//
///**
// *  @brief              Get Vin
// *  @retval             Vin voltage in mV
// */
//uint32_t board_hw_get_vin(void);
//
///**
// *  @brief              Get VBAT RF
// *  @retval             VBAT RF voltage in mV
// */
//uint32_t board_hw_get_vbat_rf(void);
//
///**
// *  @brief              Get 3V3
// *  @retval             3V3 value voltage in mV
// */
//uint32_t board_hw_get_v3v3(void);
//

#endif /*BOARD_HW_H */

