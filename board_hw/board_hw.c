#include "board_hw.h"
//#include "gd32e23x.h"
//#include "gd32e23x_usart.h"
#include "app_debug.h"
#include "main.h"

#define RTC_BKP_VALUE    0x32F0
__ALIGNED(4) uint16_t m_adc_value[4];

//void pwm_config(void);

void board_hw_initialize(void)
{
//	/* RCC configurations */
//    rcu_periph_clock_enable(RCU_GPIOA);
//    rcu_periph_clock_enable(RCU_GPIOB);
//    rcu_periph_clock_enable(RCU_GPIOC);
//    rcu_periph_clock_enable(RCU_GPIOF);
//
//
//	// Ext wdt
//	gpio_mode_set(BOARD_HW_EXT_WDT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_EXT_WDT_PIN);
//	gpio_output_options_set(BOARD_HW_EXT_WDT_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_EXT_WDT_PIN);
//	gpio_bit_reset(BOARD_HW_EXT_WDT_PORT, BOARD_HW_EXT_WDT_PIN);
//	volatile uint32_t delay = 0xFF;
//	while (delay--);
//	gpio_bit_set(BOARD_HW_EXT_WDT_PORT, BOARD_HW_EXT_WDT_PIN);
//
//	// Button STT
//    gpio_mode_set(BOARD_HW_BTN_STT0_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BOARD_HW_BTN_STT0_PIN);
//	gpio_mode_set(BOARD_HW_BTN_STT1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BOARD_HW_BTN_STT1_PIN);
//
	// LCD
	gpio_mode_set(BOARD_HW_LCD_RS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_RS_PIN);
	gpio_output_options_set(BOARD_HW_LCD_RS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_RS_PIN);
	gpio_bit_reset(BOARD_HW_LCD_RS_PORT, BOARD_HW_LCD_RS_PIN);

	gpio_mode_set(BOARD_HW_LCD_RW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_RW_PIN);
	gpio_output_options_set(BOARD_HW_LCD_RW_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_RW_PIN);
	gpio_bit_reset(BOARD_HW_LCD_RW_PORT, BOARD_HW_LCD_RW_PIN);

	gpio_mode_set(BOARD_HW_LCD_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_EN_PIN);
	gpio_output_options_set(BOARD_HW_LCD_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_EN_PIN);
	gpio_bit_reset(BOARD_HW_LCD_EN_PORT, BOARD_HW_LCD_EN_PIN);

	gpio_mode_set(BOARD_HW_LCD_D0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D0_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D0_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D0_PORT, BOARD_HW_LCD_D0_PIN);

	gpio_mode_set(BOARD_HW_LCD_D1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D1_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D1_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D1_PORT, BOARD_HW_LCD_D1_PIN);

	gpio_mode_set(BOARD_HW_LCD_D2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D2_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D3_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D2_PORT, BOARD_HW_LCD_D3_PIN);

	gpio_mode_set(BOARD_HW_LCD_D3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D3_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D3_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D3_PORT, BOARD_HW_LCD_D3_PIN);

	gpio_mode_set(BOARD_HW_LCD_D4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D4_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D4_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D4_PORT, BOARD_HW_LCD_D4_PIN);

	gpio_mode_set(BOARD_HW_LCD_D5_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D5_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D5_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D5_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D5_PORT, BOARD_HW_LCD_D5_PIN);

	gpio_mode_set(BOARD_HW_LCD_D6_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D6_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D6_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D6_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D6_PORT, BOARD_HW_LCD_D6_PIN);

	gpio_mode_set(BOARD_HW_LCD_D7_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_D7_PIN);
	gpio_output_options_set(BOARD_HW_LCD_D7_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_D7_PIN);
	gpio_bit_reset(BOARD_HW_LCD_D7_PORT, BOARD_HW_LCD_D7_PIN);
//
//    // LCD LED
//    gpio_mode_set(BOARD_HW_LCD_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_LED_PIN);
//    gpio_output_options_set(BOARD_HW_LCD_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_LED_PIN);
//    gpio_bit_reset(BOARD_HW_LCD_LED_PORT, BOARD_HW_LCD_LED_PIN);
//
//    pwm_config();
//
//    // LCD power
//    gpio_mode_set(BOARD_HW_LCD_POWER_CONTROL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_POWER_CONTROL_PIN);
//	gpio_output_options_set(BOARD_HW_LCD_POWER_CONTROL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_POWER_CONTROL_PIN);
//	gpio_bit_reset(BOARD_HW_LCD_RESET_PORT,BOARD_HW_LCD_POWER_CONTROL_PIN);
//
//    // LCD reset
//    gpio_mode_set(BOARD_HW_LCD_RESET_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_RESET_PIN);
//	gpio_output_options_set(BOARD_HW_LCD_RESET_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_RESET_PIN);
//	gpio_bit_reset(BOARD_HW_LCD_RESET_PORT, BOARD_HW_LCD_RESET_PIN);
//
//    // BUZZER
//    gpio_mode_set(BOARD_HW_BUZZER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_BUZZER_PIN);
//	gpio_output_options_set(BOARD_HW_BUZZER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_BUZZER_PIN);
//	gpio_bit_reset(BOARD_HW_BUZZER_PORT, BOARD_HW_BUZZER_PIN);
//
//    gpio_mode_set(BOARD_HW_LCD_BRIGHTNESS_CONTROL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LCD_BRIGHTNESS_CONTROL_PIN);
//	gpio_output_options_set(BOARD_HW_LCD_BRIGHTNESS_CONTROL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LCD_BRIGHTNESS_CONTROL_PIN);
//	gpio_bit_set(BOARD_HW_LCD_BRIGHTNESS_CONTROL_PORT, BOARD_HW_LCD_BRIGHTNESS_CONTROL_PIN);
//
//    // LED
//    gpio_mode_set(BOARD_HW_LED1_R_PIN, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LED1_R_PIN);
//	gpio_output_options_set(BOARD_HW_LED1_R_PIN, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LED1_R_PIN);
//	gpio_bit_set(BOARD_HW_LED1_R_PIN, BOARD_HW_LED1_R_PIN);
//
//    gpio_mode_set(BOARD_HW_LED1_G_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LED1_G_PIN);
//	gpio_output_options_set(BOARD_HW_LED1_G_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LED1_G_PIN);
//	gpio_bit_set(BOARD_HW_LED1_G_PORT, BOARD_HW_LED1_G_PIN);
//
//    gpio_mode_set(BOARD_HW_LED2_R_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LED2_R_PIN);
//	gpio_output_options_set(BOARD_HW_LED2_R_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LED2_R_PIN);
//	gpio_bit_set(BOARD_HW_LED2_R_PORT, BOARD_HW_LED2_R_PIN);
//
//    gpio_mode_set(BOARD_HW_LED2_G_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BOARD_HW_LED2_G_PIN);
//	gpio_output_options_set(BOARD_HW_LED2_G_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_LED2_G_PIN);
//	gpio_bit_set(BOARD_HW_LED2_G_PORT, BOARD_HW_LED2_G_PIN);
//
//
//    // ADC on board
//    /* enable ADC clock */
//    rcu_periph_clock_enable(RCU_ADC);
//    /* enable DMA clock */
//    rcu_periph_clock_enable(RCU_DMA);
//    /* config ADC clock */
//    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
//
//
//    // DMA
//    /* ADC_DMA_channel configuration */
//    dma_parameter_struct dma_data_parameter;
//
//    /* ADC DMA_channel configuration */
//    dma_deinit(DMA_CH0);
//
//    /* initialize DMA single data mode */
//    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA);
//    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
//    dma_data_parameter.memory_addr  = (uint32_t)(&m_adc_value);
//    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
//    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
//    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
//    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
//    dma_data_parameter.number       = sizeof(m_adc_value)/sizeof(m_adc_value[0]);
//    dma_data_parameter.priority     = DMA_PRIORITY_LOW;
//    dma_init(DMA_CH0, &dma_data_parameter);
//
//    dma_circulation_enable(DMA_CH0);
//
//    /* enable DMA channel */
//    dma_channel_enable(DMA_CH0);
//
//    // ADC
//    /* ADC contineous function enable */
//    adc_special_function_config(ADC_CONTINUOUS_MODE, ENABLE);
//    adc_special_function_config(ADC_SCAN_MODE, ENABLE);
//    /* ADC trigger config */
//    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
//    /* ADC data alignment config */
//    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
//    /* ADC channel length config */
//    adc_channel_length_config(ADC_REGULAR_CHANNEL, sizeof(m_adc_value)/sizeof(m_adc_value[0]));
//
//    /* ADC regular channel config */
//    adc_regular_channel_config(0U, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
//    adc_regular_channel_config(1U, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);
//    adc_regular_channel_config(2U, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);
//    adc_regular_channel_config(3U, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);
//    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
//
//    /* enable ADC interface */
//    adc_enable();
//    sys_delay_ms(1U);
//    /* ADC calibration and reset calibration */
//    adc_calibration_enable();
//
//    /* ADC DMA function enable */
//    adc_dma_mode_enable();
//    /* ADC software trigger enable */
//    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
}
//
//void board_hw_reset(void)
//{
//    NVIC_SystemReset();
//}
//
//
///*!
//    \brief      RTC configuration function
//    \param[in]  none
//    \param[out] none
//    \retval     none
//*/
//static volatile uint32_t prescaler_a = 0, prescaler_s = 0;
//rtc_timestamp_struct rtc_timestamp;
//rtc_parameter_struct rtc_initpara;
//static void rtc_pre_config(void)
//{
//#if defined (RTC_CLOCK_SOURCE_IRC40K)
//	  rcu_osci_on(RCU_IRC40K);
//	  rcu_osci_stab_wait(RCU_IRC40K);
//	  rcu_rtc_clock_config(RCU_RTCSRC_IRC40K);
//
//	  prescaler_s = 0x18F;
//	  prescaler_a = 0x63;
//#elif defined (RTC_CLOCK_SOURCE_LXTAL)
//	  rcu_osci_on(RCU_LXTAL);
//	  rcu_osci_stab_wait(RCU_LXTAL);
//	  rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
//	  prescaler_s = 0xFF;
//	  prescaler_a = 0x7F;
//#else
//    #error RTC clock source should be defined.
//#endif /* RTC_CLOCK_SOURCE_IRC40K */
//
//    rcu_periph_clock_enable(RCU_RTC);
//    rtc_register_sync_wait();
//}
//
//void rtc_show_time(void)
//{
//    uint32_t time_subsecond = 0;
//    uint8_t subsecond_ss = 0,subsecond_ts = 0,subsecond_hs = 0;
//
//    rtc_current_time_get(&rtc_initpara);
//    /* get the subsecond value of current time, and convert it into fractional format */
//    time_subsecond = rtc_subsecond_get();
//    subsecond_ss=(1000-(time_subsecond*1000+1000)/400)/100;
//    subsecond_ts=(1000-(time_subsecond*1000+1000)/400)%100/10;
//    subsecond_hs=(1000-(time_subsecond*1000+1000)/400)%10;
//
//    DEBUG_PRINTF("Current time: %0.2x:%0.2x:%0.2x .%d%d%d\r\n", \
//          rtc_initpara.rtc_hour, rtc_initpara.rtc_minute, rtc_initpara.rtc_second,\
//          subsecond_ss, subsecond_ts, subsecond_hs);
//}
//
//
///*!
//    \brief      use hyperterminal to setup RTC time and alarm
//    \param[in]  none
//    \param[out] none
//    \retval     none
//*/
//static void rtc_setup(void)
//{
//    /* setup RTC time value */
//    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;
//
//    rtc_initpara.rtc_factor_asyn = prescaler_a;
//    rtc_initpara.rtc_factor_syn = prescaler_s;
//    rtc_initpara.rtc_year = 0x16;
//    rtc_initpara.rtc_day_of_week = RTC_SATURDAY;
//    rtc_initpara.rtc_month = RTC_APR;
//    rtc_initpara.rtc_date = 0x30;
//    rtc_initpara.rtc_display_format = RTC_24HOUR;
//    rtc_initpara.rtc_am_pm = RTC_AM;
//
//    /* current time input */
//    DEBUG_PRINTF("=======Configure RTC Time========\r\n");
//    tmp_hh = 0;
//	tmp_mm = 0;
//	tmp_ss = 0;
//	tmp_ss = 0;
//
//    /* RTC current time configuration */
//    if(ERROR == rtc_init(&rtc_initpara))
//	{
//        DEBUG_PRINTF("\r\n** RTC time configuration failed! **\r\n");
//    }
//	else
//	{
//        DEBUG_PRINTF("\r\n** RTC time configuration success! **\r\n");
//        rtc_show_time();
//        RTC_BKP0 = RTC_BKP_VALUE;
//    }
//}
//
//
//void board_hw_rtc_start(void)
//{
//	/* Enable access to RTC registers in Backup domain */
//    rcu_periph_clock_enable(RCU_PMU);
//    pmu_backup_write_enable();
//
//    rtc_pre_config();
//
//    /* check if RTC has aready been configured */
//    if (RTC_BKP_VALUE != RTC_BKP0)
//	{
//        rtc_setup();
//    }
//	else
//	{
//        /* detect the reset source */
//        if (RESET != rcu_flag_get(RCU_FLAG_PORRST))
//		{
//            DEBUG_PRINTF("power on reset occurred....\r\n");
//        }
//		else if (RESET != rcu_flag_get(RCU_FLAG_EPRST))
//		{
//            DEBUG_PRINTF("external reset occurred....\r\n");
//        }
//        DEBUG_PRINTF("no need to configure RTC....\r\n");
//        rtc_show_time();
//    }
//
//    rcu_all_reset_flag_clear();
//
////	exti_flag_clear(EXTI_19);
////    exti_init(EXTI_19,EXTI_INTERRUPT,EXTI_TRIG_RISING);
////    nvic_irq_enable(RTC_IRQn,0);
//
//    /* RTC timestamp configuration */
////    rtc_timestamp_enable(RTC_TIMESTAMP_FALLING_EDGE);
////    rtc_interrupt_enable(RTC_INT_TIMESTAMP);
////    rtc_flag_clear(RTC_STAT_TSF|RTC_STAT_TSOVRF);
//
//}
//
//static void rtc_show_timestamp(void)
//{
//    uint32_t ts_subsecond = 0;
//    uint8_t ts_subsecond_ss,ts_subsecond_ts,ts_subsecond_hs;
//
//    rtc_timestamp_get(&rtc_timestamp);
//    /* get the subsecond value of timestamp time, and convert it into fractional format */
//    ts_subsecond = rtc_timestamp_subsecond_get();
//    ts_subsecond_ss=(1000-(ts_subsecond*1000+1000)/400)/100;
//    ts_subsecond_ts=(1000-(ts_subsecond*1000+1000)/400)%100/10;
//    ts_subsecond_hs=(1000-(ts_subsecond*1000+1000)/400)%10;
//
//    DEBUG_PRINTF("Get the time-stamp time: %0.2u:%0.2u:%0.2u .%d%d%d\r\n", \
//          rtc_timestamp.rtc_timestamp_hour, rtc_timestamp.rtc_timestamp_minute, rtc_timestamp.rtc_timestamp_second,\
//          ts_subsecond_ss, ts_subsecond_ts, ts_subsecond_hs);
//
//}
//
//rtc_time_t m_rtc_time;
//static inline uint8_t bcd_2_decimal(uint8_t hex)
//{
//    int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
//    return dec;
//}
//
//// Function to convert Decimal to BCD
//static inline uint8_t decimal_2_bcd(uint8_t num)
//{
//	return (num/10) * 16 + (num % 10);
//}
//
//rtc_time_t *board_hw_rtc_get(void)
//{
//	rtc_current_time_get(&rtc_initpara);
//
//	m_rtc_time.year = bcd_2_decimal(rtc_initpara.rtc_year) + 1970;
//	if (m_rtc_time.year  > 2049)
//	{
//		m_rtc_time.year = 2020;
//	}
//	m_rtc_time.month = bcd_2_decimal(rtc_initpara.rtc_month);
//	m_rtc_time.day = bcd_2_decimal(rtc_initpara.rtc_date);
//	m_rtc_time.hour = bcd_2_decimal(rtc_initpara.rtc_hour);
//	m_rtc_time.min = bcd_2_decimal(rtc_initpara.rtc_minute);
//	m_rtc_time.sec = bcd_2_decimal(rtc_initpara.rtc_second);
//
//	return &m_rtc_time;
//}
//
///*******************************************************************************
//**
//    Function Name  : counter_to_struct
//    * Description    : populates time-struct based on counter-value
//    * input          : - counter-value (unit seconds, 0 -> 1.1.2000 00:00:00),
//    *                  - Pointer to time-struct
//    * Output         : time-struct gets populated, DST not taken into account here
//    * Return         : none
//    * Based on code from Peter Dannegger found in the mikrocontroller.net forum.
//*/
//static const uint8_t days_in_month[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
//static void counter_to_struct(uint32_t sec, rtc_time_t *t, uint8_t cal_year)
//{
//	#define FIRSTYEAR 2000 // start year
//	#define FIRSTDAY 6     // 0 = Sunday
//
//    uint16_t day;
//    uint8_t year;
//    uint16_t day_of_year;
//    uint8_t leap400;
//    uint8_t month;
//
//    t->sec = sec % 60;
//    sec /= 60;
//    t->min = sec % 60;
//    sec /= 60;
//    t->hour = sec % 24;
//
//    if (cal_year == 0)
//        return;
//
//    day = (uint16_t)(sec / 24);
//
//    year = FIRSTYEAR % 100;                    // 0..99
//    leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3); // 4, 3, 2, 1
//
//    for (;;)
//    {
//        day_of_year = 365;
//        if ((year & 3) == 0)
//        {
//            day_of_year = 366; // leap year
//            if (year == 0 || year == 100 || year == 200)
//            { // 100 year exception
//                if (--leap400)
//                { // 400 year exception
//                    day_of_year = 365;
//                }
//            }
//        }
//        if (day < day_of_year)
//        {
//            break;
//        }
//        day -= day_of_year;
//        year++; // 00..136 / 99..235
//    }
//    t->year = year + FIRSTYEAR / 100 * 100 - 2000; // + century
//
//    if (day_of_year & 1 && day > 58)
//    {          // no leap year and after 28.2.
//        day++; // skip 29.2.
//    }
//
//    for (month = 1; day >= days_in_month[month - 1]; month++)
//    {
//        day -= days_in_month[month - 1];
//    }
//
//    t->month = month; // 1..12
//    t->day = day + 1; // 1..31
//}

//
//
//void board_hw_rtc_set_timestamp(uint32_t timestamp)
//{
////	DEBUG_PRINTF("Update timestamp %u\r\n", timestamp);
////	timestamp += 25200; // gmt adjust +7
//	counter_to_struct(timestamp, &m_rtc_time, 1);
////	/* setup RTC time value */
//
//    rtc_initpara.rtc_factor_asyn = prescaler_a;
//    rtc_initpara.rtc_factor_syn = prescaler_s;
//    rtc_initpara.rtc_year = decimal_2_bcd(m_rtc_time.year);
//    rtc_initpara.rtc_day_of_week = RTC_SATURDAY;
//    rtc_initpara.rtc_month = decimal_2_bcd(m_rtc_time.month);
//    rtc_initpara.rtc_date = decimal_2_bcd(m_rtc_time.day);
//    rtc_initpara.rtc_display_format = RTC_24HOUR;
//	rtc_initpara.rtc_hour = decimal_2_bcd(m_rtc_time.hour);;
//	rtc_initpara.rtc_minute = decimal_2_bcd(m_rtc_time.min);;
//	rtc_initpara.rtc_second = decimal_2_bcd(m_rtc_time.sec);
//    rtc_initpara.rtc_am_pm = RTC_AM;
//
//    /* RTC current time configuration */
//    if(ERROR == rtc_init(&rtc_initpara))
//	{
//        DEBUG_PRINTF("\r\n** RTC time configuration failed! **\r\n");
//    }
//	else
//	{
////        DEBUG_PRINTF("\r\n** RTC time configuration success! **\r\n");
////        rtc_show_time();
//        RTC_BKP0 = RTC_BKP_VALUE;
//    }
//}
//
//
/////*!
////    \brief      this function handles RTC interrupt request
////    \param[in]  none
////    \param[out] none
////    \retval     none
////*/
////void RTC_IRQHandler(void)
////{
////    if(RESET != rtc_flag_get(RTC_STAT_TSF))
////	{
////        exti_flag_clear(EXTI_19);
////        rtc_show_timestamp();
////        rtc_flag_clear(RTC_STAT_TSF|RTC_STAT_TSOVRF);
////    }
////}
//
////void board_hw_set_lcd_brightness(uint8_t percent)
////{
////    if (percent > 100)
////    {
////        percent = 100;
////    }
////
////    uint32_t pwm = (uint32_t)percent * 999 / 100;
////    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, pwm);
////}
//
//void pwm_config(void)
//{
//    /* Configure PA8(TIMER0 CH0) as alternate function */
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
//    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_8);
//
//    /* Configure PA9(TIMER0 CH1) as alternate function */
//    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
//    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
//    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_9);
//
//    /* -----------------------------------------------------------------------
//    TIMER0 configuration: generate 3 PWM signals with 3 different duty cycles:
//    TIMER0CLK = SystemCoreClock / 72 = 1MHz, the PWM frequency is 62.5Hz.
//
//    TIMER0 channel0 duty cycle = (4000/ 16000)* 100  = 25%
//    TIMER0 channel1 duty cycle = (8000/ 16000)* 100  = 50%
//    ----------------------------------------------------------------------- */
//    timer_oc_parameter_struct timer_ocinitpara;
//    timer_parameter_struct timer_initpara;
//
//    /* enable the TIMER clock */
//    rcu_periph_clock_enable(RCU_TIMER0);
//    /* deinit a TIMER */
//    timer_deinit(TIMER0);
//    /* initialize TIMER init parameter struct */
//    timer_struct_para_init(&timer_initpara);
//    /* TIMER0 configuration */
//    timer_initpara.prescaler         = 71;
//    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
//    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
//    timer_initpara.period            = 15999;
//    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
//    timer_initpara.repetitioncounter = 0;
//    timer_init(TIMER0, &timer_initpara);
//
//    /* initialize TIMER channel output parameter struct */
//    timer_channel_output_struct_para_init(&timer_ocinitpara);
//    /* configure TIMER channel output function */
//    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
//    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
//    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
//    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
//    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
//    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
//    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocinitpara);
//    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocinitpara);
//
//    /* CH0 configuration in PWM mode0, duty cycle 25% */
//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);
//    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
//    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
//
//    /* CH1 configuration in PWM mode0, duty cycle 50% */
//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);
//    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
//
//    /* auto-reload preload enable */
//    timer_auto_reload_shadow_enable(TIMER0);
//    /* auto-reload preload enable */
//    timer_enable(TIMER0);
//
//
//    /* Timer 14 */
//    /* Configure PB14(TIMER14 CH0) as alternate function */
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
//    gpio_af_set(GPIOB, GPIO_AF_3, GPIO_PIN_14);
//
//    /* Configure PA9(TIMER14 CH1) as alternate function */
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_15);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
//    gpio_af_set(GPIOB, GPIO_AF_3, GPIO_PIN_15);
//
//    /* -----------------------------------------------------------------------
//    TIMER14 configuration: generate 3 PWM signals with 3 different duty cycles:
//    TIMER14CLK = SystemCoreClock / 72 = 1MHz, the PWM frequency is 62.5Hz.
//
//    TIMER14 channel0 duty cycle = (4000/ 16000)* 100  = 25%
//    TIMER14 channel1 duty cycle = (8000/ 16000)* 100  = 50%
//    ----------------------------------------------------------------------- */
//    /* enable the TIMER clock */
//    rcu_periph_clock_enable(RCU_TIMER14);
//    /* deinit a TIMER */
//    timer_deinit(TIMER14);
//    /* initialize TIMER init parameter struct */
//    timer_struct_para_init(&timer_initpara);
//    /* TIMER14 configuration */
//    timer_initpara.prescaler         = 71;
//    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
//    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
//    timer_initpara.period            = 15999;
//    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
//    timer_initpara.repetitioncounter = 0;
//    timer_init(TIMER14, &timer_initpara);
//
//    /* initialize TIMER channel output parameter struct */
//    timer_channel_output_struct_para_init(&timer_ocinitpara);
//    /* configure TIMER channel output function */
//    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
//    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
//    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
//    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
//    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
//    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
//    timer_channel_output_config(TIMER14, TIMER_CH_0, &timer_ocinitpara);
//    timer_channel_output_config(TIMER14, TIMER_CH_1, &timer_ocinitpara);
//
//    /* CH0 configuration in PWM mode0, duty cycle 0% */
//    timer_channel_output_pulse_value_config(TIMER14, TIMER_CH_0, 0);
//    timer_channel_output_mode_config(TIMER14, TIMER_CH_0, TIMER_OC_MODE_PWM0);
//    timer_channel_output_shadow_config(TIMER14, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
//
//    /* CH1 configuration in PWM mode0, duty cycle 0% */
//    timer_channel_output_pulse_value_config(TIMER14, TIMER_CH_1, 0);
//    timer_channel_output_mode_config(TIMER14, TIMER_CH_1, TIMER_OC_MODE_PWM0);
//
//    /* auto-reload preload enable */
//    timer_auto_reload_shadow_enable(TIMER14);
//    /* auto-reload preload enable */
//    timer_enable(TIMER14);
//}
//
//void board_hw_pwm_vsys_out(uint8_t percent)
//{
//    if (percent > 100)
//    {
//        percent = 100;
//    }
//
//    uint32_t pwm = (uint32_t)percent * 16000 / 100;
//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pwm);
//}
//
//void board_hw_pwm_vin_out(uint8_t percent)
//{
//    if (percent > 100)
//    {
//        percent = 100;
//    }
//
//    uint32_t pwm = (uint32_t)percent * 16000 / 100;
//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pwm);
//}
//
//void board_hw_pwm_vbatRF_out(uint8_t percent)
//{
//    if (percent > 100)
//    {
//        percent = 100;
//    }
//
//    uint32_t pwm = (uint32_t)percent * 16000 / 100;
//    timer_channel_output_pulse_value_config(TIMER14, TIMER_CH_0, pwm);
//}
//
//void board_hw_pwm_3v3_out(uint8_t percent)
//{
//    if (percent > 100)
//    {
//        percent = 100;
//    }
//    
//    uint32_t pwm = (uint32_t)percent * 16000 / 100;
//    timer_channel_output_pulse_value_config(TIMER14, TIMER_CH_1, pwm);
//}
//
//uint32_t board_hw_get_vsys()
//{
//    uint32_t voltage = m_adc_value[2] * 70;        //  2 = resistor div
//    voltage *= 330;
//    voltage /= 4096;
//    return voltage;
//}
//
//
//uint32_t board_hw_get_vin()
//{
//    uint32_t voltage = m_adc_value[3] * 2;        //  7.8 = resistor div =>> *3300 = 79 *330
//    voltage *= 3300;
//    voltage /= 4096;
//    return voltage;
//}
//
//
//uint32_t board_hw_get_vbat_rf()
//{
//    uint32_t voltage = m_adc_value[0] * 2;        //  2 = resistor div
//    voltage *= 3300;
//    voltage /= 4096;
//    return voltage;
//}
//
//
//uint32_t board_hw_get_v3v3()
//{
//    uint32_t voltage = m_adc_value[1];
//    voltage *= 3300;
//    voltage /= 4096;
//    return voltage;
//}

