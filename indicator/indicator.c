/*
 * indicator.c
 *
 *  Created on: Jun 18, 2021
 *      Author: huybk213
 */

/******************************************************************************
 * @file    	LedStatus.c
 * @author
 * @version 	V1.0.0
 * @date    	15/01/2014
 * @brief
 ******************************************************************************/

/******************************************************************************
                                   INCLUDES
 ******************************************************************************/
#include "indicator.h"
#include "main.h"

#define ON 1
#define OFF 0
#define BEEP_COUNT_LENGTH 1

/******************************************************************************
                                   PRIVATE VARIABLES
 ******************************************************************************/
static volatile uint8_t m_beep_count = 0;
static volatile uint8_t m_tmp_beep_count = 0;
static volatile uint16_t m_beep_length = 0;

/******************************************************************************
                                   LOCAL FUNCTIONS
 ******************************************************************************/
static void buzzer_tick(void);
static void control_user_buzzer(uint8_t MODE);

void indicator_tick(void)
{
	static volatile uint32_t buzzer_tick_timming = 0;
	if (m_beep_count == 0
		&& m_beep_length == 0)
	{
		control_user_buzzer(OFF);
		return;
	}
	if (buzzer_tick_timming++ == 100)
	{
		buzzer_tick_timming = 0;

		buzzer_tick();

		if (m_beep_count == 0
			&& m_beep_length == 0)
		{
			control_user_buzzer(OFF);
		}
	}
}

static void buzzer_tick(void)
{
    if (m_beep_count > 0)
    {
        if (m_tmp_beep_count++ >= BEEP_COUNT_LENGTH)
        {
            m_tmp_beep_count = 0;
            control_user_buzzer(m_beep_count % 2 ? OFF : ON);

            m_beep_count--;
            if (m_beep_count == 0)
            {
                control_user_buzzer(OFF);
            }
        }
    }
    else
    {
        if (m_beep_length > 0)
        {
            m_beep_length--;
            if (m_beep_length == 0)
            {
                control_user_buzzer(OFF);
            }
        }
    }
}

void indicator_buzzer_beep(uint16_t length)
{
    control_user_buzzer(OFF);
    control_user_buzzer(ON);
    m_beep_length = length;
}

void indicator_buzzer_beeps(uint8_t count)
{
    if (m_beep_count == 0)
    {
        control_user_buzzer(ON);
    }
    m_beep_count = count + (count - 1);
}

static void control_user_buzzer(uint8_t MODE)
{
    if (MODE == ON)
    {
    	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 1);
    }
    else
    {
    	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 0);
    }
}

bool indicator_is_beeping(void)
{
    return m_beep_count ? true : false;
}
