/*
 * app_drv_spi.c
 *
 *  Created on: Aug 1, 2021
 *      Author: huybk
 */
#include "app_drv_spi.h"
//#include "FreeRTOS.h"		don't need free rtos in this
//#include "task.h"
//#include "semphr.h"
#include "spi.h"
#include <stdbool.h>

void app_drv_spi_cs(void *spi, bool level);
//static SemaphoreHandle_t m_sem_spi = NULL;
uint16_t time_out_poll = 0;
static bool spi_done_action;

static inline void on_spi_done(void)
{
	spi_done_action = true;
//    BaseType_t ctx_sw;
//    xSemaphoreGiveFromISR(m_sem_spi, &ctx_sw);
//    portYIELD_FROM_ISR(ctx_sw);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    on_spi_done();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    on_spi_done();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    on_spi_done();
}


void app_drv_spi_initialize(void)
{
//    if (!m_sem_spi)
//    {
//        m_sem_spi = xSemaphoreCreateBinary();
//    }
//
	spi_done_action = false;
}

void app_drv_spi_transmit_frame(void *spi, uint8_t *tx_data, uint32_t length)
{
    if (length && tx_data)
    {
        HAL_SPI_Transmit_DMA(spi, tx_data, length);
        time_out_poll = 500;
        while ((spi_done_action == false) && time_out_poll)
        {
        	spi_done_action = false;
        	time_out_poll--;
        	if (spi_done_action)
        	{
        		spi_done_action = false;
        		break;
        	}
        }
        if (time_out_poll == 0)
        {
        	Error_Handler();
        }
//        xSemaphoreTake(m_sem_spi, portMAX_DELAY);
    }
}

void app_drv_spi_receive_frame(void *spi, uint8_t *rx_data, uint32_t length)
{
    if (length && rx_data)
    {
        HAL_SPI_Receive_DMA(spi, rx_data, length);
        time_out_poll = 500;
        while ((spi_done_action == false) && time_out_poll)
        {
        	spi_done_action = false;
        	time_out_poll--;
        	if (spi_done_action)
        	{
        		spi_done_action = false;
        		break;
        	}
        }
        if (time_out_poll == 0)
        {
        	Error_Handler();
        }
    }
}

void app_drv_spi_transmit_receive_frame(void *spi, uint8_t *tx_data, uint8_t *rx_data, uint32_t length)
{
    if (length && tx_data && rx_data)
    {
        HAL_SPI_TransmitReceive_DMA(spi, tx_data, rx_data, length);
        time_out_poll = 500;
        while ((spi_done_action == false) && time_out_poll)
        {
        	spi_done_action = false;
        	time_out_poll--;
        	if (spi_done_action)
        	{
        		spi_done_action = false;
        		break;
        	}
        }
        if (time_out_poll == 0)
        {
        	Error_Handler();
        }
    }
}

uint8_t app_drv_spi_transmit_byte(void *spi, uint8_t data)
{
    uint8_t tmp[2] = {data, 0xFF};
    HAL_SPI_TransmitReceive(spi, tmp, tmp+1, 1, 10);
    return tmp[1];
}

void app_drv_spi_cs(void *spi, bool level)
{
	if (spi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}
