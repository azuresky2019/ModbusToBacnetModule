/**************************************************************************
*
* Copyright (C) 2011 Steve Karg <skarg@users.sourceforge.net>
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Module Description:
* Handle the configuration and operation of the RS485 bus.
**************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "hardware.h"
#include "timer.h"
#include "bits.h"
#include "fifo.h"
#include "led.h"
#include "rs485.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "app_error.h"
#include "nrf_drv_uart.h"
#include "nrf_delay.h"

/* buffer for storing received bytes - size must be power of two */
static uint8_t Receive_Buffer_Data[512];
static FIFO_BUFFER Receive_Buffer;
/* amount of silence on the wire */
static struct etimer Silence_Timer;
/* baud rate */
static uint32_t Baud_Rate = 115200;

/* The minimum time after the end of the stop bit of the final octet of a */
/* received frame before a node may enable its EIA-485 driver: 40 bit times. */
/* At 9600 baud, 40 bit times would be about 4.166 milliseconds */
/* At 19200 baud, 40 bit times would be about 2.083 milliseconds */
/* At 38400 baud, 40 bit times would be about 1.041 milliseconds */
/* At 57600 baud, 40 bit times would be about 0.694 milliseconds */
/* At 76800 baud, 40 bit times would be about 0.520 milliseconds */
/* At 115200 baud, 40 bit times would be about 0.347 milliseconds */
/* 40 bits is 4 octets including a start and stop bit with each octet */
#define 	Tturnaround  		(40UL)
#define		RS485_EN	 		(2UL)
#define 	BOARD_UART0_RX		(11UL)
#define 	BOARD_UART0_TX		(9UL)
#define 	RTS_PIN_NUMBER		(15)
#define 	CTS_PIN_NUMBER		(16)
#define 	UART_TX_BUF_SIZE 	128  /**< UART TX buffer size. */
#define 	UART_RX_BUF_SIZE 	128  /*2,power of two*/

bool volatile rs485_TC_flag = false;
uint8_t volatile TC_Count = 0;
extern nrf_drv_uart_t app_uart_inst;
void send_uart_byte(uint8_t ch);
/*************************************************************************
* Description: Reset the silence on the wire timer.
* Returns: nothing
* Notes: none
**************************************************************************/
void rs485_silence_reset(
    void)
{
    timer_elapsed_start(&Silence_Timer);
}

/*************************************************************************
* Description: Determine the amount of silence on the wire from the timer.
* Returns: true if the amount of time has elapsed
* Notes: none
**************************************************************************/
bool rs485_silence_elapsed(
    uint32_t interval)
{
    return timer_elapsed_milliseconds(&Silence_Timer, interval);
}

/*************************************************************************
* Description: Baud rate determines turnaround time.
* Returns: amount of milliseconds
* Notes: none
**************************************************************************/
static uint16_t rs485_turnaround_time(
    void)
{
    /* delay after reception before transmitting - per MS/TP spec */
    /* wait a minimum  40 bit times since reception */
    /* at least 2 ms for errors: rounding, clock tick */
    if (Baud_Rate) {
        return (2 + ((Tturnaround * 1000UL) / Baud_Rate));
    } else {
        return 2;
    }
}

/*************************************************************************
* Description: Use the silence timer to determine turnaround time.
* Returns: true if turnaround time has expired.
* Notes: none
**************************************************************************/
bool rs485_turnaround_elapsed(
    void)
{
    return timer_elapsed_milliseconds(&Silence_Timer, rs485_turnaround_time());
}


/*************************************************************************
* Description: Determines if an error occured while receiving
* Returns: true an error occurred.
* Notes: none
**************************************************************************/
bool rs485_receive_error(
    void)
{
    return false;
}
#if 1
/*********************************************************************//**
 * @brief        USARTx interrupt handler sub-routine
 * @param[in]    None
 * @return       None
 **********************************************************************/
void UART0_IRQHandler(							/* APP_UART_DATA_READY */
    void)
{
    uint8_t data_byte;
	
    if (NRF_UART0->EVENTS_RXDRDY){
		/* Read one byte from the receive data register */
		NRF_UART0->EVENTS_RXDRDY = 0;
		data_byte = NRF_UART0->RXD;		
        (void) FIFO_Put(&Receive_Buffer, data_byte);
	}			
}
#endif

/*************************************************************************
* DESCRIPTION: Return true if a byte is available
* RETURN:      true if a byte is available, with the byte in the parameter
* NOTES:       none
**************************************************************************/
bool rs485_byte_available(
    uint8_t * data_register)
{
    bool data_available = false;        /* return value */

    if (!FIFO_Empty(&Receive_Buffer)) {
        if (data_register) {
            *data_register = FIFO_Get(&Receive_Buffer);
        }
        timer_elapsed_start(&Silence_Timer);
        data_available = true;
        led_rx_on_interval(10);
    }

    return data_available;
}
/* ----------------------- Start implementation -----------------------------*/

/*************************************************************************
* DESCRIPTION: Sends a byte of data
* RETURN:      nothing
* NOTES:       none
**************************************************************************/
void rs485_byte_send(
    uint8_t tx_byte)
{   
	led_tx_on_interval(10);	    
	send_uart_byte(tx_byte);
    timer_elapsed_start(&Silence_Timer);	
}

/*************************************************************************
* Description: Determines if a byte in the USART has been shifted from
*   register
* Returns: true if the USART register is empty
* Notes: none
**************************************************************************/
bool rs485_byte_sent(
    void)
{
    //return nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_TXDRDY);		/* NRF_DRV_UART_EVT_TX_DONE */
	return (bool)NRF_UART0->EVENTS_TXDRDY;
	
}

/*************************************************************************
* Description: Determines if the entire frame is sent from USART FIFO
* Returns: true if the USART FIFO is empty
* Notes: none
**************************************************************************/
bool rs485_frame_sent(
    void)
{
    return rs485_TC_flag;						/* APP_UART_TX_EMPTY */
	//return (bool)NRF_UART0->TASKS_STOPTX;

}

/*************************************************************************
* DESCRIPTION: Send some data and wait until it is sent
* RETURN:      true if a collision or timeout occurred
* NOTES:       none
**************************************************************************/
void rs485_bytes_send(
    uint8_t * buffer,   /* data to send */
    uint16_t nbytes)
{       /* number of bytes of data */
    uint8_t tx_byte;

    while (nbytes) {
        /* Send the data byte */
        tx_byte = *buffer;
        /* Send one byte */
        //USART_SendData(USART2, tx_byte);
		send_uart_byte(tx_byte);	
        while (!rs485_byte_sent()) {
            /* do nothing - wait until Tx buffer is empty */
        }
        buffer++;
        nbytes--;
    }
    /* was the frame sent? */
    while (!rs485_frame_sent()) {
        /* do nothing - wait until the entire frame in the
           Transmit Shift Register has been shifted out */
    }
    timer_elapsed_start(&Silence_Timer);

    return;
}

/*************************************************************************
* Description: Sets the baud rate to non-volatile storeage and configures USART
* Returns: true if a value baud rate was saved
* Notes: none
**************************************************************************/
bool rs485_baud_rate_set(
    uint32_t baud)
{
    bool valid = true;
    switch (baud) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 76800:
        case 115200:
			//nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_115200);
			NRF_UART0->BAUDRATE = 0x01D7E000;
            break;
        default:
            valid = false;
            break;
    }
    return valid;
}

/*************************************************************************
* Description: Determines the baud rate in bps
* Returns: baud rate in bps
* Notes: none
**************************************************************************/
uint32_t rs485_baud_rate(
    void)
{
    return Baud_Rate;
}

/*************************************************************************
* Description: Enable the Request To Send (RTS) aka Transmit Enable pin
* Returns: nothing
* Notes: none
**************************************************************************/
void rs485_rts_enable(
    bool enable)
{
    if (enable) {       
		nrf_gpio_pin_set(RS485_EN);
    } else {      
		nrf_gpio_pin_clear(RS485_EN);
    }
}
/***************************************************************************/
//// Must be called once initially

void uart_init(void){

    //设置uart pin 输入输出方向
    nrf_gpio_cfg_input(BOARD_UART0_RX, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(BOARD_UART0_TX);

    //设置UART TX,RX 流控脚无效
    NRF_UART0->PSELRXD = BOARD_UART0_RX;      
    NRF_UART0->PSELTXD = BOARD_UART0_TX;
    NRF_UART0->PSELRTS = 0XFFFFFFFF;
    NRF_UART0->PSELCTS = 0XFFFFFFFF;  
    NRF_UART0->BAUDRATE = 0x01D7E000;  /* 115200 波特率 */
    NRF_UART0->CONFIG = 0;             /* 不使用流控，不使用校验 */   

    //事件清零
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;  
	NRF_UART0->INTENSET = 1 << 2;	// Enable interrupt on RXDRDY event.
    NRF_UART0->ENABLE = 4;          //开启uart
    NRF_UART0->TASKS_STARTRX = 1;   //使能接收
    NRF_UART0->TASKS_STARTTX = 1;   //使能发送   
	
    // 使能UART中断
    NVIC_SetPriority(UART0_IRQn, 1);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn); 	
}
//// Send a single byte
void send_uart_byte(uint8_t data){

    uint8_t temp = data;

    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->TXD = temp;
    //轮询等待直到数据发送完毕
    while(NRF_UART0->EVENTS_TXDRDY == 0){
       ;
    }
	TC_Count = 5;
	rs485_TC_flag = 0;
}
/*************************************************************************
* Description: Initialize the room network USART
* Returns: nothing
* Notes: none
**************************************************************************/
//uint32_t err_code = NRF_SUCCESS;
void rs485_init(
    void)
{
	uart_init();
	
    FIFO_Init(&Receive_Buffer, &Receive_Buffer_Data[0],
        (unsigned) sizeof(Receive_Buffer_Data));
    timer_elapsed_start(&Silence_Timer);
	
	nrf_gpio_cfg_output(RS485_EN);	
	
}
