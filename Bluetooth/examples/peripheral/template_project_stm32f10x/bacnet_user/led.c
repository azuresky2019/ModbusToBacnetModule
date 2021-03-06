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
*********************************************************************/
#include <stdint.h>
#include "hardware.h"
#include "timer.h"
#include "led.h"

#define		TX_LED		21		//PIN_15
#define		RX_LED		22		//PIN_14
#define		LED3		23		//PIN_9
#define		LED4		24		//PIN_8

static struct itimer Off_Delay_Timer_Rx;
static struct itimer Off_Delay_Timer_Tx;
static bool Rx_State;
static bool Tx_State;
static bool LD3_State;


/*************************************************************************
* Description: Activate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_tx_on(
    void)
{
    //GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET);
	nrf_gpio_pin_set(TX_LED);
    timer_interval_no_expire(&Off_Delay_Timer_Tx);
    Tx_State = true;
}

/*************************************************************************
* Description: Activate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_rx_on(
    void)
{
    //GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
    nrf_gpio_pin_set(RX_LED);
	timer_interval_no_expire(&Off_Delay_Timer_Rx);
    Rx_State = true;
}

/*************************************************************************
* Description: Deactivate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_tx_off(
    void)
{
    //GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	nrf_gpio_pin_clear(TX_LED);
    timer_interval_no_expire(&Off_Delay_Timer_Tx);
    Tx_State = false;
}

/*************************************************************************
* Description: Deactivate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_rx_off(
    void)
{
    //GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
	nrf_gpio_pin_clear(RX_LED);
    timer_interval_no_expire(&Off_Delay_Timer_Rx);
    Rx_State = false;
}

/*************************************************************************
* Description: Get the state of the LED
* Returns: true if on, false if off.
* Notes: none
*************************************************************************/
bool led_rx_state(
    void)
{
    return Rx_State;
}

/*************************************************************************
* Description: Get the state of the LED
* Returns: true if on, false if off.
* Notes: none
*************************************************************************/
bool led_tx_state(
    void)
{
    return Tx_State;
}

/*************************************************************************
* Description: Toggle the state of the LED
* Returns: none
* Notes: none
*************************************************************************/
void led_tx_toggle(
    void)
{
    if (led_tx_state()) {
        led_tx_off();
    } else {
        led_tx_on();
    }
}

/*************************************************************************
* Description: Toggle the state of the LED
* Returns: none
* Notes: none
*************************************************************************/
void led_rx_toggle(
    void)
{
    if (led_rx_state()) {
        led_rx_off();
    } else {
        led_rx_on();
    }
}

/*************************************************************************
* Description: Delay before going off to give minimum brightness.
* Returns: none
* Notes: none
*************************************************************************/
void led_rx_off_delay(
    uint32_t delay_ms)
{
    timer_interval_start(&Off_Delay_Timer_Rx, delay_ms);
}

/*************************************************************************
* Description: Delay before going off to give minimum brightness.
* Returns: none
* Notes: none
*************************************************************************/
void led_tx_off_delay(
    uint32_t delay_ms)
{
    timer_interval_start(&Off_Delay_Timer_Tx, delay_ms);
}

/*************************************************************************
* Description: Turn on, and delay before going off.
* Returns: none
* Notes: none
*************************************************************************/
void led_rx_on_interval(
    uint16_t interval_ms)
{
    led_rx_on();
    timer_interval_start(&Off_Delay_Timer_Rx, interval_ms);
}

/*************************************************************************
* Description: Turn on, and delay before going off.
* Returns: none
* Notes: none
*************************************************************************/
void led_tx_on_interval(
    uint16_t interval_ms)
{
    led_tx_on();
    timer_interval_start(&Off_Delay_Timer_Tx, interval_ms);
}

/*************************************************************************
* Description: Task for blinking LED
* Returns: none
* Notes: none
*************************************************************************/
void led_task(
    void)
{
    if (timer_interval_expired(&Off_Delay_Timer_Rx)) {
        timer_interval_no_expire(&Off_Delay_Timer_Rx);
        led_rx_off();
    }
    if (timer_interval_expired(&Off_Delay_Timer_Tx)) {
        timer_interval_no_expire(&Off_Delay_Timer_Tx);
        led_tx_off();
    }
}

/*************************************************************************
* Description: Activate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_ld4_on(
    void)
{
    //GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
	nrf_gpio_pin_set(LED4);
}

/*************************************************************************
* Description: Deactivate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_ld4_off(
    void)
{
    //GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
	nrf_gpio_pin_clear(LED4);
}

/*************************************************************************
* Description: Activate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_ld3_on(
    void)
{
    //GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
	nrf_gpio_pin_set(LED3);
    LD3_State = true;
}

/*************************************************************************
* Description: Deactivate the LED
* Returns: nothing
* Notes: none
**************************************************************************/
void led_ld3_off(
    void)
{
    //GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
	nrf_gpio_pin_clear(LED3);
    LD3_State = false;
}

/*************************************************************************
* Description: Get the state of the LED
* Returns: true if on, false if off.
* Notes: none
*************************************************************************/
bool led_ld3_state(
    void)
{
    return LD3_State;
}

/*************************************************************************
* Description: Toggle the state of the LED
* Returns: none
* Notes: none
*************************************************************************/
void led_ld3_toggle(
    void)
{
    if (led_ld3_state()) {
        led_ld3_off();
    } else {
        led_ld3_on();
    }
}

/*************************************************************************
* Description: Initialize the LED hardware
* Returns: none
* Notes: none
*************************************************************************/
void led_init(
    void)
{
	nrf_gpio_cfg_output(TX_LED);
	nrf_gpio_cfg_output(RX_LED);
	nrf_gpio_cfg_output(LED3);
	nrf_gpio_cfg_output(LED4);
	
    led_tx_on();
    led_rx_on();
    led_ld3_on();
    led_ld4_on();
}

