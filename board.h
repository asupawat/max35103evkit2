/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

#include "max3510x.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

uint8_t board_read_bcd_switches(void);
void board_wait( uint32_t ms );
void board_init( void );

extern const gpio_cfg_t g_board_led;
extern const gpio_cfg_t g_board_relay;
extern const gpio_cfg_t g_board_max3510x_int;


void max3510x_int_isr(void * pv);
extern max3510x_t g_max35103;

void board_printf( const char *p_format, ... );
uint16_t board_uart_write( void *pv, uint16_t length );
uint16_t board_uart_read( void *pv, uint16_t length );
void board_tdc_interrupt_enable(bool b);
bool board_flash_write( const void *p_data, uint16_t size );
void board_flash_read( void *p_data, uint16_t size );

#define BOARD_J3_UART				0				// defines the uart associated with the 6-pin header 
#define BOARD_MBED_UART			3				// uart connected to the mbed pico module

#ifdef __cplusplus
};
#endif

