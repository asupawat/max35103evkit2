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


// Smart Valve

#include "global.h"

#include "spim.h"
#include "uart.h"

#include "pwrseq_regs.h"
#include "pwrman_regs.h"
#include "flc_regs.h"
#include "rtc_regs.h"
#include "trim_regs.h"
#include "board.h"
#include "transducer.h"

#include <tmr.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#define TIMESTAMP_TIMER	MXC_TMR1

	
// The MAX35103 is connected to SPIM0A:

//  P0.4 = SPMI0A_SCK
//  P0.5 = SPIM0A_MOSI
//  P0.6 = SPIM0A_MISO
//  P0.7 = SPIM0A_SS0

#define BOARD_SIGNATURE			0x8C01

// map interrupt and control signals to pins


#define BOARD_RELAY_PORT	PORT_3
#define BOARD_RELAY_PIN		PIN_3
#define MAX3510X_INT_PORT	PORT_0
#define MAX3510X_INT_PIN	PIN_3

// maps 10-position switch lines (S1 & S2) to pins
#define S00	PIN_0
#define S03 PIN_1
#define S01 PIN_2
#define S02 PIN_3
#define S13 PIN_4
#define S11 PIN_5
#define S12 PIN_6
#define S10 PIN_7

const gpio_cfg_t g_board_led = { PORT_5, PIN_0, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };
const gpio_cfg_t g_board_relay = { BOARD_RELAY_PORT, BOARD_RELAY_PIN, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };
const gpio_cfg_t g_board_max3510x_int = { MAX3510X_INT_PORT, MAX3510X_INT_PIN, GPIO_FUNC_GPIO, GPIO_PAD_INPUT_PULLUP };

static const gpio_cfg_t g_board_max3510x_reset = { PORT_1, PIN_1, GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

static const ioman_cfg_t s_j3_uart_cfg = IOMAN_UART(BOARD_J3_UART, IOMAN_MAP_A, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0);
static const ioman_cfg_t s_mbed_uart_cfg = IOMAN_UART(BOARD_MBED_UART, IOMAN_MAP_B, IOMAN_MAP_UNUSED, IOMAN_MAP_UNUSED, 1, 0, 0);

static const gpio_cfg_t bcd_switches = { PORT_4, PIN_0|PIN_1|PIN_2|PIN_3|PIN_4|PIN_5|PIN_6|PIN_7, GPIO_FUNC_GPIO, GPIO_PAD_INPUT_PULLUP };

static const ioman_cfg_t spi_cfg = IOMAN_SPIM0(1, 1, 0, 0, 0, 0, 0, 0);
static const spim_cfg_t max3510x_spim_cfg = { 1, SPIM_SSEL0_LOW, 12000000/2 }; // SCLK = 12MHz, baudrate is half that
static const gpio_cfg_t max3510x_spi = { PORT_0, (PIN_4 | PIN_5 | PIN_6 | PIN_7), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

static const gpio_cfg_t s_j3_uart = { PORT_0, (PIN_0 | PIN_1), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };
static const gpio_cfg_t s_mbed_uart = { PORT_5, (PIN_3 | PIN_4), GPIO_FUNC_GPIO, GPIO_PAD_NORMAL };

extern void (* const __isr_vector[])(void);

static mxc_uart_regs_t * s_p_uart;

max3510x_t g_max35103;

void GPIO_P0_IRQHandler(void)
{
	GPIO_Handler(0);
}

void UART0_IRQHandler(void)
{
    UART_Handler(MXC_UART_GET_UART(BOARD_J3_UART));
}

void UART3_IRQHandler(void)
{
    UART_Handler(MXC_UART_GET_UART(BOARD_MBED_UART));
}

static void init_isr(void * pv_status )
{
	uint16_t *p_status = (uint16_t*)pv_status;
	*p_status |= max3510x_interrupt_status( &g_max35103 );
}

static void wait_for_interrupt( volatile uint16_t *p_isr_status, uint16_t any )
{
	while( !(*p_isr_status & any) );
	GPIO_IntDisable(&g_board_max3510x_int);
	*p_isr_status &= ~any;
	GPIO_IntEnable(&g_board_max3510x_int);
}

static void init_uart( uint32_t uart_ndx, const ioman_cfg_t *p_ioman_cfg, uint32_t baud )
{
	IRQn_Type irq = MXC_UART_GET_IRQ(uart_ndx);
	NVIC_DisableIRQ(irq);
	uart_cfg_t cfg;
	cfg.parity = UART_PARITY_DISABLE;
	cfg.size = UART_DATA_SIZE_8_BITS;
	cfg.extra_stop = 0;
	cfg.cts = 0;
	cfg.rts = 0;
	cfg.baud = baud;

	sys_cfg_uart_t sys_cfg;
	sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;
	sys_cfg.io_cfg = *p_ioman_cfg;
	

	while( UART_Init(MXC_UART_GET_UART(uart_ndx), &cfg, &sys_cfg) != E_NO_ERROR );

	NVIC_ClearPendingIRQ(irq);
	NVIC_EnableIRQ(irq);
}

void board_init(void)
{
	const uint8_t uart_ndx = BOARD_MBED_UART;
	
	const uint16_t board_rev_addr = (MAX3510X_FLASH_BLOCK_SIZE_WORDS*MAX3510X_FLASH_BLOCK_COUNT)-2;
	volatile uint16_t isr_status;

	SYS_IOMAN_UseVDDIOH( &g_board_relay );
	SYS_IOMAN_UseVDDIOH( &g_board_led );
	SYS_IOMAN_UseVDDIOH( &max3510x_spi );
	SYS_IOMAN_UseVDDIOH( &s_j3_uart );
	SYS_IOMAN_UseVDDIOH( &g_board_max3510x_int );
	SYS_IOMAN_UseVDDIOH( &g_board_max3510x_reset );
 	SYS_IOMAN_UseVDDIOH( &bcd_switches );
	
	GPIO_OutPut( &g_board_relay, ~0 );
	GPIO_OutPut( &g_board_led, ~0 );

	GPIO_Config(&g_board_max3510x_reset);
	GPIO_Config(&g_board_relay);
	GPIO_Config(&g_board_max3510x_int);
	GPIO_Config(&max3510x_spi);
	GPIO_Config(&s_j3_uart);
	GPIO_Config(&s_mbed_uart);
	GPIO_Config(&g_board_led);
	GPIO_Config(&bcd_switches);
	
	s_p_uart = MXC_UART_GET_UART(uart_ndx);
	
	{
		// use this timer as a 96MHz 32-bit timestamp
		tmr32_cfg_t cont_cfg;
 		cont_cfg.mode = TMR32_MODE_CONTINUOUS;
		cont_cfg.polarity = TMR_POLARITY_INIT_LOW;  //start GPIO low
		cont_cfg.compareCount = ~0;
		while( TMR_Init(TIMESTAMP_TIMER, TMR_PRESCALE_DIV_2_0, NULL) != E_NO_ERROR )
			GPIO_OutPut( &g_board_led, 0 );
		TMR32_Config(TIMESTAMP_TIMER, &cont_cfg);
	}

	TMR32_Start(TIMESTAMP_TIMER);
	board_wait(10);									// wait for power to settle
	
	{
		// initialize the SPI port connected to the MAX35103
		sys_cfg_t sys_cfg;
		sys_cfg.clk_scale = CLKMAN_SCALE_AUTO;
		sys_cfg.io_cfg = spi_cfg;
		if( SPIM_Init(MXC_SPIM0, &max3510x_spim_cfg, &sys_cfg) != E_NO_ERROR )
		{
			GPIO_OutPut( &g_board_led, 0 );
			while( 1 );	// initialization failed -- step into CSL to determine the reason
		}
	}

	
	GPIO_IntDisable(&g_board_max3510x_int);
	GPIO_IntConfig(&g_board_max3510x_int, GPIO_INT_LOW_LEVEL);
	GPIO_RegisterCallback(&g_board_max3510x_int, init_isr, (void*)&isr_status );

	transducer_init();
	
	GPIO_OutPut(&g_board_max3510x_reset,~0);	// take the MAX35103 out of reset
	board_wait(1);			// give the MAX35103 some time come out of reset
	max3510x_init(&g_max35103, transducer_config() );	// configure the MAX3510x according per transducer requirements
	board_wait(1);			// give the MAX35103 some time to init

	GPIO_IntClr(&g_board_max3510x_int);
	NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(g_board_max3510x_int.port));
	GPIO_IntEnable(&g_board_max3510x_int);

	max3510x_ldo(&g_max35103,max3510x_ldo_mode_on);
	wait_for_interrupt(&isr_status, MAX3510X_REG_INTERRUPT_STATUS_LDO);
	uint16_t sig = max3510x_read_flash( &g_max35103, board_rev_addr );
	if( sig == BOARD_SIGNATURE )
	{
		// get the trim value from the max35103 flash
		uint16_t trim = max3510x_read_flash( &g_max35103, board_rev_addr-2 );
		MXC_PWRSEQ->reg6 = (MXC_PWRSEQ->reg6 & ~MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF) |
						   ((trim << MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF_POS) & MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF);

		SystemCoreClockUpdate();
	}
	
	max3510x_ldo(&g_max35103,max3510x_ldo_mode_off);

	GPIO_RegisterCallback(&g_board_max3510x_int, max3510x_int_isr, NULL );

	init_uart(BOARD_MBED_UART, &s_mbed_uart_cfg, 9600 );
	init_uart(BOARD_J3_UART, &s_j3_uart_cfg, 115200 );
}

// SystemInit() is modified from what exists in the CSL to accomodate the fact that the 32KHz clock
// isn't available on board reset

void SystemInit(void)
{
    /* Configure the interrupt controller to use the application vector table in */
    /* the application space */
#if defined ( __CC_ARM) || defined ( __GNUC__) 
    /* IAR sets the VTOR pointer prior to SystemInit and setting it here causes stack corruption on IAR startup. */
	  __disable_irq();
    SCB->VTOR = (uint32_t)__isr_vector;
	__DSB();
	__enable_irq();	
#endif /* __CC_ARM || __GNUC__ */
    /* Copy trim information from shadow registers into power manager registers */
    /* NOTE: Checks have been added to prevent bad/missing trim values from being loaded */
    if ((MXC_FLC->ctrl & MXC_F_FLC_CTRL_INFO_BLOCK_VALID) &&
            (MXC_TRIM->for_pwr_reg5 != 0xffffffff) &&
            (MXC_TRIM->for_pwr_reg6 != 0xffffffff)) {
        MXC_PWRSEQ->reg5 = MXC_TRIM->for_pwr_reg5;
        MXC_PWRSEQ->reg6 = MXC_TRIM->for_pwr_reg6;
    } else {
        /* No valid info block, use some reasonable defaults */
        MXC_PWRSEQ->reg6 &= ~MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF;
        MXC_PWRSEQ->reg6 |= (0x1e0 << MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF_POS);
    }

    /* Improve flash access timing */
    MXC_FLC->perform |= (MXC_F_FLC_PERFORM_EN_BACK2BACK_RDS |
                         MXC_F_FLC_PERFORM_EN_MERGE_GRAB_GNT |
                         MXC_F_FLC_PERFORM_AUTO_TACC |
                         MXC_F_FLC_PERFORM_AUTO_CLKDIV);

    /* First, eliminate the unnecessary RTC handshake between clock domains. Must be set as a pair. */
    MXC_RTCTMR->ctrl |= (MXC_F_RTC_CTRL_USE_ASYNC_FLAGS |
                         MXC_F_RTC_CTRL_AGGRESSIVE_RST);
    /* Enable fast read of the RTC timer value, and fast write of all other RTC registers */
    MXC_PWRSEQ->rtc_ctrl2 |= (MXC_F_PWRSEQ_RTC_CTRL2_TIMER_AUTO_UPDATE |
                              MXC_F_PWRSEQ_RTC_CTRL2_TIMER_ASYNC_WR);
    MXC_PWRSEQ->rtc_ctrl2 &= ~(MXC_F_PWRSEQ_RTC_CTRL2_TIMER_ASYNC_RD);

    /* Clear the GPIO WUD event if not waking up from LP0 */
    /* this is necessary because WUD flops come up in undetermined state out of POR or SRST*/
    if(MXC_PWRSEQ->reg0 & MXC_F_PWRSEQ_REG0_PWR_FIRST_BOOT ||
       !(MXC_PWRMAN->pwr_rst_ctrl & MXC_F_PWRMAN_PWR_RST_CTRL_POR)) {
        /* Clear GPIO WUD event and configuration registers, globally */
        MXC_PWRSEQ->reg1 |= (MXC_F_PWRSEQ_REG1_PWR_CLR_IO_EVENT_LATCH |
			     MXC_F_PWRSEQ_REG1_PWR_CLR_IO_CFG_LATCH);
        MXC_PWRSEQ->reg1 &= ~(MXC_F_PWRSEQ_REG1_PWR_CLR_IO_EVENT_LATCH |
			      MXC_F_PWRSEQ_REG1_PWR_CLR_IO_CFG_LATCH);
    } else {
	/* Unfreeze the GPIO by clearing MBUS_GATE, when returning from LP0 */
	MXC_PWRSEQ->reg1 &= ~(MXC_F_PWRSEQ_REG1_PWR_MBUS_GATE);
	/* LP0 wake-up: Turn off special switch to eliminate ~50nA of leakage on VDD12 */
	MXC_PWRSEQ->reg1 &= ~MXC_F_PWRSEQ_REG1_PWR_SRAM_NWELL_SW;
    }
    
    /* Turn on retention regulator */
    MXC_PWRSEQ->reg0 |= (MXC_F_PWRSEQ_REG0_PWR_RETREGEN_RUN |
                         MXC_F_PWRSEQ_REG0_PWR_RETREGEN_SLP);

    /* Turn on Auto GPIO Freeze/UnFreeze in sleep modes */
    MXC_PWRSEQ->reg1 |= MXC_F_PWRSEQ_REG1_PWR_AUTO_MBUS_GATE;

    /* Adjust settings in the retention controller for fastest wake-up time */
    MXC_PWRSEQ->retn_ctrl0 |= (MXC_F_PWRSEQ_RETN_CTRL0_RC_REL_CCG_EARLY |
                               MXC_F_PWRSEQ_RETN_CTRL0_RC_POLL_FLASH);
    MXC_PWRSEQ->retn_ctrl0 &= ~(MXC_F_PWRSEQ_RETN_CTRL0_RC_USE_FLC_TWK);

    
    /* Set retention controller TWake cycle count to 1us to minimize the wake-up time */
    /* NOTE: flash polling (...PWRSEQ_RETN_CTRL0_RC_POLL_FLASH) must be enabled before changing POR default! */
    MXC_PWRSEQ->retn_ctrl1 = (MXC_PWRSEQ->retn_ctrl1 & ~MXC_F_PWRSEQ_RETN_CTRL1_RC_TWK) |
                             (1 << MXC_F_PWRSEQ_RETN_CTRL1_RC_TWK_POS);

    /* Improve wake-up time by changing ROSEL to 140ns */
    MXC_PWRSEQ->reg3 = (1 << MXC_F_PWRSEQ_REG3_PWR_ROSEL_POS) |
        (1 << MXC_F_PWRSEQ_REG3_PWR_FAILSEL_POS) |
        (MXC_PWRSEQ->reg3 & ~(MXC_F_PWRSEQ_REG3_PWR_ROSEL |
           MXC_F_PWRSEQ_REG3_PWR_FLTRROSEL));
    
    /* Enable RTOS Mode: Enable 32kHz clock synchronizer to SysTick external clock input */
    MXC_CLKMAN->clk_ctrl |= MXC_F_CLKMAN_CLK_CTRL_RTOS_MODE;

    /* Set this so all bits of PWR_MSK_FLAGS are active low to mask the corresponding flags */
    MXC_PWRSEQ->pwr_misc |= MXC_F_PWRSEQ_PWR_MISC_INVERT_4_MASK_BITS;

    /* Clear this bit to get the latest PT */
    MXC_PWRMAN->pt_regmap_ctrl &= ~MXC_F_PWRMAN_PT_REGMAP_CTRL_ME02A_MODE;

    /* Enable FPU on Cortex-M4, which occupies coprocessor slots 10 & 11 */
    /* Grant full access, per "Table B3-24 CPACR bit assignments". */
    /* DDI0403D "ARMv7-M Architecture Reference Manual" */
    SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
    __DSB();
    __ISB();

	// ensure that we're using the undivided 96MHz clock source
	MXC_CLKMAN->clk_ctrl = ((MXC_CLKMAN->clk_ctrl & ~MXC_F_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT) | (MXC_V_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT_96MHZ_RO));

	SystemCoreClockUpdate();
}


uint8_t board_read_bcd_switches(void)
{
	// returns value of the 10-position switches (S1 & S2)
	uint8_t value = 0;
	static const uint8_t c_s1[10] = 
	{
		0, 				// 0
		(S00),			// 1
		(S01),			// 2
		(S01|S00), 		// 3
		(S02),			// 4
		(S02|S00), 		// 5
		(S02|S01), 		// 6
		(S02|S01|S00), 	// 7
		(S03),			// 8
		(S03|S00)		// 9
	};
	static const uint8_t c_s2[10] = 
	{
		0, 				// 0
		(S10),			// 1
		(S11),			// 2
		(S11|S10), 		// 3
		(S12),			// 4
		(S12|S10), 		// 5
		(S12|S11), 		// 6
		(S12|S11|S10), 	// 7
		(S13),			// 8
		(S13|S10)		// 9
	};

	uint8_t s = ~GPIO_InGet(&bcd_switches);
	uint8_t s1 = s & (S00|S01|S02|S03);
	uint8_t s2 = s & (S10|S11|S12|S13);
	uint8_t i;
	for(i=0;i<ARRAY_COUNT(c_s1);i++)
	{
		if( c_s1[i] == s1 )
		{
			value = i*10;
			break;
		}
	}
	for( i=0;i<ARRAY_COUNT(c_s2); i++)
	{
		if( c_s2[i] == s2 )
		{
			value +=i;
			break;
		}
	}
	return value;
}

void board_wait( uint32_t ms )
{
	// delay at least 'ms' milliseconds using the timestamp timer
	uint32_t one_ms = SYS_SysTick_GetFreq()/1000;
	uint32_t delay = ms*one_ms;
	uint32_t now = TMR32_GetCount(TIMESTAMP_TIMER);
	while( TMR32_GetCount(TIMESTAMP_TIMER) - now < delay );
}

void max3510x_spi_xfer( max3510x_t *p, void *pv_in, const void *pv_out, uint8_t count )
{
	// used by the MAX3510x module to interface with the hardware

	spim_req_t req;
	req.ssel = 0;
	req.deass = 1;
	req.tx_data = pv_out;
	req.rx_data = pv_in;
	req.width = SPIM_WIDTH_1;
	req.len = count;

	if( SPIM_Trans( MXC_SPIM0, &req ) != count )
	{
		while( 1 ); // fatal error -- step into CSL to determine reason
	}

	// Wait for transaction to complete
	while( SPIM_Busy( MXC_SPIM0 ) != E_NO_ERROR )
	{
		// fatal
	}
}

void board_printf( const char *p_format, ... )
{
	// prints to the UART connected to the J3 header on the smart valve PCB

	static char s_buff[256];

	va_list args;
	va_start(args, p_format);
	vsnprintf( s_buff, sizeof(s_buff)-1, p_format, args );
	
	board_uart_write( s_buff, strlen(s_buff) );
	
	va_end(args);
}

// board-specific UART implimenation 

uint16_t board_uart_write( void *pv, uint16_t length )
{
	return UART_Write( s_p_uart, (uint8_t *)pv, length);
}

uint16_t board_uart_read( void *pv, uint16_t length )
{
	return UART_Read2( s_p_uart, (uint8_t *)pv, length, NULL);
}

void board_tdc_interrupt_enable(bool b)
{
	if( b )
	{
		NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(g_board_max3510x_int.port));
		GPIO_IntEnable( &g_board_max3510x_int );
	}
	else
	{
		GPIO_IntDisable( &g_board_max3510x_int );
	}
}
bool board_flash_write( const void *p_data, uint16_t size )
{
	return false;
}

void board_flash_read( void *p_data, uint16_t size )
{
}
