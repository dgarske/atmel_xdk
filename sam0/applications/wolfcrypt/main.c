/* main.c
 *
 * Copyright (C) 2006-2015 wolfSSL Inc.
 *
 * This file is part of wolfSSL. (formerly known as CyaSSL)
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA
 */

#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfcrypt/benchmark/benchmark.h>
#include <wolfcrypt/test/test.h>
#include <stdio.h>
#include <asf.h>
#include <delay.h>
#include <rtc_calendar.h>

#include "conf_uart_serial.h"

static struct usart_module cdc_uart_module;
struct rtc_module rtc_instance;

/** SysTick counter to avoid busy wait delay. */
volatile uint32_t gu32MsTicks = 0;

typedef struct func_args {
    int    argc;
    char** argv;
    int    return_code;
} func_args;

static func_args args = { 0 };


/* Local Functions */
double current_time(int reset);
void configure_rtc_calendar(void);
void HardFault_HandlerC(uint32_t *hardfault_args);

/* Hard fault handler */
void HardFault_HandlerC(uint32_t *hardfault_args)
{
    /* These are volatile to try and prevent the compiler/linker optimizing them
    away as the variables never actually get used.  If the debugger won't show the
    values of the variables, make them global my moving their declaration outside
    of this function. */
    volatile uint32_t stacked_r0;
	volatile uint32_t stacked_r1;
	volatile uint32_t stacked_r2;
	volatile uint32_t stacked_r3;
	volatile uint32_t stacked_r12;
	volatile uint32_t stacked_lr;
    volatile uint32_t stacked_pc;
	volatile uint32_t stacked_psr;
	volatile uint32_t _CFSR;
	volatile uint32_t _HFSR;
	volatile uint32_t _DFSR;
	volatile uint32_t _AFSR;
	volatile uint32_t _BFAR;
	volatile uint32_t _MMAR;

	stacked_r0 = ((uint32_t)hardfault_args[0]);
	stacked_r1 = ((uint32_t)hardfault_args[1]);
	stacked_r2 = ((uint32_t)hardfault_args[2]);
	stacked_r3 = ((uint32_t)hardfault_args[3]);
	stacked_r12 = ((uint32_t)hardfault_args[4]);
	stacked_lr = ((uint32_t)hardfault_args[5]);
	stacked_pc = ((uint32_t)hardfault_args[6]);
	stacked_psr = ((uint32_t)hardfault_args[7]);

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
	_CFSR = (*((volatile uint32_t *)(0xE000ED28)));	
											
	// Hard Fault Status Register
	_HFSR = (*((volatile uint32_t *)(0xE000ED2C)));

	// Debug Fault Status Register
	_DFSR = (*((volatile uint32_t *)(0xE000ED30)));

	// Auxiliary Fault Status Register
	_AFSR = (*((volatile uint32_t *)(0xE000ED3C)));

	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	_MMAR = (*((volatile uint32_t *)(0xE000ED34)));
	// Bus Fault Address Register
	_BFAR = (*((volatile uint32_t *)(0xE000ED38)));

    printf ("\n\nHard fault handler (all numbers in hex):\n");
    printf ("R0 = %x\n", stacked_r0);
    printf ("R1 = %x\n", stacked_r1);
    printf ("R2 = %x\n", stacked_r2);
    printf ("R3 = %x\n", stacked_r3);
    printf ("R12 = %x\n", stacked_r12);
    printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
    printf ("PC [R15] = %x  program counter\n", stacked_pc);
    printf ("PSR = %x\n", stacked_psr);
    printf ("CFSR = %x\n", _CFSR);
    printf ("HFSR = %x\n", _HFSR);
    printf ("DFSR = %x\n", _DFSR);
    printf ("AFSR = %x\n", _AFSR);
    printf ("MMAR = %x\n", _MMAR);
    printf ("BFAR = %x\n", _BFAR);

    // Break into the debugger
	__asm("BKPT #0\n");
}

__attribute__( ( naked ) ) 
void HardFault_Handler(void)
{
	__asm(
		"  mov r0, #4          \n"
		"  mov r1, lr          \n"
		"  tst r0, r1          \n"
		"  beq using_msp       \n"
		"  mrs r0, psp         \n"
		"  b call_c            \n"
		"using_msp:            \n"
		"  mrs r0, msp         \n"
		"call_c:               \n"
		"  ldr r2, =HardFault_HandlerC \n"
		"  bx r2               \n"
	);
}

/**
 * Configure RTC
 */
#define BUILD_SECOND (__TIME__[6] * 10 + __TIME__[7] - 528)
#define BUILD_MINUTE (__TIME__[3] * 10 + __TIME__[4] - 528)
#define BUILD_HOUR   (__TIME__[0] * 10 + __TIME__[1] - 528)

#define BUILD_DAY   (__DATE__[4] * 10 + __DATE__[5] - (__DATE__[4] == ' ' ? 368 : 528))
#define BUILD_YEAR  (__DATE__[7] * 1000 + __DATE__[8] * 100 + __DATE__[9] * 10 + __DATE__[10] - 53328)

#define BUILD_MONTH ((__DATE__[1]+__DATE__[2] == 207) ? 1  : (__DATE__[1]+__DATE__[2] == 199) ? 2  : \
                     (__DATE__[1]+__DATE__[2] == 211) ? 3  : (__DATE__[1]+__DATE__[2] == 226) ? 4  : \
                     (__DATE__[1]+__DATE__[2] == 218) ? 5  : (__DATE__[1]+__DATE__[2] == 227) ? 6  : \
                     (__DATE__[1]+__DATE__[2] == 225) ? 7  : (__DATE__[1]+__DATE__[2] == 220) ? 8  : \
                     (__DATE__[1]+__DATE__[2] == 213) ? 9  : (__DATE__[1]+__DATE__[2] == 215) ? 10 : \
                     (__DATE__[1]+__DATE__[2] == 229) ? 11 : (__DATE__[1]+__DATE__[2] == 200) ? 12 : 0)
void configure_rtc_calendar(void)
{
	/* Initialize RTC in calendar mode. */
	struct rtc_calendar_config config_rtc_calendar;
	struct rtc_calendar_time time;
    
	rtc_calendar_get_config_defaults(&config_rtc_calendar);

#ifdef ENABLE_RTC_ALARM
	struct rtc_calendar_time alarm;
	rtc_calendar_get_time_defaults(&alarm);
	alarm.year   = 2013;
	alarm.month  = 1;
	alarm.day    = 1;
	alarm.hour   = 0;
	alarm.minute = 0;
	alarm.second = 4;
	config_rtc_calendar.alarm[0].time = alarm;
	config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_YEAR;
#endif
	config_rtc_calendar.clock_24h     = true;

	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);
	rtc_calendar_enable(&rtc_instance);

	/* Set current time. */
	time.year   = BUILD_YEAR;
	time.month  = BUILD_MONTH;
	time.day    = BUILD_DAY;
	time.hour   = BUILD_HOUR;
	time.minute = BUILD_MINUTE;
	time.second = BUILD_SECOND;
	rtc_calendar_set_time(&rtc_instance, &time);
}

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;

	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}

/*
 * \brief SysTick handler used to measure precise delay.
 */
void SysTick_Handler(void)
{
	gu32MsTicks++;
}

static void systick_init(void)
{
	uint32_t cycles_per_ms = system_gclk_gen_get_hz(0);
	cycles_per_ms /= 1000;
    SysTick_Config(cycles_per_ms);
}

int main(void) 
{
    int test_num = 0;
	const uint8_t welcomeStr[] = "Atmel SAMD21 wolfCrypt Test/Benchmark\r\n";
	
	/* Initialize system */
	system_init();
    systick_init();
    delay_init();
    configure_rtc_calendar();
	configure_console();

    /* Send welcome message to UART */
	usart_write_buffer_wait(&cdc_uart_module, welcomeStr, sizeof(welcomeStr));

    do
    {
#ifndef NO_CRYPT_TEST
        printf("\nCrypt Test %d:\n", test_num);
        wolfcrypt_test(&args);
        printf("Crypt Test %d: Return code %d\n", test_num, args.return_code);
#endif

#ifndef NO_CRYPT_BENCHMARK
        printf("\nBenchmark Test %d:\n", test_num);
        benchmark_test(&args);
        printf("Benchmark Test %d: Return code %d\n", test_num, args.return_code);
#endif

        test_num++;
    } while(args.return_code == 0);
    
    return 0;
}

double current_time(int reset)
{
    double time;
	(void)reset;
    time = ((double)gu32MsTicks) / 1000;
    return time;
}
