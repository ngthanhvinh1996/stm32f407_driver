/*
 * 017rtc_lcd.c
 *
 *  Created on: Jul 7, 2021
 *      Author: vinhkuto
 */

#include "stm32f407xx.h"
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK		16000000

#define PRINT_LCD	0

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

int main(void)
{

	RTC_date_t cur_date;
	RTC_time_t cur_time;

#if PRINT_LCD
	printf("RTC test...\n");
#endif

	lcd_init();

	lcd_print_string("RTC test ...");

	if(ds1307_init())
	{
#if PRINT_LCD
		printf("RTC init has fail...\n");
#endif
		while(1);
	}

	init_systick_timer(1);

	cur_date.day = FRIDAY;
	cur_date.date = 15;
	cur_date.month = 1;
	cur_date.year = 21;

	ds1307_set_current_date(&cur_date);

	cur_time.hour = 4;
	cur_time.minutes = 25;
	cur_time.second = 41;
	cur_time.time_format = TIME_FORMAT_12HRS_PM;

	ds1307_set_current_time(&cur_time);

	ds1307_get_current_date(&cur_date);
	ds1307_get_current_time(&cur_time);

	char *am_pm;
	if(cur_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (cur_time.time_format) ? "PM" : "AM";
#if PRINT_LCD
		printf("Current time: %s %s\n", (time_to_string(&cur_time)), am_pm);
#endif
		lcd_display_clear();
		lcd_display_return_home();
		lcd_print_string((time_to_string(&cur_time)));
		lcd_print_string(am_pm);
	}
	else
	{
#if PRINT_LCD
		printf("Current time: %s\n", (time_to_string(&cur_time)));
#endif
		lcd_print_string((time_to_string(&cur_time)));
	}

#if PRINT_LCD
	printf("Current date: %s <%s>\n", (date_to_string(&cur_date)), get_day_of_week(cur_date.day));
#endif

	lcd_set_cursor(2, 1);
	lcd_print_string((date_to_string(&cur_date)));
	lcd_print_char(' ');
	lcd_print_string(get_day_of_week(cur_date.day));

	while(1)
	{

	}
	return 0;
}

void SysTick_Handler(void)
{
	RTC_date_t cur_date;
	RTC_time_t cur_time;

	ds1307_get_current_date(&cur_date);

	lcd_set_cursor(1, 1);

	char *am_pm;
	if(cur_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (cur_time.time_format) ? "PM" : "AM";
#if PRINT_LCD
		printf("Current time: %s %s\n", (time_to_string(&cur_time)), am_pm);
#endif
		lcd_print_string((time_to_string(&cur_time)));
		lcd_print_string(am_pm);
	}
	else
	{
#if PRINT_LCD
		printf("Current time: %s\n", (time_to_string(&cur_time)));
#endif
		lcd_print_string((time_to_string(&cur_time)));
	}

	ds1307_get_current_time(&cur_time);

#if PRINT_LCD
	printf("Current date: %s <%s>\n", (date_to_string(&cur_date)), get_day_of_week(cur_date.day));
#endif
	lcd_set_cursor(2, 1);
	lcd_print_string((date_to_string(&cur_date)));
	lcd_print_char(' ');
	lcd_print_string(get_day_of_week(cur_date.day));
}
