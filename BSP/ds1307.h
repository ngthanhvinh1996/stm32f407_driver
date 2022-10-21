/*
 * ds1307.h
 *
 *  Created on: Jul 7, 2021
 *      Author: vinhkuto
 */

#ifndef DS1307_H_
#define DS1307_H_

#include "stm32f407xx.h"

/* Application configurable items */
#define DS1307_I2C				I2C1
#define DS1307_I2C_GPIO_PORT	GPIOB
#define DS1307_I2C_SDA_PIN		GPIO_PIN_NO_7
#define DS1307_I2C_SCL_PIN		GPIO_PIN_NO_6
#define DS1307_I2C_SPEED		I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD			GPIO_PIN_PU


/* Register address*/
#define DS1307_ADDR_SECONDS		0x00
#define DS1307_ADDR_MINUTES		0x01
#define DS1307_ADDR_HOUR		0x02
#define DS1307_ADDR_DAY			0x03
#define DS1307_ADDR_DATE		0x04
#define DS1307_ADDR_MONTH		0x05
#define DS1307_ADDR_YEAR		0x06
#define DS1307_ADDR_CTRL		0x07

#define TIME_FORMAT_12HRS_AM	0
#define TIME_FORMAT_12HRS_PM	1
#define TIME_FORMAT_24HRS		2

#define DS1307_I2C_ADDRESS		0x68

#define SUNDAY					1
#define MONDAY					2
#define TUESDAY					3
#define WESNESDAY				4
#define THUSDAY					5
#define FRIDAY					6
#define SATURDAY				7


typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_date_t;

typedef struct
{
	uint8_t second;
	uint8_t minutes;
	uint8_t hour;
	uint8_t time_format;
}RTC_time_t;

//Function prototypes
uint8_t ds1307_init(void);

void ds1307_set_current_time(RTC_time_t *rtc_time);
void ds1307_get_current_time(RTC_time_t *rtc_time);

void ds1307_set_current_date(RTC_date_t *rtc_date);
void ds1307_get_current_date(RTC_date_t *rtc_date);

char* get_day_of_week(uint8_t day);
char* time_to_string(RTC_time_t *rtc_time);
char* date_to_string(RTC_date_t *rtc_date);

#endif /* DS1307_H_ */
