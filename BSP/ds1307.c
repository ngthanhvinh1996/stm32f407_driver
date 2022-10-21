/*
 * ds1307.c
 *
 *  Created on: Jul 7, 2021
 *      Author: vinhkuto
 */

#include "ds1307.h"

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t bcd_to_binary(uint8_t value);
static uint8_t binary_to_bcd(uint8_t value);
static void number_to_string(uint8_t number, char *buf);

I2C_Handle_t g_ds1307I2cHandle;

//Returns 1: CH = 1, init fail
//Returns 0: CH = 0, init success
uint8_t ds1307_init(void)
{
	//1 Initialize the I2C pin
	ds1307_i2c_pin_config();

	//2. Initialize the I2C peripheral
	ds1307_i2c_config();

	//3. Enable the I2C peripheral
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	//4. Make clock halt = 0
	ds1307_write(0x00, DS1307_ADDR_SECONDS);

	//5. Read back clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SECONDS);

	return ( (clock_state >> 7) & 0x01 );
}

void ds1307_set_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds, hours;
	seconds = binary_to_bcd(rtc_time->second);
	seconds &= ~( 1 << 7 );
	ds1307_write(seconds, DS1307_ADDR_SECONDS);

	ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_SECONDS);

	hours = binary_to_bcd(rtc_time->hour);

	if(rtc_time->time_format == TIME_FORMAT_24HRS)
	{
		hours &= ~( 1 << 6 );
	}
	else
	{
		hours |= ( 1 << 6 );
		if(rtc_time->time_format == TIME_FORMAT_12HRS_PM)
		{
			hours |= ( 1 << 5 );
		}
		else
		{
			hours &= ~( 1 << 5 );
		}
	}
	ds1307_write(hours, DS1307_ADDR_HOUR);
}

void ds1307_get_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds, hours;

	seconds = ds1307_read(DS1307_ADDR_SECONDS);

	seconds &= ~( 1 << 7 );

	rtc_time->second = bcd_to_binary(seconds);

	rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MINUTES));

	hours = ds1307_read(DS1307_ADDR_HOUR);

	if(hours & ( 1 << 6 ))
	{
		//12 Hours format
		if(hours & ( 1 << 5))
		{
			//12 hours PM
			rtc_time->time_format = TIME_FORMAT_12HRS_PM;
		}
		else
		{
			//12 hours AM
			rtc_time->time_format = TIME_FORMAT_12HRS_AM;
		}
		hours &= ~(0x03 << 5); // Clear bit 6 & 5
	}
	else
	{
		//24 Hours format
		rtc_time->time_format = TIME_FORMAT_24HRS;
	}
	rtc_time->hour = bcd_to_binary(hours);
}

void ds1307_set_current_date(RTC_date_t *rtc_date)
{
	uint8_t day, date, month, year;

	day = binary_to_bcd(rtc_date->day);
	ds1307_write(day, DS1307_ADDR_DAY);

	date = binary_to_bcd(rtc_date->date);
	ds1307_write(date, DS1307_ADDR_DATE);

	month = binary_to_bcd(rtc_date->month);
	ds1307_write(month, DS1307_ADDR_MONTH);

	year = binary_to_bcd(rtc_date->year);
	ds1307_write(year, DS1307_ADDR_YEAR);
}

void ds1307_get_current_date(RTC_date_t *rtc_date)
{

}

static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/*
	 * I2C_SDA --> PB6
	 * I2C_SCL --> PB7
	 */
	//SDA
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF4;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;

	GPIO_Init(&i2c_sda);

	//SCL
	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF4;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;

	GPIO_Init(&i2c_scl);
}

static void ds1307_i2c_config(void)
{
	g_ds1307I2cHandle.pI2Cx = DS1307_I2C;
	g_ds1307I2cHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_ds1307I2cHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_Init(&g_ds1307I2cHandle);
}

static void ds1307_write(uint8_t value, uint8_t reg_addr)
{
	uint8_t tx[2];

	tx[0] = reg_addr;
	tx[1] = value;

	I2C_MasterSendData(&g_ds1307I2cHandle, (uint8_t *)tx, 2, DS1307_I2C_ADDRESS, DISABLE);
}

static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t data;

	I2C_MasterSendData(&g_ds1307I2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, DISABLE);

	I2C_MasterReceiveData(&g_ds1307I2cHandle, &data, 1, DS1307_I2C_ADDRESS, DISABLE);

	return data;
}

static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t m, n, bin;

	m = (uint8_t)((value >> 4) & (uint8_t)0x0F);
	n = (uint8_t)(value & (uint8_t)0x0F);
	bin = ( (m*10) + n );

	return bin;
}
static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m, n, bcd;

	bcd = value;

	if(value >= 10)
	{
		m = value/10;
		n = value%10;
		bcd = (uint8_t)( (m << 4) | n );
	}

	return bcd;
}

char* get_day_of_week(uint8_t i)
{
	char *days[] = {"SUNDAY", "MONDAY", "TUESDAY", "WESNESDAY", "THUSDAY", "FRIDAY", "SATURDAY"};

	return days[i - 1];
}

//hh:mm:ss
char* time_to_string(RTC_time_t *rtc_time)
{
	static char buf[9];

	buf[2] = buf[5] = ':';

	//hh
	number_to_string(rtc_time->hour, &buf[0]);

	//mm
	number_to_string(rtc_time->minutes, &buf[3]);

	//ss
	number_to_string(rtc_time->second, &buf[6]);

	buf[8] = '\0';

	return buf;
}

//dd:mm:yy
char* date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2] = buf[5] = '/';

	//dd
	number_to_string(rtc_date->date, &buf[0]);

	//mm
	number_to_string(rtc_date->month, &buf[3]);

	//yy
	number_to_string(rtc_date->year, &buf[6]);

	buf[8] = '\0';

	return buf;
}

static void number_to_string(uint8_t number, char *buf)
{
	if(number < 10)
	{
		buf[0] = '0';
		buf[1] = number+48;
	}
	else if(number >= 10 && number <= 99)
	{
		buf[0] = (number / 10) + 48;
		buf[1] = (number % 10) + 48;
	}
}
