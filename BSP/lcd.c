/*
 * lcd.c
 *
 *  Created on: Jul 7, 2021
 *      Author: vinhkuto
 */

#include "lcd.h"
#include "stm32f407xx.h"

static void write_4_bits(uint8_t value);
static void lcd_enable( void );
static void mdelay(uint8_t cnt);
static void udelay(uint8_t cnt);


void lcd_send_command(uint8_t cmd)
{
	/* RS = 0 for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* r/nW = 0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits((cmd >> 4) & 0x0F);
	write_4_bits(cmd & 0x0F);
}

/*
 * This function sends a character to the LCD
 * Here we used 4 bit parallel data transmission
 * First higher nibble of the data will be sent on to the data lines D4, D5, D6, D7
 * Then lower nibble of the data will be sent on to the data lines D4, D5, D6, D7
 */
void lcd_print_char(uint8_t data)
{
	/* RS = 1 for LCD user data */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	/* r/nW = 0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits((data >> 4) & 0x0F); 	//Sending higher nibble
	write_4_bits(data & 0x0F);	 		//Sending lower nibble
}

void lcd_print_string(char *message)
{
	do
	{
		lcd_print_char((uint8_t) *message++);
	}
	while(*message != '\0');
}

void lcd_init(void)
{
	//1. Configure the gpio pins which are used for lcd connections
	GPIO_Handle_t lcd_signal;

	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Pin RS
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcd_signal);

	//Pin RW
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	//Pin EN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	//Pin D4
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	//Pin D5
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	//Pin D6
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	//Pin D7
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	//Keep all pins in low level
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);


	//2. Do the LCD initialization
	mdelay(40);

	/* RS = 0, For LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* RnW = 0, Writing to LCD */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x03);

	mdelay(5);

	write_4_bits(0x03);

	udelay(150);

	write_4_bits(0x03);
	write_4_bits(0x02);

	//Function set command
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	//display ON and cursor ON
	lcd_send_command(LCD_CMD_DON_CURON);

	//Display clear
	lcd_display_clear();

	//Entry mode set
	lcd_send_command(LCD_CMD_INCADD);
}

/* writes 4 bits of data/command on to D4, D5, D6, D7 lines*/
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x01));

	lcd_enable();
}

static void lcd_enable( void )
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100); //execution time > 37 micro seconds
}

void lcd_display_clear(void)
{
	lcd_send_command(LCD_CMD_DIS_CLEAR);

	/*
	 * Check page number 24 of data sheet
	 * Display clear command execution wait time around 2ms
	 */
	mdelay(2);
}

void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);

	/*
	 * Check page number 24 of data sheet
	 * Display clear command execution wait time around 2ms
	 */
	mdelay(2);
}

/*
 * Set LCD to a specified location given by row and column information
 * Row Number (1 to 2)
 * Column number (1 to 16) Assuming a 2 x 16 characters display
 */
void lcd_set_cursor(uint8_t row, uint8_t column)
{
	column--;
	switch(row)
	{
		case 1:
			/* Set cursor to 1st row address and add index */
			lcd_send_command((column |= 0x80));
			break;
		case 2:
			/* Set cursor to 2nd row address and add index */
			lcd_send_command((column |= 0xC0));
			break;
		default:
			break;
	}
}
static void mdelay(uint8_t cnt)
{
	for(uint32_t i = 0; i < ( cnt * 1000 ); i++);
}

static void udelay(uint8_t cnt)
{
	for(uint32_t i = 0; i < ( cnt * 1 ); i++);
}
