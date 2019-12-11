#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

#include "LCD_4bit.h"
//#include "delay.h"
#include <stdint.h>

void Delay(int t)
{
	volatile int c = 0;
	while (t--)
	{
		c = 10000; //adjust c to a suiting time interval
		while (c--)
			;
	}
}

static uint8_t lcd_read_status(void)
{
	uint8_t status;

	SET_LCD_DATA_DIR_IN
	SET_LCD_RS(0)
	SET_LCD_RW(1)
	Delay(1);
	SET_LCD_E(1)
	Delay(1);
	status = GET_LCD_DATA_IN << 4;
	SET_LCD_E(0)
	Delay(1);
	SET_LCD_E(1)
	Delay(1);
	status |= GET_LCD_DATA_IN;
	SET_LCD_E(0)
	SET_LCD_DATA_DIR_OUT
	return (status);
}

void wait_while_busy(void)
{
	for (; lcd_read_status() & LCD_BUSY_FLAG_MASK;)
		;
}

void lcd_write_4bit(uint8_t c)
{
	SET_LCD_RW(0)
	SET_LCD_E(1)
	SET_LCD_DATA_OUT(c & 0x0F)
	Delay(1);
	SET_LCD_E(0)
	Delay(1);
}

void lcd_write_cmd(uint8_t c)
{
	wait_while_busy();

	SET_LCD_RS(0)
	lcd_write_4bit(c >> 4);
	lcd_write_4bit(c);
}

static void lcd_write_data(uint8_t c)
{
	wait_while_busy();

	SET_LCD_RS(1)
	lcd_write_4bit(c >> 4);
	lcd_write_4bit(c);
}

void lcd_putchar(char c)
{
	lcd_write_data(c);
}

void lcd_init_port(void)
{
	/* Enable clocks for peripherals        */
	ENABLE_LCD_PORT_CLOCKS

	/* Set Pin Mux to GPIO */
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT] = PORT_PCR_MUX(1);
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT + 1] = PORT_PCR_MUX(1);
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT + 2] = PORT_PCR_MUX(1);
	PIN_DATA_PORT->PCR[PIN_DATA_SHIFT + 3] = PORT_PCR_MUX(1);
	PIN_E_PORT->PCR[PIN_E_SHIFT] = PORT_PCR_MUX(1);
	PIN_RW_PORT->PCR[PIN_RW_SHIFT] = PORT_PCR_MUX(1);
	PIN_RS_PORT->PCR[PIN_RS_SHIFT] = PORT_PCR_MUX(1);
}

void Init_LCD(void)
{
	/* initialize port(s) for LCD */
	lcd_init_port();

	/* Set all pins for LCD as outputs */
	SET_LCD_ALL_DIR_OUT
	Delay(100);
	SET_LCD_RS(0)
	lcd_write_4bit(0x3);
	Delay(100);
	lcd_write_4bit(0x3);
	Delay(10);
	lcd_write_4bit(0x3);
	lcd_write_4bit(0x2);
	lcd_write_cmd(0x28);
	lcd_write_cmd(0x0C);
	lcd_write_cmd(0x06);
	lcd_write_cmd(0x80);
}

void Set_Cursor(uint8_t column, uint8_t row)
{
	uint8_t address;

	address = (row * 0x40) + column;
	address |= 0x80;
	lcd_write_cmd(address);
}

void Clear_LCD(void)
{
	lcd_write_cmd(0x01);
	Set_Cursor(0, 0);
}

void Print_LCD(char *string)
{
	while (*string)
	{
		lcd_putchar(*string++);
	}
}

int main(void) {
	lcd_init();
	lcd_clear();
	set_cursor(0,0);
	lcd_print(" Hello, World!");

    return 0 ;
}
