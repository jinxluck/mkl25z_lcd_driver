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

// Assuming that all switches are on port E
#define SW_UP_POS (21)
#define SW_DN_POS (29)
#define SW_LT_POS (30)
#define SW_RT_POS (23)
#define SW_CR_POS (22)

// Macro to read switches returns state of switches, active low
#define READ_SWITCHES (PTE->PDIR)

#define RED_LED_POS (18) //port b
#define GREEN_LED_POS (19) //port b
#define BLUE_LED_POS (1) //port d
#define SW1_SHIFT (5)  //port a

#define MASK(x) (1UL << (x))


void init_5way_switch(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; /* enable clock for port E */
	/* Select GPIO and enable pull-up resistors (if present) for pins
	 connected to switches */
	PORTE->PCR[SW_UP_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
	PORT_PCR_PE_MASK;
	PORTE->PCR[SW_DN_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
	PORT_PCR_PE_MASK;
	PORTE->PCR[SW_LT_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
	PORT_PCR_PE_MASK;
	PORTE->PCR[SW_RT_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
	PORT_PCR_PE_MASK;
	PORTE->PCR[SW_CR_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
	PORT_PCR_PE_MASK;
	/* Set port C bits 0-3, 7 to inputs */
	PTE->PDDR &= ~( GPIO_PDDR_PDD(SW_UP_POS) | GPIO_PDDR_PDD(SW_DN_POS)
			| GPIO_PDDR_PDD(SW_LT_POS) | GPIO_PDDR_PDD(SW_RT_POS)
			| GPIO_PDDR_PDD(SW_CR_POS));
}

void init_RGB_LEDs(void) {
// Enable clock to ports B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	;
// Make 3 pins GPIO
	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);
// Set ports to outputs
	PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
	PTD->PDDR |= MASK(BLUE_LED_POS);
}

void control_RGB_LEDs(unsigned int red_on, unsigned int green_on, unsigned
int blue_on, unsigned int yellow_on, unsigned int magenta_on) {
	if (red_on) {
		PTB->PCOR = MASK(RED_LED_POS);
	} else {
		PTB->PSOR = MASK(RED_LED_POS);
	}
	if (green_on) {
		PTB->PCOR = MASK(GREEN_LED_POS);
	} else {
		PTB->PSOR = MASK(GREEN_LED_POS);
	}
	if (blue_on) {
		PTD->PCOR = MASK(BLUE_LED_POS);
	} else {
		PTD->PSOR = MASK(BLUE_LED_POS);
	}
	if(yellow_on) {
		PTD->PCOR = MASK(GREEN_LED_POS) | MASK(RED_LED_POS);
	} else {
		PTD->PSOR = MASK(GREEN_LED_POS) | MASK(RED_LED_POS);
	}
	if(magenta_on) {

		PTD->PCOR |= MASK(RED_LED_POS) | MASK(BLUE_LED_POS);
	} else {
		PTD->PSOR |= MASK(RED_LED_POS) | MASK(BLUE_LED_POS);
	}
}

void Delay(int t) {
	volatile int c = 0;
	while (t > 0) {
		t--;
		c = 10000; //adjust c to a suiting time interval
		while (c > 0)
			c--;
			;
	}
}

static uint8_t lcd_read_status(void) {
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

void wait_while_busy(void) {
	for (; lcd_read_status() & LCD_BUSY_FLAG_MASK;)
		;
}

void lcd_write_4bit(uint8_t c) {

	SET_LCD_RW(0)
	SET_LCD_E(1)
	SET_LCD_DATA_OUT(c & 0x0F)
	Delay(1);
	SET_LCD_E(0)
	Delay(1);
}

void lcd_write_cmd(uint8_t c) {
	wait_while_busy();

	SET_LCD_RS(0)
	lcd_write_4bit(c >> 4);
	lcd_write_4bit(c);
}

static void lcd_write_data(uint8_t c) {
	wait_while_busy();

	SET_LCD_RS(1)
	lcd_write_4bit(c >> 4);
	lcd_write_4bit(c);
}

void lcd_putchar(char c) {
	lcd_write_data(c);
}

void lcd_init_port(void) {
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

void Init_LCD(void) {
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

void Set_Cursor(uint8_t column, uint8_t row) {
	uint8_t address;

	address = (row * 0x40) + column;
	address |= 0x80;
	lcd_write_cmd(address);
}

void Clear_LCD(void) {
	lcd_write_cmd(0x01);
	Set_Cursor(0, 0);
}

void Print_LCD(char *string) {
	while (*string) {
		lcd_putchar(*string++);
	}
}

void Test_Switches_And_LCD(void) {
	unsigned switch_code;
	printf("before loop\n");
	while (1) {
		Set_Cursor(0, 0);
		switch_code = ~READ_SWITCHES;
		control_RGB_LEDs(
				(switch_code & MASK(SW_UP_POS)),
				(switch_code & MASK(SW_RT_POS)),
				(switch_code & MASK(SW_LT_POS)),
				(switch_code & MASK(SW_DN_POS)),
				(switch_code & MASK(SW_CR_POS)));
		if (switch_code & MASK(SW_UP_POS)) {
			printf(" up \n");
			Clear_LCD();
			Print_LCD(" Up ");
		} else if (switch_code & MASK(SW_DN_POS)) {
			printf(" down \n");
			Clear_LCD();
			Print_LCD(" Down ");
		} else if (switch_code & MASK(SW_LT_POS)) {
			printf(" left \n");
			Clear_LCD();
			Print_LCD(" Left ");
		} else if (switch_code & MASK(SW_RT_POS)) {
			printf(" right \n");
			Clear_LCD();
			Print_LCD(" Right ");
		} else if (switch_code & MASK(SW_CR_POS)) {
			printf(" center \n");
			Clear_LCD();
			Print_LCD(" Center");
		} else {
			Print_LCD(" ");
		}
	}
}

void test_switches(void) {
	unsigned switch_code;
	while (1) {
		switch_code = ~READ_SWITCHES;
		control_RGB_LEDs((switch_code & MASK(SW_UP_POS)),
				(switch_code & MASK(SW_RT_POS)),
				(switch_code & MASK(SW_LT_POS)),
				(switch_code & MASK(SW_LT_POS)),
				(switch_code & MASK(SW_LT_POS)));
	}
}

int main(void) {

	init_5way_switch();
	init_RGB_LEDs();
	//test_switches();

	Init_LCD();
	Clear_LCD();
	Set_Cursor(0, 0);
//	Print_LCD(" Hello, World!");
	printf("before test\n");
	Test_Switches_And_LCD();
	printf("after test");
	return 0;

}
