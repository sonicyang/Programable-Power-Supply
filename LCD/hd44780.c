#include "hd44780.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"

#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"

void delay(int i)
{
	vTaskDelay(i/1000);
}

void lcd_delay(void) {
	vTaskDelay(1);
}

void lcd_init() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &gpio);

	delay(500);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_13);
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_12);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
	GPIO_ResetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_13);
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_SetBits(GPIOE, GPIO_Pin_11);
	GPIO_SetBits(GPIOE, GPIO_Pin_12);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_SetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_13);
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	GPIO_ResetBits(GPIOE, GPIO_Pin_10); // E V
	GPIO_SetBits(GPIOE, GPIO_Pin_7);
	GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
	GPIO_ResetBits(GPIOE, GPIO_Pin_13);
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
	GPIO_SetBits(GPIOE, GPIO_Pin_10); // E ^
	delay(50000);
	lcd_set_state(LCD_ENABLE,CURSOR_DISABLE,NO_BLINK);
	lcd_clear();
	lcd_send(0x06,COMMAND);
}

void lcd_set_user_char(uint8_t char_num, uint8_t * char_data) {
	uint8_t i;
	lcd_send(((1<<6) | (char_num * 8) ), COMMAND);
	for (i=0;i<=7;i++) {
		lcd_send(char_data[i],DATA);
	}
	lcd_send((1<<7), COMMAND);
}

void lcd_set_xy(uint8_t x, uint8_t y)  {
	if (y==0) {
		lcd_send( ((1<<7) | (x)),COMMAND);
	} else {
		lcd_send( ((3<<6) | (x)),COMMAND);
	}
}


void lcd_out(char * txt) {
	while(*txt) {
		lcd_send(*txt,DATA);
		txt++;
	}
}

void lcd_clear(void) {
	lcd_send(0x01,COMMAND);
}

void lcd_set_state(lcd_state state, cursor_state cur_state, cursor_mode cur_mode)  {
	if (state==LCD_DISABLE)  {
		lcd_send(0x08,COMMAND);
	} else {
		if (cur_state==CURSOR_DISABLE) {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0C,COMMAND);
			} else {
				lcd_send(0x0D,COMMAND);
			}
		} else  {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0E,COMMAND);
			} else {
				lcd_send(0x0F,COMMAND);
			}
		}
	}
}



void lcd_send(uint8_t byte, dat_or_comm dc)  {

	GPIO_ResetBits(GPIOE, GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_7 | GPIO_Pin_10);
	//LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);

	if (dc) {
		GPIO_SetBits(GPIOE, GPIO_Pin_7);
		//LCD_PORT->BSRR=LCD_CD_BS;
	}

	if (byte & 0x10) {
		GPIO_SetBits(GPIOE, GPIO_Pin_14);
		//LCD_PORT->BSRR=LCD_DB4_BS;
	}
	if (byte & 0x20) {
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		//LCD_PORT->BSRR=LCD_DB5_BS;
	}
	if (byte & 0x40) {
		GPIO_SetBits(GPIOE, GPIO_Pin_11);
		//LCD_PORT->BSRR=LCD_DB6_BS;
	}
	if (byte & 0x80) {
		GPIO_SetBits(GPIOE, GPIO_Pin_12);
		//LCD_PORT->BSRR=LCD_DB7_BS;
	}

	GPIO_SetBits(GPIOE, GPIO_Pin_10);
	//LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);
	//LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();


	GPIO_ResetBits(GPIOE, GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14);
	//LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC );

	if (byte & 0x01) {
		GPIO_SetBits(GPIOE, GPIO_Pin_14);
		//LCD_PORT->BSRR=LCD_DB4_BS;
	}
	if (byte & 0x02) {
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		//LCD_PORT->BSRR=LCD_DB5_BS;
	}
	if (byte & 0x04) {
		GPIO_SetBits(GPIOE, GPIO_Pin_11);
		//LCD_PORT->BSRR=LCD_DB6_BS;
	}
	if (byte & 0x08) {
		GPIO_SetBits(GPIOE, GPIO_Pin_12);
		//LCD_PORT->BSRR=LCD_DB7_BS;
	}

	GPIO_SetBits(GPIOE, GPIO_Pin_10);
	//LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);
	//LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();


}
