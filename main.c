#include "stm32f4xx.h"

#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dac.h"

#include "hd44780.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/semphr.h"

uint64_t u64Ticks=0; // Counts OS ticks (default = 1000Hz).
uint64_t u64IdleTicks=0; // Value of u64IdleTicksCnt is copied once per sec.
uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.

volatile short stateReg = 0;

struct inputData{
	int type;
	int data;
};

struct controllerData{
	int chan;
	int type;
	int data;
	int upper;
	int lower;
	int perdiv;
};

QueueHandle_t userInput = 0;
QueueHandle_t cha1Data = 0;
QueueHandle_t cha2Data = 0;
QueueHandle_t LCDData = 0;

void init_GPIO(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		GPIO_InitTypeDef gpio;
		gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &gpio);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;
		GPIO_Init(GPIOD, &gpio);

		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio.GPIO_Speed = GPIO_Speed_25MHz;
		GPIO_Init(GPIOD, &gpio);

		return;
}

int init_DAC(){
        /* DAC Periph clock enable */

        RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_APB1Periph_DAC, ENABLE);

        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        DAC_InitTypeDef DAC_InitStructure;
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
        DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
        DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable; //Enable Buffer
        DAC_Init(DAC_Channel_2, &DAC_InitStructure);

        DAC_Cmd(DAC_Channel_2,ENABLE);
        return 0;
}

void loadCustomChar(){
	uint8_t ch[8] = {6,8,6,0,10,14,10,0};
	uint8_t arrow[8] = {0,8,12,14,12,8,0,0};
	lcd_set_user_char(1,ch);
	lcd_set_user_char(2,arrow);
	return;
}

int readGPIOpin(GPIO_TypeDef *PORT,int pinNum){
	return (PORT->IDR & (1 << (pinNum-1)));
}

int decodeKeyData(int keyNum){
	switch(keyNum){
			case 2:
				return 7;
	    	case 3:
	    		return 8;
	    	case 4:
	    		return 9;
	    	case 6:
	    		return 4;
	    	case 7:
	    		return 5;
	    	case 8:
	    		return 6;
	    	case 10:
	    		return 1;
	    	case 11:
	    		return 2;
	    	case 12:
	    		return 3;
	    	case 15:
	    		return 0;
	    	default:
	    		return -1;
	}

}

int getDigits(int a){
	int i = 0;
	while(a > 0){
		a/=10;
		i++;
	}
	return i;
}

void taskRotaryEncoderReader(){
	short lock = 0;
    while(1){
    	unsigned short capture = GPIOB->IDR;
    	if(!(capture & 0x2000) && !lock){
    		struct inputData toSend;
    		toSend.type = 0;
    		toSend.data = (capture & 0x4000) ? 1 : -1;
    		xQueueSend(userInput,&toSend,0);
    		vTaskDelay(5);
    		lock = 1;
    	}else if((capture & 0x2000)){
    		lock = 0;
    	}

    	/*if((capture & 0x8000)){
    		struct inputData toSend;
    		toSend.type = 1;
    		toSend.data = 0;
    		xQueueSend(userInput,&toSend,0);
    	}*/ //Fking Trash like Encoder, Implement later
    }
}

void taskKeypadReader(){
	short cols = 0x0100;
	int lock = 1;
    while(1){
    	GPIOD->BSRRH = 0x0F00;
    	GPIOD->BSRRL = cols;


    	unsigned short capture = GPIOD->IDR;
    	//capture = ~capture;
    	capture &= 0xF000;

    	struct inputData toSend;
    	toSend.type = 1;
    	if(capture&& lock){
			switch(cols){
				case 0x0100:
					toSend.data = 1;
					break;
				case 0x0200:
					toSend.data = 2;
					break;
				case 0x0400:
					toSend.data = 3;
					break;
				case 0x0800:
					toSend.data = 4;
					break;
			}

			switch(capture){
				case(0x1000):
					toSend.data += 0;
					break;
				case(0x2000):
					toSend.data += 4;
					break;
				case(0x4000):
				    toSend.data += 8;
					break;
				case(0x8000):
					toSend.data += 12;
					break;
			}

			xQueueSend(userInput,&toSend,0);
			vTaskDelay(200);
			lock = 0;
    	}else if(!capture && !lock){
    		lock = 1;
    	}

    	if(cols != 0x0800)
    		cols = cols << 1;
    	else
    		cols = 0x0100;
    }
}

void taskLCDController(){
    lcd_init();
    loadCustomChar();
    char screen[17];
    while(1){
    	if(xQueueReceive(LCDData,screen,portMAX_DELAY )){
    		screen[16] = '\0';

    		if(strlen(screen) < 16){
    			for(int i = strlen(screen); i < 16; i++)
    				screen[i] = ' ';
    		}

    		//Update data;
    		lcd_set_xy(0, 0);
    		lcd_out(screen);
    		lcd_set_xy(0, 1);
    		lcd_out(screen+8);
    	}
    }
}

void taskVoltageController2(){
	int value;
    while(1){
    	struct controllerData pxRxedData;
    	if(xQueueReceive(cha2Data,&pxRxedData,portMAX_DELAY )){
    		if(pxRxedData.type == 1)
    			value = pxRxedData.data;

    	}
    	if(stateReg & 0x0001){
    		DAC_SetChannel2Data(DAC_Align_12b_R, ((double)value / 200) * 2814 + 21); // Mask 12 bits
    	}else{
    		DAC_SetChannel2Data(DAC_Align_12b_R, 0); // Mask 12 bits
    	}
    	vTaskDelay(100);
    }
}

void taskUIController(){

	/*strcpy(screen,"C4Labs");
	vTaskDelay(500);
	strcpy(screen,"PowerSupply");
	vTaskDelay(500);
	strcpy(screen,"v1.0 Booting");
	vTaskDelay(500);
	strcpy(screen,"");*/

	struct controllerData cha1Volt = {1,1,5,200,5,1};
	struct controllerData cha1Amp = {1,2,10,3000,0,10};
	struct controllerData cha2Volt = {2,1,5,200,5,1};
	struct controllerData cha2Amp = {2,2,10,3000,0,10};

	struct controllerData *currentSelected = &cha1Volt;

	char screen[17] = "";
	sprintf(screen, "c1  %3d.V %4dmA", cha1Volt.data, cha1Amp.data);
	screen[0] = '\x01';
	screen[7] = screen[6];
	screen[6] = '.';
	if(cha1Volt.data < 10)
		screen[5] = '0';
	if(cha2Volt.data < 100)
		screen[4] = '0';
	screen[3] = '\x02';
	xQueueSend(LCDData,screen,0);

    while(1){

    	struct inputData pxRxedUserData;
    	if(xQueueReceive(userInput, &pxRxedUserData, 0)) {
    		switch(pxRxedUserData.type){
    			case 0:
    				currentSelected->data += currentSelected->perdiv * pxRxedUserData.data;
    				if(currentSelected->data > currentSelected->upper) currentSelected->data = currentSelected-> upper;
    				if(currentSelected->data < currentSelected->lower) currentSelected->data = currentSelected-> lower;
    				break;
    			case 1:
    				if(decodeKeyData(pxRxedUserData.data) == -1){
    					switch(pxRxedUserData.data){
    					    case 0:

    					    	break;
    					    case 1:
    					    	currentSelected = &cha1Volt;
    					    	break;
    					    case 5:
    					        currentSelected = &cha1Amp;
    					        break;
    					    case 9:
    					        currentSelected = &cha2Volt;
    					        break;
    					    case 13:
    					        currentSelected = &cha2Amp;
    					        break;
    					    case 16:
    					    	stateReg ^= 0x0001;
    					    	break;
    					}
    				}else{
    					if(currentSelected->type == 1)
    						sprintf(screen,"\x01%d Volt:",currentSelected->chan);
    					else
    						sprintf(screen,"\x01%d Amps:",currentSelected->chan);

    					int out = decodeKeyData(pxRxedUserData.data);
    					sprintf(screen+8,"%d",out);
    					xQueueSend(LCDData,screen,0);

    					short err = 0;

    					int i;
    					for(i = getDigits(currentSelected->upper)-2; i >= 0; i--){
    						while(!xQueueReceive(userInput, &pxRxedUserData, 0));
    						if(pxRxedUserData.type==1 && decodeKeyData(pxRxedUserData.data)!=-1){
    							out *= 10;
    							out += decodeKeyData(pxRxedUserData.data);
    							sprintf(screen+8,"%d",out);
    							xQueueSend(LCDData,screen,0);
    						}else{
    							err = -1;
    							break;
    						}
    					}

    					if(out > currentSelected->upper){
    						xQueueSend(LCDData,"Err Upper Limit\0",0);
    						vTaskDelay(1200);
    						err = -1;
    					}
    					if(out < currentSelected->lower){
    						xQueueSend(LCDData,"Err Lower Limit\0",0);
    						vTaskDelay(1200);
    						err = -1;
    					}

    					if(err != -1){
    						char sending[17] = "";
    						sprintf(sending,"Accepted %d",out);
    						xQueueSend(LCDData,sending,0);
    						vTaskDelay(900);
    						currentSelected->data = out;
    					}
    				}

    				break;
    		}

    		if(currentSelected->chan == 1)
    			xQueueSend(cha1Data,currentSelected,500);
    		else
    			xQueueSend(cha2Data,currentSelected,500);

			if(currentSelected->chan == 1){
				sprintf(screen, "c1  %3d.V %4dmA", cha1Volt.data, cha1Amp.data);
				screen[0] = '\x01';
				screen[7] = screen[6];
				screen[6] = '.';
				if(cha1Volt.data < 10)
					screen[5] = '0';
				if(cha1Volt.data < 100)
					screen[4] = '0';
			}else{
				sprintf(screen, "c2  %3d.V %4dmA", cha2Volt.data, cha2Amp.data);
				screen[0] = '\x01';
				screen[7] = screen[6];
				screen[6] = '.';
				if(cha2Volt.data < 10)
					screen[5] = '0';
				if(cha2Volt.data < 100)
					screen[4] = '0';
			}
			if(currentSelected->type == 1){
				screen[3] = '\x02';
			}else{
				screen[9] = '\x02';
			}
			xQueueSend(LCDData,screen,0);
    	}
    	vTaskDelay(10);
    }
}

int main(void)
{
	init_GPIO();
	init_DAC();
	//DAC_SetChannel2Data(DAC_Align_12b_R, 21); // Mask 12 bits

	userInput = xQueueCreate(1,sizeof(struct inputData));
	LCDData = xQueueCreate(10,sizeof(char)*17);
	cha1Data = xQueueCreate(10,sizeof(struct controllerData));
	cha2Data = xQueueCreate(10,sizeof(struct controllerData));


	xTaskCreate( taskRotaryEncoderReader, ( const char * ) "RotEnco", configMINIMAL_STACK_SIZE,NULL, 3, NULL );
	xTaskCreate( taskKeypadReader, ( const char * ) "Keypad", configMINIMAL_STACK_SIZE,NULL, 3, NULL );
	xTaskCreate( taskLCDController, ( const char * ) "LCD", configMINIMAL_STACK_SIZE,NULL, 3, NULL );
	xTaskCreate( taskUIController, ( const char * ) "UI", configMINIMAL_STACK_SIZE,NULL, 3, NULL );
	xTaskCreate( taskVoltageController2, ( const char * ) "VC2", configMINIMAL_STACK_SIZE,NULL, 4, NULL );


    vTaskStartScheduler(); // This should never return.


    while(1)
    {

    }

    return 1;
}

void vApplicationTickHook( void ) {
    ++u64Ticks;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ----------------------------------------------------------------------------
void vApplicationIdleHook( void ) {
    ++u64IdleTicksCnt;
}

// A required FreeRTOS function.
// ----------------------------------------------------------------------------
void vApplicationMallocFailedHook( void ) {
    configASSERT( 0 ); // Latch on any failure / error.
}
