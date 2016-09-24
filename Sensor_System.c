/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/
#include "LPC17xx.h"
#include "stdio.h"
#include "string.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "led7seg.h"
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "light.h"
#include "lpc17xx_uart.h"

#define DARK 100
#define SHOCK 75
#define ALARM_TIME_DETECT 500
#define ALARM_TIME_COOP 1000
#define SAMPLING_PERIOD 40

int CONTROL = 1;
int mean, max;
int mode = 0;

extern int asm_variance(int x, int mean, int reset, int* max);

//EINT3 Interrupt Handler
void EINT3_IRQHandler(void){
	int max;
	//determine whether GPIO Interrupt P2.10 has occured
	if((LPC_GPIOINT->IO2IntStatF>>10) & 0x1){
		//clear GPIO Interrupt 2.10
		LPC_GPIOINT->IO2IntClr = 1<<10;
		CONTROL = 0;

		max = asm_variance(mean,mean,1, &max);
	}
}

#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);

static uint32_t notes[] = {
        2272, // A - 440 Hz
        2024, // B - 494 Hz
        3816, // C - 262 Hz
        3401, // D - 294 Hz
        3030, // E - 330 Hz
        2865, // F - 349 Hz
        2551, // G - 392 Hz
        1136, // a - 880 Hz
        1012, // b - 988 Hz
        1912, // c - 523 Hz
        1703, // d - 587 Hz
        1517, // e - 659 Hz
        1432, // f - 698 Hz
        1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs) {

    uint32_t t = 0;

    if (note > 0) {

        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            t += note;
        }

    }
    else {
    	Timer0_Wait(durationMs);
        //delay32Ms(0, durationMs);
    }
}

static uint32_t getNote(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'G')
        return notes[ch - 'A'];

    if (ch >= 'a' && ch <= 'g')
        return notes[ch - 'a' + 7];

    return 0;
}

static uint32_t getDuration(uint8_t ch)
{
    if (ch < '0' || ch > '9')
        return 400;

    /* number of ms */

    return (ch - '0') * 50;
}

static uint32_t getPause(uint8_t ch)
{
    switch (ch) {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void playSong(uint8_t *song) {
    uint32_t note = 0;
    uint32_t dur  = 0;
    uint32_t pause = 0;

    /*
     * A song is a collection of tones where each tone is
     * a note, duration and pause, e.g.
     *
     * "E2,F4,"
     */

    while(*song != '\0') {
        note = getNote(*song++);
        if (*song == '\0')
            break;
        dur  = getDuration(*song++);
        if (*song == '\0')
            break;
        pause = getPause(*song++);

        playNote(note, dur);
        //delay32Ms(0, pause);
        Timer0_Wait(pause);

    }
}

static uint8_t * song = (uint8_t*)"g8,";
        //(uint8_t*)"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,A2.A2,H4,F4,G4,F8,";
        //"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,";

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;// SPI receive clock
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;// SPI MISO
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;// SPI MOSI
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;// SPI SSEL (Slave Select)
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);
}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	// Initialize I2C2 pin connect
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;// SDA - data line
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;// SCL - clock line
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	// Enable I2C1 operation
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
	// Initialize button SW4
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum= 0;
	PinCfg.OpenDrain= 0;
	PinCfg.Pinmode= 0;
	PinCfg.Portnum= 1;
	PinCfg.Pinnum= 31;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(1, 1<<31, 0);

	// Initialize button SW3
	PinCfg.Funcnum = 1;	// Function: EINT0
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize RGB
	PinCfg.Funcnum= 0;
	PinCfg.OpenDrain= 0;
	PinCfg.Pinmode= 0;
	PinCfg.Portnum= 2;
	PinCfg.Pinnum= 0;
	PINSEL_ConfigPin(&PinCfg);

		GPIO_SetDir(2, 1<<0, 1);
}

void pinsel_uart3(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart3;
    pinsel_uart3();
    //supply power & setup working par.s for uart3
    UART_Init(LPC_UART3, &uartCfg);
    //enable transmit for uart3
    UART_TxCmd(LPC_UART3, ENABLE);
    }

int main (void) {


	int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;
    uint8_t SW4;

    init_i2c();
    init_ssp();
    init_GPIO();
    led7seg_init();



    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
    light_init();
    light_enable();

    //Enable GPIO interrupt
    LPC_GPIOINT->IO2IntEnF |= 1<<10;
    //Enable EINT0 interrupt
    NVIC_EnableIRQ(EINT3_IRQn);

    //---- Speaker ------>

    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    //<---- Speaker ------

    oled_clearScreen(OLED_COLOR_BLACK);
    char lptr[20];
    char vptr[20];
    char mptr[20];
    char mxptr[20];
    char ALARM[20];
    char DT[20];
    char CO[20];

    uint8_t data = 0 , databack = 0;
    uint32_t index;
    uint8_t message[101], report1[101],report2[101];
    int isShock;
    int reset = 1;

    acc_read(&x, &y, &z);
    mean = z;
    int largest_lux = 0;
    int largest_acc = 0;

    while (1==1)
    {
    	SW4 = (GPIO_ReadValue(1) >> 31) & 0x01;
    	if (SW4 ==0){
        	mode = (mode + 1) % 2;
        	largest_lux = 0;
        	largest_acc = 0;
    	}

    	if (mode==0){

        Timer0_Wait(SAMPLING_PERIOD);
    	acc_read(&x, &y, &z);
        isShock = asm_variance(z, mean, reset, &max);

    	unsigned int l = light_read();
        sprintf(lptr, "Light: %u ", l);
        sprintf(vptr,"Var_acce: %d", isShock);
        sprintf(mptr,"Mean: %d", mean);
        sprintf(mxptr,"Max: %d", max);
        sprintf(DT,"DETECT MODE");

        oled_clearScreen(OLED_COLOR_BLACK);
        oled_putString(0, 0, (uint8_t *) DT, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(0, 10, (uint8_t *) lptr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(0, 20, (uint8_t *) vptr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(0, 30, (uint8_t *) mptr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
        oled_putString(0, 40, (uint8_t *) mxptr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

        if (l<100)
        {
           reset = 0;

           if(isShock > SHOCK)
             {
           CONTROL = 1;
              while (CONTROL == 1)
              {
               playSong(song);
               uint8_t RGB = (GPIO_ReadValue(2) >> 0) & 0x01;
               if (RGB==0) GPIO_SetValue(2,1<<0);
               else GPIO_ClearValue(2,1<<0);
               sprintf(ALARM,"ALARM");
               oled_putString(30, 55, (uint8_t *) ALARM, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
               }
             }
         }
        sprintf(ALARM,"NO ALARM");
        oled_putString(25, 55, (uint8_t *) ALARM, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
     } else if (mode==1){
               oled_clearScreen(OLED_COLOR_BLACK);
    	       sprintf(CO, "CooperativeMode");
               oled_putString(0, 0, (uint8_t *) CO, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

               init_uart();


               index = 0;
         	   do {
         		   data = 0;
         		   UART_Receive(LPC_UART3, &data, 1, BLOCKING);
         		   if (data != '\r' && data != 0) {
         			   message[index] = data;
         			   index++;
         			   UART_Send(LPC_UART3, &data, 1, BLOCKING);
         		   }
         		   SW4 = (GPIO_ReadValue(1) >> 31) & 0x01;
         		   if (SW4 ==0){
         			  mode = (mode + 1) % 2;
         		   }

         			if (light_read()>largest_lux){
         			    largest_lux = light_read();
         			}

         			acc_read(&x, &y, &z);
         			isShock = asm_variance(z, mean, reset, &max);
         			if (isShock > largest_acc){
         			    largest_acc = isShock;
         			}

         		} while ((index<100) && (data != '\r') && (mode == 1));

         		message[index] = 0;
         		databack = 13;			// 13 = "\r" carriage return
         		UART_Send(LPC_UART3, &databack, 1, BLOCKING);
         		databack = 10;			// 10 = "\n" new line
        		UART_Send(LPC_UART3, &databack, 1, BLOCKING);
        		databack = 0;			//  0 = NULL character
        		UART_Send(LPC_UART3, &databack, 1, BLOCKING);
         		if (!strcmp(message,"report")) {
         			sprintf(report1, "    Luminance(largest) = %d lux\r\n", largest_lux);
         			UART_SendString(LPC_UART3, &report1);
         			sprintf(report2, "    Accelerometer(largest) = %d lux\r\n", largest_acc);
         			UART_SendString(LPC_UART3, &report2);
         		}
     }

}
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

