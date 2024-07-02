#include "gpio.h"
#include "i2c.h"
#include "rgb.h"

// pin 34, 33, 32, 03, 15 are used for address output
// these pins can be found on the hc32l110 datasheet
// pin numbers 1-20, which shows their location on the 
// controller are labelled on the diagram. The table 
// underneath shows the pin name for each pin (e.g. pin 
// number 2 is labelled as P01). In this case, pin number
// 18 (P34), 17 (P33), 16 (P32), 7 (P03), and 8 (P15) are
// used. The first digit in the pin name refers to the 
// port whereas the second digit refers to the pin. Port
// is essentially a group of pins. 

#define CLVHD_ADD0_PORT 3
#define CLVHD_ADD0_PIN 4
#define CLVHD_ADD1_PORT 3
#define CLVHD_ADD1_PIN 3
#define CLVHD_ADD2_PORT 3
#define CLVHD_ADD2_PIN 2
#define CLVHD_ADD3_PORT 0
#define CLVHD_ADD3_PIN 3
#define CLVHD_ADD4_PORT 1
#define CLVHD_ADD4_PIN 5

// I2C on pins SDA: 35, SCL: 36
#define CLVHD_SDA_PORT 3
#define CLVHD_SDA_PIN 5
#define CLVHD_SCL_PORT 3
#define CLVHD_SCL_PIN 6

uint8_t r = 255, g = 0, b = 0;

uint8_t module_address = 0x01;
uint8_t module_initiated = 0;

#define CLVHD_DIN_PORT (0)
#define CLVHD_DIN_PIN (2)
#define CLVHD_DOUT_PORT (3)
#define CLVHD_DOUT_PIN (1)
#define CLVHD_DATA_PORT (3)
#define CLVHD_DATA_PIN (5)
#define CLVHD_CLK_PORT (3)
#define CLVHD_CLK_PIN (6)

#define HIGH 1
#define LOW 0

void Gpio_SetIO(uint8_t port, uint8_t pin, uint8_t value)
{
    if (value)
    {
        GPIO_SetPinOutHigh(port, pin);
    }
    else
    {
        GPIO_SetPinOutLow(port, pin);
    }
}

void set_address(uint8_t address)
{
    Gpio_SetIO(CLVHD_ADD0_PORT, CLVHD_ADD0_PIN, address & 0x01); // set the first digit of the address (binary)
    Gpio_SetIO(CLVHD_ADD1_PORT, CLVHD_ADD1_PIN, (address >> 1) & 0x01); // set the second digit of the address (binary)
    Gpio_SetIO(CLVHD_ADD2_PORT, CLVHD_ADD2_PIN, (address >> 2) & 0x01); // set the third digit of the address (binary)
    Gpio_SetIO(CLVHD_ADD3_PORT, CLVHD_ADD3_PIN, (address >> 3) & 0x01); // set the fourth digit of the address (binary)
    Gpio_SetIO(CLVHD_ADD4_PORT, CLVHD_ADD4_PIN, (address >> 4) & 0x01); // set the fifth digit of the address (binary)
    module_address = address;
}

void CLVHD_ConfigPin(void)
{
    // configure P01 to control the neopixel
    RGB_Init();
    RGB_Reset();

    // set the address output pin as output
    Gpio_InitIOExt(CLVHD_ADD0_PORT, CLVHD_ADD0_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD1_PORT, CLVHD_ADD1_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD2_PORT, CLVHD_ADD2_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD3_PORT, CLVHD_ADD3_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD4_PORT, CLVHD_ADD4_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);

    Gpio_InitIOExt(3, 1, GpioDirIn, FALSE, TRUE, FALSE, FALSE); // Stop input
    Gpio_InitIOExt(3, 6, GpioDirIn, FALSE, TRUE, FALSE, 0); // CLK interrupt pin 
    Gpio_InitIOExt(0, 2, GpioDirOut, TRUE, FALSE, FALSE, FALSE); // Stop output
    Gpio_ClearIrq(3, 6);
    Gpio_EnableIrq(3, 6, GpioIrqRising);
    EnableNvic(PORT3_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
}

volatile uint8_t flag;

uint8_t count = 0;
uint8_t stop = HIGH;
uint8_t total = 6; // total number of modules for testing 

void PORT3_IRQHandler(void)
{
    /** Clear interrupt flag before handling, otherwise it will be triggered twice */
    uint8_t stopState = GPIO_GetPinIn(3, 1);
    GPIO_SetPinOutHigh(0,2);
    Gpio_ClearIrq(3, 6);
    if(stopState==1){

        if (Gpio_GetIrqStat(3, 6)) // Interrupt not triggered. Can leave this block blank
        {
            RGB_SetColor(0,255,0);
            RGB_SetColor(0,255,0);
            delay1ms(10);
        }else{                     // Interrupt triggered by rising edge
            RGB_SetColor(255,0,0);
            RGB_SetColor(255,0,0);
            count=count+1;
            delay1ms(10); 
        }
    }else{                         // Pull chip select low (Stops the counting)
        if(Gpio_GetIrqStat(3,6)){
            stop = LOW;
        }else{
            stop = LOW;
        }
    }
}

// Trying to use interrupts 

int main(void)
{ 
    CLVHD_ConfigPin();

    const int colors[6][3] = {
        {255, 0, 0},   // Red
        {255, 127, 0}, // Orange
        {255, 255, 0}, // Yellow
        {0, 255, 0},   // Green
        {0, 0, 255},   // Blue
        {75, 0, 130}   // Purple 
    };

    while (1){
        GPIO_SetPinOut(0,2,stop);
        set_address(count);
        if(stop==LOW){
            RGB_SetColor(colors[count-1][0],colors[count-1][1],colors[count-1][2]);
            RGB_SetColor(colors[count-1][0],colors[count-1][1],colors[count-1][2]);
            delay1ms(10); 
        }else{
            RGB_SetColor(0, 0, 255);
            RGB_SetColor(0, 0, 255);
            delay1ms(10);
        }
    }
    
    return 0;
}

