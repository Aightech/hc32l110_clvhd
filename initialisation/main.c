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
    delay1ms(1000);
    RGB_SetColor(0, 0, 0);
    RGB_SetColor(0, 0, 0);

    // set the address output pin as output
    Gpio_InitIOExt(CLVHD_ADD0_PORT, CLVHD_ADD0_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD1_PORT, CLVHD_ADD1_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD2_PORT, CLVHD_ADD2_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD3_PORT, CLVHD_ADD3_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_InitIOExt(CLVHD_ADD4_PORT, CLVHD_ADD4_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);

    // setup I2C
    Gpio_InitIOExt(3, 5, GpioDirOut, FALSE, FALSE, TRUE, FALSE);
    Gpio_InitIOExt(3, 6, GpioDirIn, FALSE, FALSE, TRUE, FALSE);
    Gpio_SetFunc_I2C_DAT_P35();
    Gpio_SetFunc_I2C_CLK_P36();

    set_address(8);

    // // Clock and Data
    // Gpio_InitIOExt(CLVHD_CLK_PORT, CLVHD_CLK_PIN, GpioDirIn, TRUE, FALSE, FALSE, 0);
    // Gpio_ClearIrq(CLVHD_CLK_PORT, CLVHD_CLK_PIN);
    // Gpio_EnableIrq(CLVHD_CLK_PORT, CLVHD_CLK_PIN, GpioIrqLow);
    // EnableNvic(PORT3_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
    // Gpio_InitIOExt(CLVHD_DATA_PORT, CLVHD_DATA_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);

    // Chained INPUT
    Gpio_InitIOExt(CLVHD_DIN_PORT, CLVHD_DIN_PIN, GpioDirIn, TRUE, FALSE, FALSE, FALSE);

    // Chained OUTPUT
    Gpio_InitIOExt(CLVHD_DOUT_PORT, CLVHD_DOUT_PIN, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    Gpio_SetIO(CLVHD_DOUT_PORT, CLVHD_DOUT_PIN, HIGH);
}

volatile uint8_t flag;

void PORT3_IRQHandler(void)
{
    /** Clear interrupt flag before handling, otherwise it will be triggered twice */
    Gpio_ClearIrq(CLVHD_CLK_PORT, CLVHD_CLK_PIN);
    if (Gpio_GetIrqStat(CLVHD_CLK_PORT, CLVHD_CLK_PIN))
    {
        if (!module_initiated)
        {
            if (GPIO_GetPinIn(CLVHD_DIN_PORT, CLVHD_DIN_PIN)) // if the previous module is not initiated
            {
                RGB_SetColor(r, g, b);
                RGB_SetColor(r, g, b);
                module_initiated = 1;
                // Acknowledge the initialisation (by pulling the data line low for 50us)
                // Gpio_SetIO(CLVHD_DATA_PORT, CLVHD_DATA_PIN, LOW);
                // Gpio_SetIO(CLVHD_DATA_PORT, CLVHD_DATA_PIN, HIGH);

                // Propagate the initialisation to the next module
                Gpio_SetIO(CLVHD_DOUT_PORT, CLVHD_DOUT_PIN, HIGH);

                // set the current address
                set_address(module_address);
            }
            else
            {
                module_address++;
            }
        }
    }
}

int main(void)
{
    Gpio_InitIOExt(2, 6, GpioDirIn, FALSE, TRUE, FALSE, FALSE);
    Gpio_InitIOExt(2, 5, GpioDirIn, FALSE, TRUE, FALSE, FALSE);
    Gpio_InitIOExt(1, 4, GpioDirOut, FALSE, FALSE, FALSE, FALSE);
    // Gpio_InitIOExt(0, 1, GpioDirOut, FALSE, FALSE, FALSE, FALSE);

    CLVHD_ConfigPin();
    // RGB_Init();
    // RGB_Reset();
    uint8_t oldState = GPIO_GetPinIn(2, 6);
    uint8_t count = 0;
    uint8_t stopState = GPIO_GetPinIn(2, 5);
    
    while(1){
        GPIO_SetPinOutHigh(1,4);
        uint8_t stopState = GPIO_GetPinIn(2, 5);
        if(stopState==1){
            uint8_t pinState = GPIO_GetPinIn(2, 6);
            if(pinState>oldState){
                RGB_SetColor(0,255,0);
                delay1ms(10);
                count=count+1;
            }else{
                RGB_SetColor(0,0,0);
                delay1ms(10);
            }
            oldState = pinState;
            uint8_t stopState = GPIO_GetPinIn(2, 5);
        }else{
            RGB_SetColor(0,0,255);
            delay1ms(10);
            break;
        }
    }
    delay1ms(3000);
    GPIO_SetPinOutLow(1,4);
    set_address(count);
    
    return 0;
}