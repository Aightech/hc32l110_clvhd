#include "rgb.h"

#define RGB_LED_HIGH *((volatile uint32_t *)((uint32_t) & M0P_GPIO->P0OUT)) |= (0x2); // set bit 1 to 1 (P01)
#define RGB_LED_LOW *((volatile uint32_t *)((uint32_t) & M0P_GPIO->P0OUT)) &= (0xFFFFFFFD); // set bit 1 to 0 (P01)

/**
 * @brief Send the value 1 to the RGB LED WS2812 on pin P01
 *
 * @param rgb_led Pointer to the RGB_LED_t struct
 */
void RGB_Write1()
{
    RGB_LED_HIGH;
    __NOP();
    __NOP();
    __NOP();
    RGB_LED_LOW;
    __NOP();
    __NOP();
}

/**
 * @brief Send the value 0 to the RGB LED WS2812 on pin P01
 *
 * @param rgb_led Pointer to the RGB_LED_t struct
 */
void RGB_Write0()
{
    RGB_LED_HIGH;
    RGB_LED_LOW;
    __NOP();
    __NOP();
    __NOP();
}

/**
 * @brief Send the reset signal to the RGB LED WS2812 at the specified port and pin
 *
 * @param rgb_led Pointer to the RGB_LED_t struct
 */
void RGB_Reset()
{
    RGB_LED_LOW;
    delay100us(3);
}

/**
 * @brief Send the specified byte to the RGB LED WS2812 at the specified port and pin
 *
 * @param rgb_led Pointer to the RGB_LED_t struct
 * @param byte Byte to send
 */
void RGB_WriteByte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80)
        {
            RGB_Write1();
        }
        else
        {
            RGB_Write0();
        }
        byte <<= 1;
    }
}

/**
 * @brief Send the specified color to the RGB LED WS2812 at the specified port and pin
 *
 * @param rgb_led Pointer to the RGB_LED_t struct
 * @param r Red value
 * @param g Green value
 * @param b Blue value
 */
void RGB_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    RGB_WriteByte(g);
    RGB_WriteByte(r);
    RGB_WriteByte(b);
}

/**
 * @brief Initialize the RGB LED WS2812 at the specified port and pin
 *
 * @param rgb_led Pointer to the RGB_LED_t struct
 */
void RGB_Init()
{
    // set the clock
    Clk_SwitchTo(ClkRCH);
    Clk_Init(ClkFreq24Mhz, ClkDiv1, ClkDiv1);

    // Set the pin as output
    Gpio_InitIOExt(0, 1,
                   GpioDirOut, // Output
                   FALSE,      // No pull up
                   FALSE,      // No pull down
                   FALSE,      // No open drain
                   FALSE);     // High driver capability
}