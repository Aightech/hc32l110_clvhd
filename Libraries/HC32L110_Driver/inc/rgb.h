#include "gpio.h"

/**
 *******************************************************************************
 ** \defgroup RgbGroup RGB LED(Red, Green, Blue) WS2812B Driver on pin P01
 **
 ******************************************************************************/

/**
 * @brief Initialize the RGB LED WS2812 on pin P01
 */
void RGB_Init();

/**
 * @brief Send the reset signal to the RGB LED WS2812 on pin P01
 */
void RGB_Reset();

/**
 * @brief Send the reset signal to the RGB LED WS2812 on pin P01
 * 
 * @param red Red value
 * @param green Green value
 * @param blue Blue value
 */
void RGB_SetColor(uint8_t red, uint8_t green, uint8_t blue);