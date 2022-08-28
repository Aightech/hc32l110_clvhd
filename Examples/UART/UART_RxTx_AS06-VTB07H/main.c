#include "uart.h"
#include "base_timer.h"

volatile uint8_t u8RxData[16] = {0x00}, u8RxFlg = 0, u8RxPos = 0;

void RxIntCallback(void)
{
    u8RxData[u8RxPos++] = UART1_RxReceive();
    u8RxPos = u8RxPos % 16;
    u8RxFlg = 1;
}

int main(void)
{
    uint8_t i;
    uint16_t period;
    uint32_t pclk;

    stc_uart_irq_cb_t stcUartIrqCb;
    stc_bt_config_t baseTimerConfig;

    /**
     * Set PCLK = HCLK = Clock source to 24MHz
     */
    Clk_Init(ClkFreq24Mhz, ClkDiv1, ClkDiv1);
    // Enable peripheral clock
    CLK_EnablePeripheralClk(ClkPeripheralBaseTim);
    CLK_EnablePeripheralClk(ClkPeripheralGpio); // GPIO clock is required, equal to M0P_CLOCK->PERI_CLKEN_f.GPIO = 1;
    CLK_EnablePeripheralClk(ClkPeripheralUart1);
    // Set P01,P02 as UART1 TX,RX
    Gpio_SetFunc_UART1_TXD_P01();
    Gpio_SetFunc_UART1_RXD_P02();
    // Config UART1
    UART1_SetDoubleBaud(TRUE);
    UART1_SetMode(UartMode1);
    UART1_SetMultiModeOff();
    stcUartIrqCb.pfnRxIrqCb = RxIntCallback;
    stcUartIrqCb.pfnTxIrqCb = NULL;
    stcUartIrqCb.pfnRxErrIrqCb = NULL;
    Uart1_SetCallback(&stcUartIrqCb);
    UART1_EnableRxReceivedIrq();
    UART1_ClearRxReceivedStatus();
    UART1_ClearTxSentStatus();
    UART1_EnableRx();

    // Config timer1 as baudrate source
    baseTimerConfig.enMD = BtMode2;
    baseTimerConfig.enCT = BtTimer;
    BaseTim1_Init(&baseTimerConfig);
    // Set timer period
    pclk = Clk_GetPClkFreq();
    period = UARTx_CalculatePeriod(pclk, 1, 115200);
    BASE_TIM1_SetARR(period);
    BASE_TIM1_SetCounter16(period);
    // Start timer
    BASE_TIM1_Run();

    Uart1_TxString("PCLK:");
    Uart1_TxHex((uint8_t *)&pclk, 4);
    Uart1_TxChar('\n');
    while (1)
    {
        if (u8RxFlg)
        {
            u8RxFlg = 0;
            for (i = 0; i < 16; i++)
            {
                Uart1_TxChar(u8RxData[i]);
            }
            Uart1_TxChar('\n');
        }
        delay1ms(200);
    }
}
