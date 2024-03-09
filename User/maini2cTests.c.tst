#include "gpio.h"
#include "i2c.h"
#include "rgb.h"

// pin 34, 33, 32, 03, 15 are used for address output

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
uint8_t u8State = 0;
uint8_t u8Recdata[10];
uint32_t u32RecvLen = 0;
uint32_t u32SendLen = 0;

void delay(uint16_t u16cnt)
{
    uint16_t i;
    for (i = 0; i < u16cnt; i++)
    {
        __NOP();
    }
}

void receiveData()
{
    // controller - transmitting
    // ===================================
    // 0x08: Start condition sent
    // 0x10: Repeated START sent
    // 0x18: SLA+W sent, ACK received
    // 0x20: SLA+W sent, non-ACK received
    // 0x28: Data in I2C_DATA has been sent, ACK has been received
    // 0x30: Data in I2C_DATA has been sent, non-ACK received
    // 0x38: Lost Arbitration on SLA+ Read or Write Data Bytes

    // controller - receiving
    // =======================
    // 0x08: Start condition sent
    // 0x10: Repeated START sent
    // 0x38: Lost Arbitration on SLA+ Read or Write Data Bytes
    // 0x40: SLA+R sent, ACK received
    // 0x48: SLA+R sent, non-ACK received
    // 0x50: Data bytes received, ACK returne
    // 0x58: Data bytes received, not ACK returned

    // peripheral - transmitting
    // ============================
    // 0xA8: Received its own SLA+R; ACK has been returned, data will be sent and the ACK bit will be received
    // 0xB0: Losing arbitration in SLA+ read and write when master; has received its own SLA+R; has returned ACK
    // 0xB8: Data sent; ACK received
    // 0xC0: Send data, receive non-ACK
    // 0xC8: The loaded data byte has been sent; ACK received

    // peripheral - receiving
    // ========================
    // 0x60: Received (matching itself) SLA+W; received ACK
    // 0x68: own SLA+W has been received; ACK has been returned
    // 0x78: general call address received; ACK returned
    // 0x80: The previous addressing used its own slave address; data bytes have been received; ACK has been returned
    // 0x88: The previous addressing used its own slave address; data bytes have been received; non-ACK has been returned
    // 0xA0: Received stop condition or repeated start condition

    // broadcast mode
    // ======================
    // 0x70: The general call address (0x00) has been received; ACK has been received
    // 0x78: Arbitration lost in SLA+ read/write when mastering; general call address received; ACK returned
    // 0x90: The previous addressing used the general call address; data has been received; ACK has been returned
    // 0x98: The previous addressing used the general call address; data has been received; non-ACK has been returned
    // 0xA0: When statically addressed, a STOP condition or a repeated START condition is received

    // MISC
    // ============
    // 0xF8: No Status -- si is equal to 0
    // 0x00: Error / bus in undefined state
    uint8_t pr = r;
    r = g;
    g = b;
    b = pr;
    RGB_SetColor(255, 255, 255);

    u8State = I2C_GetState();
    switch (u8State)
    {
    }
    I2C_ClearIrq();
}

void setup_i2C(uint16_t freq, uint8_t isMaster, uint8_t address, func_ptr_t receiveData)
{
    // enable peripheral clock
    CLK_EnablePeripheralClk(ClkPeripheralI2c);

    stc_i2c_config_t i2c_config;
    // set the I2C configuration: the I2C ucontroller is slave, the address is 0x08
    i2c_config.enFunc = I2cMode_En; // enable I2C
    // set the baud rate at freq: Fscl = Fpclk / (8 * (1 + u8Tm))
    // Fpclk is the peripheral clock, u8Tm is the value to be set
    // get the value of Fpclk:

    i2c_config.u8Tm = 59;
    i2c_config.stcSlaveAddr.Gc = FALSE; // set the general call to 0
    if (!isMaster)
    {
        i2c_config.stcSlaveAddr.Addr = address; // set the I2C address
        i2c_config.pfnI2cCb = receiveData;      // set the callback function
    }

    // deinit I2C
    I2C_DeInit();
    // init I2C
    I2C_Init(&i2c_config);

    // enable NVIC if needed
    if (i2c_config.bTouchNvic)
    {
        EnableNvic(I2C_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
    }
    I2C_SetFunc(I2cHlm_En);  // enable I2C high speed mode
    I2C_SetFunc(I2cMode_En); // enable I2C
    if (!isMaster)
    {
        // I2C_ClearFunc(I2cStart_En); // disable I2C start
        // I2C_ClearFunc(I2cStop_En);  // disable I2C stop
        I2C_SetFunc(I2cAck_En); // enable I2C ACK
    }
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

    // setup_i2C(100, 0, 80, receiveData);

    // // enable peripheral clock
    // CLK_EnablePeripheralClk(ClkPeripheralI2c);

    // stc_i2c_config_t i2c_config;
    // // set the I2C configuration: the I2C ucontroller is slave, the address is 0x08
    // i2c_config.enFunc = I2cMode_En; // enable I2C
    // // set the baud rate at 100kHz: 24MHz / (4 * (1 + 0x47)) = 100kHz
    // i2c_config.u8Tm = 59;
    // i2c_config.stcSlaveAddr.Addr = 0x50; // set the address to 0x08
    // i2c_config.stcSlaveAddr.Gc = FALSE;      // set the general call to 0
    // i2c_config.pfnI2cCb = receiveData;       // set the callback function

    // // deinit I2C
    // I2C_DeInit();

    // // init I2C
    // I2C_Init(&i2c_config);

    // // enable NVIC if needed
    // if (i2c_config.bTouchNvic)
    // {
    //     EnableNvic(I2C_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
    // }
    // I2C_SetFunc(I2cAck_En);  // enable I2C ACK
    // I2C_SetFunc(I2cHlm_En);  // disable high speed mode
    // I2C_SetFunc(I2cMode_En); // enable I2C
    // // I2C_ClearFunc(I2cStart_En); // disable I2C start
    // // I2C_ClearFunc(I2cStop_En);  // disable I2C stop

    // enable peripheral clock
    CLK_EnablePeripheralClk(ClkPeripheralI2c);
    stc_i2c_config_t i2c_config;
    // set the I2C configuration: the I2C ucontroller is slave, the address is 0x08
    i2c_config.enFunc = I2cMode_En; // enable I2C
    i2c_config.u8Tm = 3;
    i2c_config.stcSlaveAddr.Gc = FALSE; // set the general call to 0
    I2C_DeInit();                       // deinit I2C
    I2C_Init(&i2c_config);              // init I2C

    // enable NVIC if needed
    if (i2c_config.bTouchNvic)
    {
        EnableNvic(I2C_IRQn, DDL_IRQ_LEVEL_DEFAULT, TRUE);
    }
    I2C_SetFunc(I2cHlm_En);  // disable high speed mode
    I2C_SetFunc(I2cMode_En); // enable I2C
    // I2C_ClearFunc(I2cStart_En); // disable I2C start
    // I2C_ClearFunc(I2cStop_En);  // disable I2C stop
}

en_result_t I2C_MasterReadData(uint8_t *pu8Data,uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i=0,u8State;
    
    I2C_SetFunc(I2cStart_En);
    
	while(1)
	{
		while(0 == I2C_GetIrq())
        {}
		u8State = I2C_GetState();
		switch(u8State)
		{
			case 0x08:
			case 0x10:
				I2C_ClearFunc(I2cStart_En);
				I2C_WriteByte(80|0x01);//从机地址发送OK
				break;
			case 0x40:
				if(u32Len>1)
				{
					I2C_SetFunc(I2cAck_En);
				}
				break;
			case 0x50:
				pu8Data[u8i++] = I2C_ReadByte();
				if(u8i==u32Len-1)
				{
					I2C_ClearFunc(I2cAck_En);
				}
				break;
			case 0x58:
				pu8Data[u8i++] = I2C_ReadByte();
				I2C_SetFunc(I2cStop_En);
				break;	
			case 0x38:
				I2C_SetFunc(I2cStart_En);
				break;
			case 0x48:
				I2C_SetFunc(I2cStop_En);
				I2C_SetFunc(I2cStart_En);
				break;
			default:
				I2C_SetFunc(I2cStart_En);//其他错误状态，重新发送起始条件
				break;
		}
		I2C_ClearIrq();
		if(u8i==u32Len)
		{
			break;
		}
	}
	enRet = Ok;
	return enRet;
}

en_result_t I2C_MasterWriteData(uint8_t *pu8Data, uint32_t u32Len)
{
   en_result_t enRet = Error;
    uint8_t u8i=0,u8State;
    I2C_SetFunc(I2cStart_En);
	while(1)
	{
		while(0 == I2C_GetIrq())
		{;}
		u8State = I2C_GetState();
		switch(u8State)
		{
			case 0x08:
			case 0x10:
				I2C_ClearFunc(I2cStart_En);
				I2C_WriteByte(80);//从设备地址发送
				break;
			case 0x18:
			case 0x28:	
				I2C_WriteByte(pu8Data[u8i++]);
				break;
			case 0x20:
			case 0x38:
				I2C_SetFunc(I2cStart_En);
				break;
			case 0x30:
				I2C_SetFunc(I2cStop_En);
				I2C_SetFunc(I2cStart_En);
				break;
			default:
				break;
		}			
		if(u8i>u32Len)
		{
			I2C_SetFunc(I2cStop_En);//此顺序不能调换，出停止条件
			I2C_ClearIrq();
			break;
		}
		I2C_ClearIrq();	
		delay(10000);
	}
    enRet = Ok;
    return enRet;
}

int main(void)
{

    CLVHD_ConfigPin();

    while (1)
    {
        // use RGB_SetColor( to make rainbow effect)
        uint8_t r, g, b;
        uint8_t inc[3] = {1, -1, 0};
        r = 0;
        g = 255;
        b = 0;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 255; j++)
            {
                RGB_SetColor(r, g, b);
                RGB_SetColor(r, g, b);
                r += inc[i];
                g += inc[(i + 1) % 3];
                b += inc[(i + 2) % 3];
                delay1ms(100);
                I2C_MasterReadData(u8Recdata, 10);
            }
        }
    }
    return 0;
}
