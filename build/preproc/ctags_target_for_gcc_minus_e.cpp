# 1 "/home/aightech/dev/prj/clvhd/hc32l110-clvhd/arduino/SPI_I2c/SPI_I2c.ino"
# 2 "/home/aightech/dev/prj/clvhd/hc32l110-clvhd/arduino/SPI_I2c/SPI_I2c.ino" 2






// Select the module by activating the coreponding address pins.
void selectBrd(uint8_t id)
{
  for (unsigned i = 0; i < 4; i++)
    digitalWrite(m_addPins[i], (id >> i) & 1);
}

// Read n bytes starting from the reg address of the module coreponding to id.
// All values are store in the val buff
void readRegister(byte reg, byte val[], unsigned n = 1, uint8_t id = 15, bool order = true)
{
  byte dataToSend = 0b10000000 | (0b01111111 & reg);

  digitalWrite(PIN_CS, 0);
  SPI.transfer(dataToSend);
  if (order)
    for (unsigned i = 0; i < n; i++)
      *(val + i) = SPI.transfer(0x00);
  else
    for (unsigned i = n - 1; i >= 0; i--)
      *(val + i) = SPI.transfer(0x00);
  digitalWrite(PIN_CS, 1);
}

// Write 1 byte to the reg address of the module coreponding to id.
// All values are store in the val buff
void writeRegister(byte reg, byte val, uint8_t id = 15)
{
  byte dataToSend = 0b00000000 | (0b01111111 & reg);
  ;
  digitalWrite(PIN_CS, 0);
  SPI.transfer(dataToSend);
  SPI.transfer(val);
  digitalWrite(PIN_CS, 1);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  SPI.beginTransaction(SPISettings(20000000, 1, 0x00));
  pinMode(PIN_CS, 1);
  digitalWrite(PIN_CS, 1);

  // setup I2C
  Wire.begin(5); // join i2c bus with address #8
  // I2C speed: 100kHz
  Wire.setClock(100000);

  delay(1000);
  byte val = 0;
  readRegister(0x40, &val, 1);
  Serial.println(val);
}

void loop()
{
  Serial.println("start I2C communication");
  // start I2C communication
  Serial.println("send data");
  Wire.beginTransmission(8);
  Wire.write(0x12);
  Wire.endTransmission();
  // request 1 byte from slave
  Wire.requestFrom(8, 1);
  // read data

  Serial.println("read data");
  while (Wire.available())
  {
    byte val = Wire.read();
    Serial.println(val);
  }

  delay(1000);
}

// Slave code:

// void receiveData()
// {
//     uint8_t data = I2C_ReadByte();
//     data += 1;
//     I2C_WriteByte(data);
// }

//     // setup I2C
//     Gpio_SetFunc_I2C_DAT_P35();
//     Gpio_SetFunc_I2C_CLK_P36();
//     stc_i2c_config_t i2c_config;
//     i2c_config.enFunc = I2cMode_En; // enable I2C
//     i2c_config.u8Tm = 0x0A;// i2c baud rate: 100k
//     i2c_config.stcSlaveAddr.Addr = 0x08; // slave address: 0x08
//     i2c_config.stcSlaveAddr.Gc = 0; // general call disabled (general call address are used for broadcast)
//     i2c_config.pfnI2cCb = receiveData; // callback function
//     I2C_Init(&i2c_config);
