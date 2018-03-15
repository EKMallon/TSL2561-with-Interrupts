//TSL2561
//DATASHEET https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/TSL2561.pdf

/* Important: TSL2561_COMMAND_BIT gets added (with | )to the register address when you are about to write data to the sensor
  A byte sent to the TSL256x with the most significant bit (MSB) equal to 1 will be interpreted as a COMMAND byte. The lower four bits of the COMMAND
  byte form the register select address (see Table 2), which is used to select the destination for the subsequent byte(s) received.
  x &= y bitwise logical AND -if both input bits are 1 output is 1, otherwise the output is 0
  x |= y bitwise logical OR -if either or both of the input bits is 1, otherwise it is 0
*/

#include <Wire.h>

//The TSL256x offers three slave addresses depending on(ADDR SEL)pin
// use a bus scanner to check which address your board is at
#define TSL2561_Address  0b00111001 // 0x39 // default (ADDR SEL)pin unconnected
//#define TSL2561_Address  0b00101001 // 0x29 // (ADDR SEL)pin to GND
//#define TSL2561_Address  0b01001001 // 0x49 // (ADDR SEL)pin to Vcc

#define TSL2561_COMMAND_BIT    0x80    // Must be 1
#define TSL2561_CLEAR_BIT      0x40    // Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT       0x20    // 1 = read/write word (rather than byte)

#define TSL2561_REG_CONTROL   0x00
#define TSL2561_REG_TIMING    0x01
#define TSL2561_REG_THRESH_L  0x02
#define TSL2561_REG_THRESH_H  0x04
#define TSL2561_REG_INTCTL    0x06
#define TSL2561_REG_ID        0x0A
#define TSL2561_REG_DATA_0    0x0C  // 1100
#define TSL2561_REG_DATA_1    0x0E  // 1110

// Global variables:
byte bytebuffer1 = 0;     // for functions that return a byte
boolean gain;             // Gain setting, 0 = X1, 1 = X16;
unsigned int IntegrationTime_ms;  // Integration ("shutter") time in milliseconds

void setup()
{

  //=================configure the hardware interrupt pins as inputs  ====================
  pinMode(2, INPUT_PULLUP); //configure the hardware interrupt pins as inputs
  pinMode(3, INPUT_PULLUP); //and pull UP with the internal pullup resistor
  // this assumes the sensor triggers alarms by pulling the line "LOW"
  //================================================================

  Serial.begin(9600);
  Wire.begin();  // I2C (TSL2561) supports Fast-Mode at 400 kHz

  i2c_writeRegisterByte(TSL2561_Address, (TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL), 0b10000000); //this shuts off the sensor
  Serial.println(F("TSL2561 Powered Down: Initializing registers"));

  byte ID; // Get factory ID from sensor:
  ID = i2c_readRegisterByte(TSL2561_Address, TSL2561_REG_ID);
  Serial.print(F("TSL2561 Factory ID: 0X"));
  Serial.print(ID, HEX);
  Serial.println(F(",(should be 0X5X)"));

//configure timing register: Uncomment only ONE of the bytebuffer modificiations below
//------------------------------------------------------------------------------------

  //bits 1 & 0 of the timing register control sensor integration time:
  bytebuffer1 = i2c_readRegisterByte(TSL2561_Address, TSL2561_REG_TIMING);

  // set last two bits to 00 = integration will be 13.7ms
  // is this its integration time is too slow for really brief flashes of light??
  bytebuffer1 &= ~(0b00000011); IntegrationTime_ms = 14; /* fast but low resolution */

  // set last two bits to 01 = integration will be 101ms  /* medium resolution and speed */
  //bytebuffer1 &=~(0b00000010);bytebuffer1 |= (0b00000001); IntegrationTime_ms = 101;

  // set last two bits to = integration will be 402ms (the default)  /* 16-bit data but slowest conversion */
  //bytebuffer1 |=(0b00000010);bytebuffer1 &=~(0b00000001); IntegrationTime_ms = 402;

  // only use this for special one-shot readings
  // set last two bits of the timing register to 11 = integration will be controlled manually
  //   bytebuffer1 |=(0b00000011); IntegrationTime_ms = 0; // In this mode, sensor sleeps until you begin each reading
  // You can also perform your own manual integration timing
  // by setting last two bits of the timing register to 11
  // then performing a manual Start and a manual Stop:
  // IntegrationTime_ms = what ever you want it to be
  // manual Start: i2c_writeRegisterByte(TSL2561_Address,(TSL2561_COMMAND_BIT |TSL2561_REG_CONTROL),0b00000011);Serial.println(F("TSL2561 Power On"));
  // delay(IntegrationTime_ms);
  // manual Stop:  i2c_writeRegisterByte(TSL2561_Address,(TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL),0b10000000); //this shuts off the sensor

  i2c_writeRegisterByte(TSL2561_Address, (TSL2561_COMMAND_BIT | TSL2561_REG_TIMING), bytebuffer1);


  Serial.print(F("Timing set to:")); Serial.print(IntegrationTime_ms); Serial.println(F("milliseconds"));

//Set GAIN (bit 4) of timing register: Uncomment only one of the two gain settings
//---------------------------------------------------------------------------------
  bytebuffer1 = i2c_readRegisterByte(TSL2561_Address, TSL2561_REG_TIMING);  //load the existing register contents

  bytebuffer1 |= (1 << 4); gain = 1; Serial.println(F("Gain set to high gain (16X)")); // gain bit4 now = 1

  //---------- OR ---------

  //bytebuffer1 &= ~(1 << 4);gain=0;Serial.println(F("Gain set low gain (1X)"));// gain bit4 = 0(default), low gain (1X)

  i2c_writeRegisterByte(TSL2561_Address, (TSL2561_COMMAND_BIT | TSL2561_REG_TIMING), bytebuffer1);


//Interrupt Threshold Registers (2h − 5h)  [defaults to 00h on power up]
//--------------------------------------------------------------------
  //TSL2561 supports only interrupts generated by thresholds on ch0 (visible light) not ch1 (IR)
  /*The interrupt threshold registers store the values to be used as the high and low trigger points
    for the comparison function for interrupt generation. If the value generated by channel 0
    crosses below or is equal to the low threshold specified, an interrupt is asserted on the interrupt pin.
    If the value generated by channel 0 crosses above the high threshold specified,
    an interrupt is asserted on the interrupt pin. */

  /* Re: Configure interrupt thresholds with LUX values*/
  /* Note: to convert a standard SI lux value to a raw sensor values (for channel 0),
    taking into account the configured gain for the sensor. Required for SI lux consistency, and needed for interrupt threshold configuration.
    In order to get an estimation of the raw value to pass here when you want a lux value as a threshold
    since that lux calculation requires input from both channel 0 AND channel 1 you need to use a conversion
    function with the right approximation parameter between the two Ch0 & Ch1 channels
    The ratio is somewhere around 0.31 - 0.32 for conditions under sunlight,
    Under artificial light (e.g. LED light), the IR component is much smaller and leans more towards ratio 0.11
    // for an example with those calculations see the ADAfruit lib derivatives: https://github.com/jacobstim/Adafruit_TSL2561
  */

  // values supplied as thresholds are raw sensor values and NOT values in the SI lux unit.
  // If you increase the gain, you must make a corresponding 16* change to the threshold values
  // -> this makes it almost impossible to use autogain together with interrupts!
  // AND changing the integration timing register ALSO affects the raw output so may reqire changing the threshold values as well

  // Generally, I just manually adjust the numbers until they suit the range for my application:
  // One drawback of this sensor, is that it always int-alarms in pure darkness...
  // because you can't disable the lowThreshold
  int lowThreshold = 0; Serial.print("Low threshold:"); Serial.print(lowThreshold);
  int highThreshold = 2000; Serial.print("   High threshold:"); Serial.println(highThreshold);

  // Write low threshold value  - The Write Word protocol should be used to write byte-paired registers
  // low byte of each 16-bit uint value first, then high byte in the following register
  Wire.beginTransmission(TSL2561_Address);
  Wire.write(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REG_THRESH_L);  //register pointer
  Wire.write(lowByte(lowThreshold));   // sends LSB  //same as  (x) & (0xff)
  Wire.write(highByte(lowThreshold));  // sends MSB  //same as (x) >> (8)
  Wire.endTransmission();
  // Write high threshold value
  Wire.beginTransmission(TSL2561_Address);
  Wire.write(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REG_THRESH_H);  //register pointer
  Wire.write(lowByte(highThreshold));   // sends LSB  //same as  (x) & (0xff)
  Wire.write(highByte(highThreshold));  // sends MSB  //same as (x) >> (8)
  Wire.endTransmission();

//Interrupt Control Register (6h)
//-------------------------------
  // bits 7&6  = 00 always  //reserved

  //INTR (bits 5 & 4 of the Interrupt Control Register)
  //00 Interrupt output disabled  (ie this is how to turn off the interrupt)
  //01 Level Interrupt enabled (ie this is how to turn on the interrupt)
  // the interrupt output is active-low and requires a pull-up resistor

  // PERSIST TimeWindow(bits 3-0 of the ICR register) -probably start with this set to 0b0001?
  // The interrupt persist bit field (PERSIST) provides control over when interrupts occur.
  // If persist bits = 0000, every integration cycle generates an interrupt (a data ready alarm)
  // If persist bits = 0001, any value outside of threshold generates an interrupt
  // If persist = 2 to 15, value must be outside of threshold for 2 to 15 integration cycles (depends on speed)

  i2c_writeRegisterByte(TSL2561_Address, (TSL2561_REG_INTCTL | TSL2561_COMMAND_BIT), 0b00010001); //level int enabled// persist to 0001

  //Clear any pending interrupt (write 1 to clear bit 6)
  //A byte sent to the TSL256x with the most significant bit (MSB) equal to 1 will be interpreted as a COMMAND byte
  Wire.beginTransmission(TSL2561_Address);
  Wire.write(TSL2561_COMMAND_BIT | TSL2561_CLEAR_BIT); //11000000
  Wire.endTransmission();

// power ON / Off
//----------------
  if (IntegrationTime_ms > 0) { // power up the sensor IF in normal mode
    // Write 0x03 to command byte (power on)
    i2c_writeRegisterByte(TSL2561_Address, (TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL), 0b00000011); Serial.println(F("TSL2561 Power On"));
  }
  else
  { i2c_writeRegisterByte(TSL2561_Address, (TSL2561_COMMAND_BIT | TSL2561_REG_CONTROL), 0b00000000); Serial.println(F("TSL2561 Power Down: waiting for manual Start..."));
    //Power down the sensor NOTE: This disables interrupts
  }

} //============ end of setup=================

void loop()
{

  // Wait between measurements before retrieving the result
  // (OR You can also configure the sensor to issue an interrupt
  // when measurements are complete with persist bits = 0000)

  // Once integration is complete, retrieve the data.
  // There are two light sensors on the device, one for visible light
  // and one for infrared. Both sensors are have to be read for lux calculations.

  // much of what follows is based on
  // the sparkfun example at https://github.com/sparkfun/TSL2561_Luminosity_Sensor_BOB/blob/master/Libraries/Arduino/src/SparkFunTSL2561.cpp

  // Retrieve the data from the sensor:
  unsigned int data0, data1; // raw Values will be set to stored unsigned integer (16 bits)
  char high, low;

  // Set up command byte for read // ADC channel 0 (visible light) lower byte first
  Wire.beginTransmission(TSL2561_Address);
  Wire.write((TSL2561_COMMAND_BIT |  TSL2561_REG_DATA_0)); //10001100  Bit7 must be a 1 for the command
  Wire.endTransmission();
  Wire.requestFrom(TSL2561_Address, 2);
  low = Wire.read();
  high = Wire.read();
  data0 = word(high, low); // Combine bytes into unsigned int

  // Set up command byte for read // ADC channel 1 lower byte first (IR)
  Wire.beginTransmission(TSL2561_Address);
  Wire.write((TSL2561_COMMAND_BIT |  TSL2561_REG_DATA_1)); //10001110
  Wire.endTransmission();
  Wire.requestFrom(TSL2561_Address, 2);
  low = Wire.read();
  high = Wire.read();
  data1 = word(high, low); // Combine bytes into unsigned int

  Serial.print(F("RAW data0: ")); Serial.print(data0); Serial.print(F(" data1: ")); Serial.println(data1);
  if ((data0 == 0xFFFF) || (data1 == 0xFFFF))
  {
    Serial.println(F("Sensor is SATURATED!")); // later calculations will not be accurate if this happens!
  }

  // Convert raw data to lux
  double lux;    // Resulting lux value
  // Convert from unsigned integers to floating point
  float ratio, d0, d1; d0 = data0; d1 = data1;
  // We need the ratio for subsequent calculations
  ratio = d1 / d0;
  // Normalize for integration time
  d0 *= (402.0 / IntegrationTime_ms); d1 *= (402.0 / IntegrationTime_ms);
  // Normalize for gain
  if (!gain) {
    d0 *= 16;
    d1 *= 16;
  }
  // Determine lux per datasheet equations:
  if (ratio < 0.5) {
    lux = 0.0304 * d0 - 0.062 * d0 * pow(ratio, 1.4);
  }
  if (ratio < 0.61) {
    lux = 0.0224 * d0 - 0.031 * d1;
  }
  if (ratio < 0.80) {
    lux = 0.0128 * d0 - 0.0153 * d1;
  }
  if (ratio < 1.30) {
    lux = 0.00146 * d0 - 0.00112 * d1;
  }
  if (ratio > 1.30) {
    lux = 0.0;
  }
  // Print out the results:
  Serial.print(F(" lux: ")); Serial.print(lux);

  bool alarm = digitalRead(3);
  if (alarm) {
    Serial.println(F(" NO interrupt..."));
  }
  else {
    Serial.println(F(" Interrupt triggered... & Cleared"));
    //Clear the interrupt (write 1 to clear bit 6 of TSL2561_COMMAND)
    Wire.beginTransmission(TSL2561_Address);
    Wire.write(TSL2561_COMMAND_BIT | TSL2561_CLEAR_BIT); //11000000
    Wire.endTransmission();
  }
  delay(3000);
}//=================end of main loop=====================


// ==========================================================================================
// I2C SENSOR MEMORY REGISTER FUNCTIONS
//===========================================================================================

byte i2c_readRegisterByte(uint8_t deviceAddress, uint8_t registerAddress)
{
  byte registerData;
  Wire.beginTransmission(deviceAddress); //set destination target
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  registerData = Wire.read();
  return registerData;
}

byte i2c_writeRegisterByte(uint8_t deviceAddress, uint8_t registerAddress, byte newRegisterByte)
{
  byte result;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(newRegisterByte);
  result = Wire.endTransmission();

  if (result > 0)   //error checking
  {
#ifdef ECHO_TO_SERIAL   //NOTE: only call halt on error if in debug mode!
    Serial.print(F("FAIL in I2C register write! Result code: "));
    Serial.println(result);
    error();
#endif
  }
  // NOTE: some sensors need settling time after a register change but MOST do not
  return result;
}


