#include "TSC2046.h"

#define REG_SER_TEMP0   (0b000)
#define REG_SER_Y_POS   (0b001)
#define REG_SER_VBAT    (0b010)
#define REG_SER_Z1_POS  (0b011)
#define REG_SER_Z2_POS  (0b100)
#define REG_SER_X_POS   (0b101)
#define REG_SER_AUX     (0b110)
#define REG_SER_TEMP1   (0b111)

#define REG_DFR_Y_POS   (0b001)
#define REG_DFR_Z1_POS  (0b011)
#define REG_DFR_Z2_POS  (0b100)
#define REG_DFR_X_POS   (0b101)

#define REG_POS_Z1_REF_OFF_ADC_ON_DIFF (0b10111001)

/*! @brief The index of the bit of the command word that indicates the start of a control byte.
 * Must always be 1.
 */
#define CMD_START_INDEX   (7)
#define CMD_START_LEN     (1)

#define CMD_ADDR_INDEX    (6)
#define CMD_ADDR_LEN      (3)

#define CMD_MODE_INDEX    (3)
#define CMD_MODE_LEN      (1)

#define CMD_SER_DFR_INDEX (2)
#define CMD_SER_DFR_LEN   (1)

#define CMD_POWER_INDEX   (1)
#define CMD_POWER_LEN     (2)

TSPoint::TSPoint(int16_t x, int16_t y, int16_t z) {
  this->x = x;
  this->y = y;
  this->z = z;
}


Adafruit_TSC2046::Adafruit_TSC2046(int32_t sensorId) {
  _sensorId = sensorId;
}

bool Adafruit_TSC2046::begin(int spiChipSelect, uint16_t rDiffX, SPIClass &spi, uint32_t spiFrequency) {
  _spiCS = spiChipSelect;
  _spi = &spi;
  _spiFrequency = spiFrequency;

  return true;
}

// spi_dev = new Adafruit_SPIDevice(chipSelect, freq, bitorder, spimode, SPIClass);

TSPoint Adafruit_TSC2046::getPoint() {

  //CommandBits readXCmd;
  //readXCmd.addBit(1); // Start bit.
  //readXCmd.addBits(REG_DFR_X_POS, 3); // A2:A0: read X position in DFR mode.
  //readXCmd.addBit(1); // MODE: use 12-bit conversion mode.
  //readXCmd.addBit(0); // SER/DFR': use differential reference mode.
  //readXCmd.addBit(0); // PD1: Internal reference voltage off.
  //readXCmd.addBit(1); // PD0: ADC on.

  //auto spiDev = new Adafruit_SPIDevice(_spiCS, _spiFrequency, SPI_BITORDER_MSBFIRST, SPI_MODE0, _spi);

  //Adafruit_BusIO_Register readXReg = Adafruit_BusIO_Register(
    //spiDev,
    //[> regAddr <] readXCmd.command,
    //[> regType <] ADDRBIT8_HIGH_TOREAD,
    //[> width <] 2, // For 12-bit reads.
    //[> byteorder <] LSBFIRST,
    //[> addressWidth <] 1
  //);

  //Adafruit_BusIO_RegisterBits resultRange = Adafruit_BusIO_RegisterBits(
    //[> reg <] &readXReg,
    //[> bits <] 12,
    //[> shift <] 0
  //);

  //uint16_t xResult = resultRange.read();

  //delete spiDev;

  uint16_t xResult = readDfr(REG_DFR_X_POS);
  uint16_t yResult = readDfr(REG_DFR_Y_POS);

  // FIXME: Calculate pressure.
  return TSPoint(xResult, yResult, 0);
}

uint16_t Adafruit_TSC2046::readDfr(uint8_t channelSelect) {
  CommandBits readCmd;
  readCmd.addBit(1); // START bit, always 1.
  readCmd.addBits(channelSelect, 3); // A2:A0.
  readCmd.addBit(1); // MODE: use 12-bit conversion mode.
  readCmd.addBit(0); // SER/DFR': use differential reference mode.
  readCmd.addBit(0); // PD1: internal reference voltage off.
  readCmd.addBit(1); // PD0: ADC on.

  Adafruit_SPIDevice spiDev = Adafruit_SPIDevice(
    _spiCS, // cspin
    _spiFrequency, // freq
    SPI_BITORDER_MSBFIRST, // dataOrder
    SPI_MODE0, // dataMode FIXME: document why
    _spi // theSPI
  );

  Adafruit_BusIO_Register cmdReg = Adafruit_BusIO_Register(
    &spiDev,
    readCmd.command, // red_addr
    ADDRBIT8_HIGH_TOREAD, // reg_type
    2 // Width: 2, for 12-bit reads.
  );

  Adafruit_BusIO_RegisterBits resultRange = Adafruit_BusIO_RegisterBits(
    &cmdReg, // reg
    12, // bits
    0 // shift
  );

  uint16_t result = resultRange.read();

  return result;
}

CommandBits::CommandBits() {
  command = 0;
}

CommandBits::CommandBits(uint8_t value) {
  command = value;
}

void CommandBits::addBit(bool value) {
  command |= (value << _currentIndex);
  _currentIndex += 1;

  if (_currentIndex > 7) {
    _currentIndex = 0;
  }
}

void CommandBits::addBits(uint8_t value, uint8_t length) {
  // NOTE: If performance is *really* needed and the complier isn't optimizing
  // this function well enough, this for-loop can be changed to:
  // command |= ~(0xFF << length) & value; _currentIndex += length
  // at the expense of losing overflow-wrapping.

  for (int i = 0; i < length; ++i) {
    addBit(value);
  }
}
