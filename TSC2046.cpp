#include <math.h>

#include "TSC2046.h"

// Multiplexer addresses for the various outputs in single-ended reference mode.
#define ADDR_SER_TEMP0 (0b000)
#define ADDR_SER_Y_POS (0b001)
#define ADDR_SER_VBAT (0b010)
#define ADDR_SER_Z1_POS (0b011)
#define ADDR_SER_Z2_POS (0b100)
#define ADDR_SER_X_POS (0b101)
#define ADDR_SER_AUX (0b110)
#define ADDR_SER_TEMP1 (0b111)

// Multiplexer addresses for the various outputs in differential reference mode.
#define ADDR_DFR_Y_POS (0b001)
#define ADDR_DFR_Z1_POS (0b011)
#define ADDR_DFR_Z2_POS (0b100)
#define ADDR_DFR_X_POS (0b101)

const float TSC2046_INTERNAL_VREF = 2.5;

TSPoint::TSPoint(int16_t x, int16_t y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

float TSPoint::xPercent() { return x / 4096.f; }

float TSPoint::yPercent() { return y / 4096.f; }

Adafruit_TSC2046::~Adafruit_TSC2046() {
  // In case this object is destroyed before `begin()` is called.
  if (_spiDev) {
    delete _spiDev;
  }
}

void Adafruit_TSC2046::begin(uint32_t xResistance, float vRef = -1,
                             int spiChipSelect, SPIClass &spi,
                             uint32_t spiFrequency) {
  _spiCS = spiChipSelect;
  _spi = &spi;
  _spiFrequency = spiFrequency;
  _xResistance = xResistance;

  // In case `begin()` is called multiple times.
  if (_spiDev) {
    delete _spiDev;
  }

  setVRef(vRef);

  // Regarding SPI mode, timing diagrams on the datasheet show DCLK idling LOW,
  // which means the leading edge is a rising edge, which means CPOL = 0.
  // For DOUT (MISO/CIPO) the datasheet says "data are shifted on the falling
  // edge of DCLK", and for DIN it says "data are latched on the rising edge
  // of DCLK", which means OUT side changes on the trailing edge of the clock
  // and the IN side changes on/after the leading edge of the clock, which
  // means CPHA = 0.
  // Therefore, our SPI mode is 0.
  _spiDev = new Adafruit_SPIDevice(_spiCS,                // cspin
                                   _spiFrequency,         // freq
                                   SPI_BITORDER_MSBFIRST, // dataOrder
                                   SPI_MODE0,             // dataMode
                                   _spi);
  _spiDev->begin();
}

void Adafruit_TSC2046::setVRef(float vRef) { _vRef = vRef; }

void Adafruit_TSC2046::setTouchedThreshold(float rTouchThreshold) {
  _touchedThreshold = rTouchThreshold;
}

TSPoint Adafruit_TSC2046::getPoint() {

  // If interrupts are enabled on the TSC2046 side, temporarily disable them
  // on the procesor side so we don't get interrupted while reading the various
  // coordinates.
  if (_interruptsEnabled) {
    noInterrupts();
  }

  int16_t xResult = readCoord(ADDR_DFR_X_POS);
  int16_t yResult = readCoord(ADDR_DFR_Y_POS);
  int16_t z1Result = readCoord(ADDR_DFR_Z1_POS);
  int16_t z2Result = readCoord(ADDR_DFR_Z2_POS);

  // Now that we're done with the sensitive stuff, re-enable interrupts.
  if (_interruptsEnabled) {
    interrupts();
  }

  // The datasheet gives two ways to calculate pressure. We're going to use the
  // one that requires the least information from the user:
  //
  // R_TOUCH = R_X_PLATE * (X_POSITON / 4096) * (Z_2 / Z_1 - 1)
  // So this requires knowing the X-Plate resistance, which thankfully we got
  // from the user back at Adafruit_TSC2046::begin().

  float rTouch = _xResistance * (xResult / 4096.f) *
                 (((float)z2Result / (float)z1Result) - 1.f);

  return TSPoint(xResult, yResult, rTouch);
}

bool Adafruit_TSC2046::isTouched() {
  TSPoint point = getPoint();

  // If the resistance is not infinity, NaN, some other non-finite number,
  // or 0, then the touchscreen is probably being touched.
  return isfinite(point.z) && point.z != 0 && point.z < _touchedThreshold;
}

void Adafruit_TSC2046::enableInterrupts(bool enable) {

  // NOTE: On my hardware, PENIRQ' goes low when the touchscreen is touched
  // *regardless* of the PD0 value that enables and disables interrupts.

  _interruptsEnabled = enable;

  // Perform any read so we can get the control byte over there with the new
  // PD values which enables or disables the PENIRQ' output.
  readCoord(0);
}

float Adafruit_TSC2046::readTemperatureC() { return readTemperatureK() - 273; }

float Adafruit_TSC2046::readTemperatureF() {
  float celsius = readTemperatureC();
  return (9.f / 5.f) * celsius + 32;
}

float Adafruit_TSC2046::readBatteryVoltage() {

  // According to the datasheet, the battery voltage readings are divided down
  // by 4 to simplify the logic in the chip, which means we have to multiple
  // it back up again.
  const float VBAT_MULTIPLIER = 4.f;

  uint16_t rawVBat = readExtra(ADDR_SER_VBAT);

  // V_BAT = ADC_VALUE * 4 * effectiveVRef() / (2 ** ADC_SIZE)
  return (rawVBat * VBAT_MULTIPLIER * effectiveVRef()) / 4096.f;
}

float Adafruit_TSC2046::effectiveVRef() {
  if (_vRef == -1) {
    return TSC2046_INTERNAL_VREF;
  } else {
    return _vRef;
  }
}

float Adafruit_TSC2046::readAuxiliaryVoltage() {

  uint16_t rawVAux = readExtra(ADDR_SER_AUX);

  // V_AUX = (ADC_VALUE * effectiveVRef()) / (2 ** ADC_SIZE)
  return (rawVAux * effectiveVRef()) / 4096.f;
}

float Adafruit_TSC2046::readTemperatureK() {

  // There are two ways to measure temperature on this chip.
  // The first is to Already Know what the ADC value of TEMP0 is at
  // 25°C for this particular chip, and then take a reading.
  // We don't want to make the user do more measurements than they have to,
  // so thankfully the second method eschews that limitation, in favor of
  // needing to measure two different values.
  // It then gives us a formula to calculate the temperature in Kelvin
  // given the difference of two voltages.
  //
  // The formula is:
  // T = (ELEMENTARY_CHARGE * ΔV) / (BOLTZMANN_CONST * ln(91)).
  // Now, the elementary charge constant and Boltzmann's constant are both
  // *incredibly* tiny -- far too tiny for us to do math with on a tiny
  // microcontroller (Boltzmann's constant is 1.3807e-23).
  //
  // Thankfully, that formula simplifies to:
  // T = 2572.52 K/V (kelvins per volt), or
  // T = 2.57257 K/mV (kelvins per millivolt)

  uint16_t temp0 = readExtra(ADDR_SER_TEMP0);
  uint16_t temp1 = readExtra(ADDR_SER_TEMP1);

  // temp0 and temp1 are given as a ratio of the reference voltage and
  // the full ADC scale.
  // In other words, the V_temp0 = (temp0 * V_REF) / (2 ** ADC_SIZE)
  // Which in our case means V_temp0 = (temp0 * effectiveVRef()) / 4096
  // We want the change in voltage across those two readings,
  // and in millivolts, so:
  float deltaMilliVolts = (((temp1 - temp0) * effectiveVRef()) / 4096.f) * 1000;

  // So now let's apply that simplified formula:
  float temperatureKelvin = deltaMilliVolts * 2.573f;

  return temperatureKelvin;
}

uint16_t Adafruit_TSC2046::readCoord(uint8_t channelSelect) {

  CommandBits controlCmd;

  // START bit, always 1.
  controlCmd.addBit(1);

  // A2:A0: the channel select/"address" bits, which control the multiplexer
  // output. This will be one of the `ADDR_` values near the top of the file.
  controlCmd.addBits(channelSelect, 3);

  // ADC conversion mode: LOW for 12-bit mode, and HIGH for 8-bit mode.
  controlCmd.addBit(0);

  // SER/DFR': use the internal or external VREF (HIGH), or use the voltage
  // across the touchscreen drivers as the ADC reference voltage (LOW).
  // The latter is more accurate, but is only available for touchscreen
  // coordinate reads, and not available for temperature, VBAT, or the other
  // extras. In this case, however, we pull it LOW for the increased
  // accuracy.
  controlCmd.addBit(0);

  // NOTE(Qyriad): The datasheet says that PD0 = 1 disables interrupts, however
  // in my testing PENIRQ' goes low when the touchscreen is touched even if
  // PD0 = 1, and the only case where PENIRQ' does not respond to touches is
  // when *both* PD0 and PD1 are HIGH.
  if (_interruptsEnabled) {
    // PD1: Enable/disable' internal VREF. We're using differential reference
    // mode, so VREF (internal or external) doesn't matter at all, so let's
    // just leave it off.
    controlCmd.addBit(0);

    // PD0: ADC on/off', sort of. When both PD1 and PD0 are LOW, then it leaves
    // the ADC off *between* conversions, but powers it on *during*
    // conversions. According to the datasheet the ADC is able to power up
    // instantly and there is no delay from leaving the ADC powered off
    // between conversions. Leaving the ADC on is intended for certain
    // strategies that use external capacitors to filter out touchscreen noise.
    // This bit also, with PD1, controls whether or not interrupts are enabled.
    // Interrupts are *only* disabled when *both* PD1 and PD0 are HIGH.
    // In this if block, interrupts are enabled, so we can leave this LOW
    // and have the ADC power down between conversions to save power.
    controlCmd.addBit(0);
  } else {
    // PD1: Enable/disable' internal VREF. We're using differential reference
    // mode, so VREF (internal or external) doesn't matter at all. However,
    // this value *does* affect whether or not interrupts or enabled.
    // If we explicitly want to disable interrupts, we have to set this bit
    // HIGH, which consumes more power.
    controlCmd.addBit(1);

    // PD0: ADC on/off'. We want the ADC on, and also this value has to be
    // HIGH to disable interrupts.
    controlCmd.addBit(1);
  }

  Adafruit_BusIO_Register controlReg = Adafruit_BusIO_Register(
      _spiDev,
      controlCmd.command,   // reg_addr
      ADDRBIT8_HIGH_TOREAD, // reg_type
      2,                    // Width: 2, to get the 12-bits we need.
      // It's a 12-bit value with the most-significant BIT first.
      // Therefore, the first 1-byte read will read the 8 most-significant bits,
      // and the next 1-byte read will read the 4 least-significant bits.
      MSBFIRST,
      1 // address_width
  );

  uint8_t spiOutputBuffer[2];
  controlReg.read(spiOutputBuffer, 2);

  // Will contain bits 11:5. See parse12BitValue for why.
  uint8_t upperByte = spiOutputBuffer[0];
  // Will contain bits 4:0. See parse12BitValue for why.
  uint8_t lowerByte = spiOutputBuffer[1];

  uint16_t result = parse12BitValue(upperByte, lowerByte);

  return result;
}

uint16_t Adafruit_TSC2046::readExtra(uint8_t channelSelect) {
  CommandBits controlCmd;

  // START bit, always 1.
  controlCmd.addBit(1);

  // A2:A0: the channel select/"address" bits, which control the multiplexer
  // output. This will be one of the `ADDR` values near the top of the file.
  controlCmd.addBits(channelSelect, 3);

  // ADC conversion mode: LOW for 12-bit mode, and HIGH for 8-bit mode.
  controlCmd.addBit(0);

  // SER/DFR': use the internal or external VREF (HIGH), or use the voltage
  // across the touchscreen drivers as the ADC reference voltage (LOW).
  // The latter is more accurate, but is only available for touchscreen
  // coordinate reads, and not available for temperature, VBAT, or the other
  // extras. So in this case we keep this HIGH so we can read things like
  // VBAT.
  controlCmd.addBit(1);

  // PD1: Enable/disable' internal VREF.
  if (_vRef != -1) {
    // The user connected an external VREF, so turn the internal one off.
    controlCmd.addBit(0);
  } else {
    // The user did not connect an external VREF, so turn the internal one on.
    controlCmd.addBit(1);
  }

  // PD0: This bit is ADC on/off', technically, but when both PD1 and PD0 are
  // 0, then it leaves the ADC off *between* conversions, but powers it on
  // *during* conversions. According to the datasheet the ADC is able to power
  // up instantly and there are no delays incured by leaving the ADC powered
  // off between conversions. Leaving the ADC on is intended for certain
  // strategies that use external capacitors to filter out touchscreen noise.
  // This doesn't apply to us, but there is one more consideration, which is
  // that the PENIRQ' output used to trigger interrupts is disabled if
  // this bit is HIGH (1). Since that's the only functionality of this bit
  // we care about, we'll make it correspond directly to the user's
  // IRQ setting.
  controlCmd.addBit(!_interruptsEnabled);

  Adafruit_BusIO_Register controlReg = Adafruit_BusIO_Register(
      _spiDev,
      controlCmd.command,   // reg_addr
      ADDRBIT8_HIGH_TOREAD, // reg_type
      2,                    // Width: 2, to get the 12-bits we need.
      // It's a 12-bit value with the most-significant BIT first.
      // Therefore, the first 1-byte read will read the 8 most-significant bits,
      // and the next 1-byte read will read the 4 least-significant bits.
      MSBFIRST,
      1 // address_width
  );

  uint8_t spiOutputBuffer[2];
  controlReg.read(spiOutputBuffer, 2);

  // Will contain bits 11:5. See parse12BitValue for why.
  uint8_t upperByte = spiOutputBuffer[0];
  // Will contain bits 4:0. See parse12BitValue for why.
  uint8_t lowerByte = spiOutputBuffer[1];

  uint16_t result = parse12BitValue(upperByte, lowerByte);

  return result;
}

static uint16_t Adafruit_TSC2046::parse12BitValue(uint8_t spiUpperByte,
                                                  uint8_t spiLowerByte) {

  // `spiUpperByte` will contain bits 11:5.
  // `spiLowerByte` will contain bits 4:0.
  // See below for why.

  // clang-format off
  /*
                                 Upper Byte                                         Lower Byte
                 -------------------------------------------------  -------------------------------------------------
    SPI Read:    |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |

    12-bit Map:  |  X  |  11 |  10 |  9  |  8  |  7  |  6  |  5  |  |  4  |  3  |  2  |  1  |  0  |  X  |  X  |  X  |

    Want:        |  X  |  X  |  X  |  12 |  11 |  10 |  9  |  8  |  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |

    Xs are "don't care" values that we don't want in our result, so we should mask them out.

--
    "12-bit Map" are the where each bit of the 12-bit number we actually want are placed *in* the data we read
    over SPI. So the bit 11 (0-indexed) of the 12-bit number is found in bit 6 of `spiUpperByte`.
    Why is bit 7 of the Upper Byte an X? Because there is *one* extra clock cycle after we finish sending the
    control byte before the TSC2046 starts sending out the converted data. BusIO captures *immediately* after
    the SPI command is finished, though, so the very first bit (the most-significant bit, in BusIO's "eyes")
    is not the most-significant bit of the 12-bit value, but instead an "empty" value that
    1) we need to mask out, and 2) pads the start of the 12-bit value by one bit index.

    "Want" is what we need to bit-shift and mask to get, with the numbers being the indexes of the overall 12-bit
    value.
   */
  // clang-format on

  // Mask out bit 7 of `spiUpperByte`, which is an "X".
  spiUpperByte &= 0x7F; // 0b0111_1111;

  // Mask out the least significant 3 bits (bits 2, 1, and 0) of `spiLowerByte`,
  // which are also "X"s.
  spiLowerByte &= 0xF8; // 0b1111_1000;

  // Bit 6 of `spiUpperByte` needs to become bit 11 of the uint16_t,
  // so we shift `spiUpperByte` LEFT by 5 (11 - 6 = 5).
  // Meanwhile, bit 3 of `spiLowerByte` needs to become bit 0 of the uint16_t,
  // so we shift that RIGHT by 3 (3 - 0 = 3).
  uint16_t finalResult = (spiUpperByte << 5) | (spiLowerByte >> 3);

  // Finally, we have the 12-bit number we read as a uint16_t.
  return finalResult;
}

CommandBits::CommandBits(uint8_t value) { command = value; }

void CommandBits::addBit(bool value) {
  uint8_t bit = value & 1;
  command = (command << 1) | bit;
}

void CommandBits::addBits(uint8_t value, uint8_t length) {
  uint8_t mask = (1 << length) - 1;
  uint8_t bits = value & mask;

  command = (command << length) | bits;
}
