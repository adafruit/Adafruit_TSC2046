/*!
 * @file TSC2046.h
 *
 * @mainpage TI TSC2046 Resistive Touchscreen Library.
 *
 * @section intro_sec Introduction
 *
 * This is a library for the TI TSC2046 resistive touchscreen.
 *
 * @section author Author
 *
 * Written by Qyriad <qyriad@qyriad.me>, 2023.
 *
 * @section license license
 *
 * MIT license. All text above must be included in any redistribution.
 *
 */

#ifndef TSC2046_H
#define TSC2046_H

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_SPIDevice.h>
#include <stdint.h>

/*!
 * @brief The type returned by Adafruit_TSC2046::getPoint.
 *
 * See the individual fields and methods for more information.
 */
class TSPoint {
public:
  /*! @brief Create a new TSPoint with these exact values.
   * Not usually needed in user code.
   */
  TSPoint(int16_t x, int16_t y, float z);

  bool operator==(TSPoint rhs);
  bool operator!=(TSPoint rhs);

  /*! @brief The full scale raw X coordinate from the touchscreen.
   * For the X-coordinate as a percentage, see TSPoint::xPercent.
   * If the touchscreen is not being touched, this value is meaningless.
   */
  int16_t x;

  /*! @brief The full scale raw Y coordinate from the touchscreen.
   * For the Y-coordinate as a percentage, see TSPoint::yPercent.
   * If the touchscreen is not being touched, this value is meaningless.
   */
  int16_t y;

  /*! @brief The resistance measurement that corresponds to the pressure
   * currently exerted on the touchscreen. The *higher* the pressure, the
   * *lower* this resistance value will be. Unlike the X and Y coordinates,
   * this value is not in an arbitrary unit of a full scale, but is a physical
   * measurement, in Ohms (Ω).
   */
  float z;

  /*! @brief The X-coordinate as a percentage. Note that physical touchscreens
   * vary, and the range of yours may not perfectly extend from 0% to 100%.
   */
  float xPercent();

  /*! @brief The Y-coordinate as a percentage. Note that physical touchscreens
   * vary, and the range of yours may not perfectly extend from 0% to 100%.
   */
  float yPercent();
};

/*! @brief Class for interfacing with a TSC2046 touchscreen controller.
 *
 * Notable methods: Adafruit_TSC2046::begin, and Adafruit_TSC2046::begin.
 */
class Adafruit_TSC2046 {
public:

  ~Adafruit_TSC2046();

  /*!
   * @brief Initialize this TSC2046 using SPI. You must call this method before
   * calling Adafruit_TSC2046::getPoint.
   *
   * @param spiChipSelect The pin number on your board that you have connected
   * to the SPI CS (Chip Select) pin on the TSC2046.
   *
   * @param xResistance
   * @parblock The resistance in Ohms between X- and X+ on the TSC2046 breakout.
   * With a multimeter set to measure resistance, place one probe on the
   * pin-hole labled "X-" on your TSC2046 breakout board, and place the other
   * probe on the pin-hole labled "X+". Your multimeter should show you a number
   * in Ohms (Ω), the unit for resistance. Pass that number as this parameter.
   * If your multimeter gives you a value in kilohms (kΩ), divide that number
   * by 1000 to get Ohms and pass that value.
   * @endparblock
   *
   * @param vRef
   * @parblock The voltage (in volts) connected to the TSC2046's VRef pin,
   * if any. `-1` (the default if the argument is not provided) indicates that
   * nothing is connected to the TSC2046's VRef pin. Connecting VRef to
   * a voltage higher than 2.5V increases the accurace of **non**-touchscreen
   * reads (temperature, battery voltage, and auxiliary voltage), and has no
   * effect on touchscreen coordinate reads.
   *
   * @note The TSC2046's VRef pin should either be connected to the same supply
   * as the TSC2046's Vin pin, or not connected at all (Vin should be connected
   * to a 5V or 3.3V supply from the Arduino). If you do not connect the VRef
   * pin, pass `-1` to this argument (which is also the default value if you do
   * not pass this argument at all).
   *
   * @endparblock
   *
   * @param spi
   * @parblock The SPI interface to use when communicating to this
   * touchscreen. Defaults to [SPI], the default SPI interface on Arduino
   * boards. This is often connected to pins labeled `SCK`, `MOSI`, and `MISO`
   * on the physical board. For example, on Arduino Uno the MISO of the default
   * `SPI` interface is pin 12.
   * [SPI]: https://docs.arduino.cc/learn/communications/spi
   * @endparblock
   *
   * @param spiFrequency The clock frequency for the SPI peripheral. Defaults
   * to 2 MHz if not specified. Must not be higher than 2 MHz, per the TSC2046
   * datasheet.
   */
  void begin(int spiChipSelect, uint32_t xResistance, float vRef = -1, SPIClass &spi = SPI,
             uint32_t spiFrequency = 2L * 1000L * 1000L);

  /*!
   * @brief Indicates the voltage connected to the TSC2046's "VRef" pin, if any.
   * Use `-1` if nothing is connected to the VRef pin. See the documentation
   * for Adafruit_TSC2046::begin's `vRef` parameter for more information.
   *
   * @param vRef The voltage in volts of the supply connected to the TSC2046's
   * VRef pin. See the documentation for Adafruit_TSC2046::begin's `vRef`
   * parameter for more information.
   */
  void setVRef(float vRef);

  /*!
   * @brief Gets the coordinates of the the current touch on the touchscreen.
   * Use Adafruit_TSC2046::isTouched to determine if the touchscreen is being
   * touched in the first place.
   *
   * @see ::TSPoint.
   */
  TSPoint getPoint();

  /*! @brief Determines if the touchscreen is currently being touched.
   * The X and Y coordinates returned by Adafruit_TSC2046::getPoint are
   * meaningless if this is false.
   *
   * @returns True if the touchscreen is being touched, false if it is not.
   */
  bool isTouched();

  /*! @brief Enables or disables interrupts that fire when the touchscreen
   * is touched. See [here] for information on using Arduino interrupts.
   *
   * @param enable True to enable interrupts, false to disable them.
   *
   * here: https://reference.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
   */
  void enableInterrupts(bool enable);

  /*! @brief Reads the temperature measurement in degrees Celsius.
   *
   * @returns The temperature in degrees Celsius.
   *
   * @see Adafruit_TSC2046::readTemperatureF.
   */
  float readTemperatureC();

  /*! @brief Gets the effective reference voltage, which is 2.5V if no external
   * reference voltage value was provided in Adafruit_TSC2046::begin or
   * Adafruit_TSC2046::setVRef, or the value of the `vRef` argument of those
   * functions otherwise.
   *
   * You probably don't need to call this function unless you're doing math
   * on the touchscreen reads.
   *
   * @returns The effective reference voltage in volts.
   */
  float effectiveVRef();

private:
  SPIClass *_spi;
  Adafruit_SPIDevice *_spiDev;
  int _spiCS;
  int32_t _sensorId;
  int64_t _spiFrequency;
  uint32_t _xResistance;
  bool _interruptsEnabled = false;
  float _vRef;

  float readTemperatureK();

  // Performs a 12-bit differential reference mode read,
  // used for coordinate reads.
  uint16_t readCoord(uint8_t channelSelect);

  // Performs a 12-bit single-ended reference mode read,
  // used for non-coordinate reads.
  uint16_t readExtra(uint8_t channelSelect);

  static uint16_t parse12BitValue(uint8_t spiUpperByte, uint8_t spiLowerByte);
};

/*! @private */
class CommandBits {
public:
  /*! @private */
  CommandBits(uint8_t value = 0);

  /*! @private */
  void addBit(bool value);

  /*! @private */
  void addBits(uint8_t value, uint8_t length);

  /*! @private */
  uint8_t command;
};

#endif
