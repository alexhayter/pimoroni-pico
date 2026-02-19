#pragma once

#include <string>

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "common/pimoroni_common.hpp"
#include "common/pimoroni_i2c.hpp"
#include "as5600_regs.hpp"

namespace pimoroni {

  class AS5600 {
    //--------------------------------------------------
    // Constants
    //--------------------------------------------------
  public:
    static const uint8_t DEFAULT_ADDRESS  = 0x36;

    //--------------------------------------------------
    // Enums
    //--------------------------------------------------
  public:
   

    //--------------------------------------------------
    // Substructures
    //--------------------------------------------------
  public:
    


    //--------------------------------------------------
    // Variables
    //--------------------------------------------------
  private:
    I2C *i2c;

    // interface pins with our standard defaults where appropriate
    //int8_t address    = DEFAULT_I2C_ADDRESS;
    uint interrupt    = PIN_UNUSED;

    


    //--------------------------------------------------
    // Constructors/Destructor
    //--------------------------------------------------
  public:
    AS5600(uint interrupt = PIN_UNUSED) : AS5600(new I2C(), interrupt) {};

    AS5600(I2C *i2c, uint interrupt = PIN_UNUSED) : i2c(i2c), interrupt(interrupt) {}


    //--------------------------------------------------
    // Methods
    //--------------------------------------------------
  public:
    bool init();
    bool isConnected();

    // For print access in micropython
    i2c_inst_t* get_i2c() const;
    int get_sda() const;
    int get_scl() const;
    int get_int() const;
    
    void     setDirection(uint8_t direction = CLOCK_WISE);
    uint8_t  getDirection();
   
    uint8_t  getZMCO();

    //  0 .. 4095
    //  returns false if parameter out of range
    bool     setZPosition(uint16_t value);
    uint16_t getZPosition();

    //  0 .. 4095
    //  returns false if parameter out of range
    bool     setMPosition(uint16_t value);
    uint16_t getMPosition();

    //  0 .. 4095
    //  returns false if parameter out of range
    bool     setMaxAngle(uint16_t value);
    uint16_t getMaxAngle();

    // Initialize I2C in class contructor
    void init(int sdaPort, int sclPort);
    //  access the whole configuration register
  //  check datasheet for bit fields
  //  returns false if parameter out of range
  bool     setConfigure(uint16_t value);
  uint16_t getConfigure();

  //  access details of the configuration register
  //  0 = Normal
  //  1,2,3 are low power mode - check datasheet
  //  returns false if parameter out of range
  bool     setPowerMode(uint8_t powerMode);
  uint8_t  getPowerMode();

  //  0 = off    1 = lsb1    2 = lsb2    3 = lsb3
  //  returns false if parameter out of range
  //  suppresses noise when the magnet is not moving.
  bool     setHysteresis(uint8_t hysteresis);
  uint8_t  getHysteresis();

  //  0 = analog 0-100%
  //  1 = analog 10-90%
  //  2 = PWM
  //  returns false if parameter out of range
  bool     setOutputMode(uint8_t outputMode);
  uint8_t  getOutputMode();

  //  0 = 115    1 = 230    2 = 460    3 = 920 (Hz)
  //  returns false if parameter out of range
  bool     setPWMFrequency(uint8_t pwmFreq);
  uint8_t  getPWMFrequency();

  //  0 = 16x    1 = 8x     2 = 4x     3 = 2x
  //  returns false if parameter out of range
  bool     setSlowFilter(uint8_t mask);
  uint8_t  getSlowFilter();

  //  0 = none   1 = LSB6   2 = LSB7   3 = LSB9
  //  4 = LSB18  5 = LSB21  6 = LSB24  7 = LSB10
  //  returns false if parameter out of range
  bool     setFastFilter(uint8_t mask);
  uint8_t  getFastFilter();

  //  0 = OFF
  //  1 = ON   (auto low power mode)
  //  returns false if parameter out of range
  bool     setWatchDog(uint8_t mask);
  uint8_t  getWatchDog();

    
  //  READ OUTPUT REGISTERS
    uint16_t rawAngle();
    uint16_t readAngle();

  //  software based offset.
  //  degrees = -359.99 .. 359.99 (preferred)
  //  returns false if abs(parameter) > 36000
  //          => expect loss of precision
  bool     setOffset(float degrees);       //  sets an absolute offset
  float    getOffset();
  bool     increaseOffset(float degrees);  //  adds to existing offset.


  //  READ STATUS REGISTERS
  uint8_t  readStatus();
  uint8_t  readAGC();
  uint16_t readMagnitude();

  //  access detail status register
  bool     detectMagnet();
  bool     magnetTooStrong();
  bool     magnetTooWeak();


  //  BURN COMMANDS
  //  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
  //  void burnAngle();
  //  void burnSetting();


  //  EXPERIMENTAL 0.1.2 - to be tested.
  //  approximation of the angular speed in rotations per second.
  //  mode == 1: radians /second
  //  mode == 0: degrees /second  (default)
  float    getAngularSpeed(uint8_t mode = MODE_DEGREES,
                           bool update = true);

  //  EXPERIMENTAL CUMULATIVE POSITION
  //  reads sensor and updates cumulative position
  int32_t  getCumulativePosition(bool update = true);
  //  converts last position to whole revolutions.
  int32_t  getRevolutions();
  //  resets position only (not the i)
  //  returns last position but not internal lastPosition.
  int32_t  resetPosition(int32_t position = 0);
  //  resets position and internal lastPosition
  //  returns last position.
  int32_t  resetCumulativePosition(int32_t position = 0);

  //  EXPERIMENTAL 0.5.2
  int      lastError();


    protected:
    //  made virtual, see #66
     virtual uint8_t  readReg(uint8_t reg);
     virtual uint16_t readReg2(uint8_t reg);
     virtual uint8_t  writeReg(uint8_t reg, uint8_t value);
     virtual uint8_t  writeReg2(uint8_t reg, uint16_t value);
  
    uint8_t  _address         = DEFAULT_ADDRESS;
    uint8_t  _directionPin    = 255;
    uint8_t  _direction       = CLOCK_WISE;
    int      _error           = AS5600_OK;

  
    //  for getAngularSpeed()
    uint32_t _lastMeasurement = 0;
    int16_t  _lastAngle       = 0;
    int16_t  _lastReadAngle   = 0;
  
    //  for readAngle() and rawAngle()
    uint16_t _offset          = 0;

    //  EXPERIMENTAL
    //  cumulative position counter
    //  works only if the sensor is read often enough.
    int32_t  _position        = 0;
    int16_t  _lastPosition    = 0;
    
  };

}
