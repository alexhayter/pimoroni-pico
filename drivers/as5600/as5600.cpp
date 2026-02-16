#include <cstdlib>
#include <math.h>
#include <map>
#include <vector>
#include <cstring>

#include "as5600.hpp"

namespace pimoroni {

  bool AS5600::init() {
    if(interrupt != PIN_UNUSED) {
      gpio_set_function(interrupt, GPIO_FUNC_SIO);
      gpio_set_dir(interrupt, GPIO_IN);
      gpio_pull_up(interrupt);
    }
  
    uint8_t aux_id, revision_id,  hardware_id;
    get_version(aux_id, revision_id, hardware_id);

    if(hardware_id != HARDWARE_ID) {
      return false;
    }
  }

  bool reset();

  i2c_inst_t* AS5600::get_i2c() const {
      return i2c->get_i2c();
    }

  int AS5600::get_sda() const {
    return i2c->get_sda();
  }

  int AS5600::get_scl() const {
    return i2c->get_scl();
  }

  int AS5600::get_int() const {
    return interrupt;
  }

  bool AS5600::isConnected() {

    int ret;
    uint8_t rxdata;
        
    ret = i2c_read_blocking(_i2c, _address, &rxdata, 1, false);
 
        return (ret >= 0);
  }

  uint8_t AS5600::getAddress() {
    return _address;
  }

  /////////////////////////////////////////////////////////
  //
  //  CONFIGURATION REGISTERS + direction pin
  //
  void AS5600::setDirection(uint8_t direction){
    _direction = direction;
    if (_directionPin != SW_DIRECTION_PIN){
      gpio_init(_directionPin);
      gpio_set_dir(_directionPin, GPIO_OUT);
      gpio_put(_directionPin, _direction);
    }
  }

  uint8_t AS5600::getDirection(){
    if (_directionPin != SW_DIRECTION_PIN){
      gpio_init(_directionPin);
      gpio_set_dir(_directionPin, GPIO_IN);
      gpio_pull_up(_directionPin);
      if (gpio_get(_directionPin))
        {_direction=CLOCK_WISE;}
      else
        {_direction=COUNTERCLOCK_WISE;}
    }
  return _direction;
  }

  uint8_t AS5600::getZMCO(){
    uint8_t value = readReg(ZMCO);
    return value;
  }

  bool AS5600::setZPosition(uint16_t value){
    if (value > 0x0FFF) return false;
    writeReg2(ZPOS, value);
    return true;
  }


  uint16_t AS5600::getZPosition(){
    uint16_t value = readReg2(ZPOS) & 0x0FFF;
    return value;
  }


  bool AS5600::setMPosition(uint16_t value){
    if (value > 0x0FFF) return false;
    writeReg2(MPOS, value);
    return true;
  }


  uint16_t AS5600::getMPosition(){
    uint16_t value = readReg2(MPOS) & 0x0FFF;
    return value;
  }


  bool AS5600::setMaxAngle(uint16_t value){
    if (value > 0x0FFF) return false;
    writeReg2(MANG, value);
    return true;
  }

  uint16_t AS5600::getMaxAngle(){
    uint16_t value = readReg2(MANG) & 0x0FFF;
    return value;
  }

  /////////////////////////////////////////////////////////
  //
  //  CONFIGURATION
  //

  bool AS5600::setConfigure(uint16_t value){
    if (value > 0x3FFF) return false;
    writeReg2(CONF, value);
    return true;
  }

  uint16_t AS5600::getConfigure(){
    uint16_t value = readReg2(CONF) & 0x3FFF;
    return value;
  }

  //  details configure
  bool AS5600::setPowerMode(uint8_t powerMode){
    if (powerMode > 3) return false;
    uint8_t value = readReg(CONF + 1);
    value &= ~POWER_MODE_MASK;
    value |= powerMode;
    writeReg(CONF + 1, value);
    return true;
  }

  uint8_t AS5600::getPowerMode(){
    return readReg(CONF + 1) & 0x03;
  }

  bool AS5600::setHysteresis(uint8_t hysteresis){
  if (hysteresis > 3) return false;
    uint8_t value = readReg(CONF + 1);
    value &= ~HYST_MASK;
    value |= (hysteresis << 2);
    writeReg(CONF + 1, value);
    return true;
  }

  uint8_t AS5600::getHysteresis(){
    return (readReg(CONF + 1) >> 2) & 0x03;
  }

  bool AS5600::setOutputMode(uint8_t outputMode){
    if (outputMode > 2) return false;
    uint8_t value = readReg(CONF + 1);
    value &= ~OUTS_MASK;
    value |= (outputMode << 4);
    writeReg(CONF + 1, value);
    return true;
  }

  uint8_t AS5600::getOutputMode(){
    return (readReg(CONF + 1) >> 4) & 0x03;
  }

  bool AS5600::setPWMFrequency(uint8_t pwmFreq){
    if (pwmFreq > 3) return false;
    uint8_t value = readReg(CONF + 1);
    value &= ~PMWF_MASK;
    value |= (pwmFreq << 6);
    writeReg(CONF + 1, value);
    return true;
  }

  uint8_t AS5600::getPWMFrequency(){
    return (readReg(CONF + 1) >> 6) & 0x03;
  }

  bool AS5600::setSlowFilter(uint8_t mask){
    if (mask > 3) return false;
    uint8_t value = readReg(CONF);
    value &= ~SF_MASK;
    value |= mask;
    writeReg(CONF, value);
    return true;
  }

  uint8_t AS5600::getSlowFilter(){
    return readReg(CONF) & 0x03;
  }

  bool AS5600::setFastFilter(uint8_t mask){
    if (mask > 7) return false;
    uint8_t value = readReg(CONF);
    value &= ~FTH_MASK;
    value |= (mask << 2);
    writeReg(CONF, value);
    return true;
  }

  uint8_t AS5600::getFastFilter(){
    return (readReg(CONF) >> 2) & 0x07;
  }

  bool AS5600::setWatchDog(uint8_t mask){
    if (mask > 1) return false;
    uint8_t value = readReg(CONF);
    value &= ~WD_MASK;
    value |= (mask << 5);
    writeReg(CONF, value);
    return true;
  }

  uint8_t AS5600::getWatchDog(){
    return (readReg(CONF) >> 5) & 0x01;
  }

    /////////////////////////////////////////////////////////
    //
    //  OUTPUT REGISTERS
    //

  uint16_t AS5600::rawAngle(){
    int16_t value = readReg2(RAW_ANGLE);
    if (_offset > 0) value += _offset;
    value &= 0x0FFF;
    if((_directionPin == SW_DIRECTION_PIN) &&(_direction == COUNTERCLOCK_WISE))
        {value = (4096-value) & 0x0FFF;}
    return value;
   }

  uint16_t AS5600::readAngle(){
    uint16_t value = readReg2(ANGLE);
    if (_error != AS5600_OK)
    {
      return _lastReadAngle;
    }
    if (_offset > 0) value += _offset;
    value &= 0x0FFF;
    if ((_directionPin == SW_DIRECTION_PIN) &&
      (_direction == COUNTERCLOCK_WISE))
    {
    //  mask needed for value == 0.
    value = (4096 - value) & 0x0FFF;
    }
    _lastReadAngle = value;
    return value;
  }

  bool AS5600::setOffset(float degrees){
    //  expect loss of precision.
    if (abs(degrees) > 36000) return false;
    bool neg = (degrees < 0);
    if (neg) degrees = -degrees;
    uint16_t offset = round(degrees * DEGREES_TO_RAW);
    offset &= 0x0FFF;
    if (neg) offset = (4096 - offset) & 0x0FFF;
    _offset = offset;
    return true;
  }

  float AS5600::getOffset(){
    return _offset * RAW_TO_DEGREES;
  }

  bool AS5600::increaseOffset(float degrees){
    //  add offset to existing offset in degrees.
    return setOffset((_offset * RAW_TO_DEGREES) + degrees);
  }

  /////////////////////////////////////////////////////////
  //
  //  STATUS REGISTERS
  //

  uint8_t AS5600::readStatus(){
    uint8_t value = readReg(STATUS);
    return value;
  }

  uint8_t AS5600::readAGC(){
    uint8_t value = readReg(AGC);
    return value;
  }

  uint16_t AS5600::readMagnitude(){
    uint16_t value = readReg2(MAGNITUDE) & 0x0FFF;
    return value;
  }

  bool AS5600::detectMagnet(){
    return (readStatus() & MAGNET_DETECT) > 1;
  }

  bool AS5600::magnetTooStrong(){
    return (readStatus() & MAGNET_HIGH) > 1;
  }

  bool AS5600::magnetTooWeak(){
    return (readStatus() & MAGNET_LOW) > 1;
  }

  float AS5600::getAngularSpeed(uint8_t mode, bool update){
  if (update)
    {
      _lastReadAngle = readAngle();
      if (_error != AS5600_OK)
        {
          return sqrt(-1);
        }
    }
    //  default behaviour
    uint32_t  now     = time_us_32();
    int      angle   = _lastReadAngle;
    uint32_t deltaT  = now - _lastMeasurement;
    int      deltaA  = angle - _lastAngle;

    //  assumption is that there is no more than 180° rotation
    //  between two consecutive measurements.
    //  => at least two measurements per rotation (preferred 4).
    if (deltaA >  2048)      deltaA -= 4096;
    else if (deltaA < -2048) deltaA += 4096;
    float speed = (deltaA * 1e6) / deltaT;

    //  remember last time & angle
    _lastMeasurement = now;
    _lastAngle       = angle;

    //  return radians, RPM or degrees.
  if (mode == MODE_RADIANS)
    {
      return speed * RAW_TO_RADIANS;
    }
  if (mode == MODE_RPM)
    {
      return speed * RAW_TO_RPM;
    }
    //  default return degrees
    return speed * RAW_TO_DEGREES;
  }

    /////////////////////////////////////////////////////////
  //
  //  POSITION cumulative
  //

  int32_t AS5600::getCumulativePosition(bool update){
    if (update)
    {
      _lastReadAngle = readAngle();
      if (_error != AS5600_OK)
        {
          return _position;  //  last known position.
        }
    }
    int16_t value = _lastReadAngle;

    //  whole rotation CW?
    //  less than half a circle
    if ((_lastPosition > 2048) && ( value < (_lastPosition - 2048)))
      {
        _position = _position + 4096 - _lastPosition + value;
      }
      //  whole rotation CCW?
      //  less than half a circle
      else if ((value > 2048) && ( _lastPosition < (value - 2048)))
      {
        _position = _position - 4096 - _lastPosition + value;
      }
    else
      {
        _position = _position - _lastPosition + value;
      }
    _lastPosition = value;
    return _position;
  }

  int32_t AS5600::getRevolutions(){
    int32_t p = _position >> 12;  //  divide by 4096
    if (p < 0) p++;  //  correct negative values, See #65
    return p;
  }

  int32_t AS5600::resetPosition(int32_t position){
    int32_t old = _position;
    _position = position;
    return old;
  }

  int32_t AS5600::resetCumulativePosition(int32_t position){
    _lastPosition = readAngle();
    int32_t old = _position;
    _position = position;
    return old;
  }

  int AS5600::lastError(){
    int value = _error;
    _error = AS5600_OK;
    return value;
  }

  /////////////////////////////////////////////////////////
  //
  //  PROTECTED AS5600
  //

  uint8_t AS5600::readReg(uint8_t reg){
    int _error = AS5600_OK;
    uint8_t data[1];
    // Result byte
    uint8_t n0 = i2c_write_blocking(_i2c, _address, &reg, 1, true);
    uint8_t n1 = i2c_read_blocking(_i2c, _address, data, 1, false);
    uint8_t _data=data[0];
    if (n0 != 1)
      {
        _error = ERROR_I2C_READ_0;
        return 0;
      }
    if (n1 != 1)
      {
        _error = ERROR_I2C_READ_1;
        return 0;
      }
    return _data;
  }

  uint16_t AS5600::readReg2(uint8_t reg){
    int _error = AS5600_OK;
    uint8_t data[2]; // Result byte
    uint8_t n1 = i2c_write_blocking(_i2c, _address, &reg, 1, true);
    uint8_t n2 = i2c_read_blocking(_i2c, _address, data, 2, false);
    uint16_t _data = data[0];
    _data <<= 8;
    _data |= data[1];
    if (n1!=1)
      {
        _error = ERROR_I2C_READ_2;
        return 0;
      }
    if (n2 != 2)
      {
        _error = ERROR_I2C_READ_3;
        return 0;
      }
    return _data;
  }

  uint8_t AS5600::writeReg(uint8_t reg, uint8_t value){
    int _error = AS5600_OK;
    uint8_t buf[] = {reg, value};  // unit8_t buf[0] is register address and buf[1] is unit8_t value
    uint8_t n1 = i2c_write_blocking(_i2c, _address, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
    if (n1 != 1)
      {
        _error = ERROR_I2C_WRITE_0;
      }
    return _error;
  }

  uint8_t AS5600::writeReg2(uint8_t reg, uint16_t value){
    int _error = AS5600_OK;
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = value & 0xff;
    buf[2] = value >> 8;  // unit8_t buf[0] is register address and buf[1] and buf[2] is unit16_t value
    uint8_t n3=i2c_write_blocking(_i2c, _address, buf, 3, false); 
    sleep_ms(10); // Allow stabilization after waking up
    if (n3 != 3)
      {
        _error = ERROR_I2C_WRITE_0;
      }
    return _error;
  }
}