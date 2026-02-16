#pragma once
#include <stdint.h>

namespace pimoroni {
  /***** Device registers and masks here *****/

  enum reg {

    //  CONFIGURE CONSTANTS
//  check datasheet for details

//  CONFIGURATION REGISTERS
ZMCO = 0x00,
ZPOS = 0x01,   //  + 0x02
MPOS = 0x03,  //  + 0x04
MANG = 0x05,   //  + 0x06
CONF = 0x07,   //  + 0x08

//  OUTPUT REGISTERS
RAW_ANGLE = 0x0C,   //  + 0x0D
ANGLE     = 0x0E,   //  + 0x0F

//  STATUS REGISTERS
STATUS    = 0x0B,
AGC       = 0x1A,
MAGNITUDE = 0x1B,   //  + 0x1C

// BURN COMMANDS
BURN      = 0xFF,

  };

#define AS5600_LIB_VERSION              (("0.6.4"))

const uint8_t DEFAULT_ADDRESS    = 0x36;

const uint8_t SW_DIRECTION_PIN   = 255;

//  setDirection
const uint8_t CLOCK_WISE         = 0;  //  LOW
const uint8_t COUNTERCLOCK_WISE  = 1;  //  HIGH

//  0.087890625;
const float   RAW_TO_DEGREES     = 360.0 / 4096;
const float   DEGREES_TO_RAW     = 4096 / 360.0;
//  0.00153398078788564122971808758949;
const float   RAW_TO_RADIANS     = 3.14159 * 2.0 / 4096;
//  4.06901041666666e-6
const float   RAW_TO_RPM         = 60.0 / 4096;

//  getAngularSpeed
const uint8_t MODE_DEGREES       = 0;
const uint8_t MODE_RADIANS       = 1;
const uint8_t MODE_RPM           = 2;

//  ERROR CODES
const int     AS5600_OK          = 0;
const int     ERROR_I2C_READ_0   = -100;
const int     ERROR_I2C_READ_1   = -101;
const int     ERROR_I2C_READ_2   = -102;
const int     ERROR_I2C_READ_3   = -103;
const int     ERROR_I2C_WRITE_0  = -200;
const int     ERROR_I2C_WRITE_1  = -201;


//  CONF+1 OutputMode masks & values 
const uint8_t OUTS_MASK					= 0b00110000;
const uint8_t OUTMODE_ANALOG_100 = 0;
const uint8_t OUTMODE_ANALOG_90  = 1;
const uint8_t OUTMODE_PWM        = 2;

//  CONF+1 PowerMode masks & values CONF+1
const uint8_t POWER_MODE_MASK		= 0b00000011;
const uint8_t POWERMODE_NOMINAL  	= 0;
const uint8_t POWERMODE_LOW1     	= 1;
const uint8_t POWERMODE_LOW2     	= 2;
const uint8_t POWERMODE_LOW3     	= 3;

//  CONF+1 PWMFrequency masks & values CONF+1
const uint8_t PMWF_MASK					= 0b11000000;
const uint8_t PWM_115            = 0;
const uint8_t PWM_230            = 1;
const uint8_t PWM_460            = 2;
const uint8_t PWM_920            = 3;

//  CONF+1 Hysteresis masks & values CONF+1
const uint8_t HYST_MASK			 = 0b00001100;
const uint8_t HYST_OFF           = 0;
const uint8_t AHYST_LSB1         = 1;
const uint8_t AHYST_LSB2         = 2;
const uint8_t HYST_LSB3          = 3;

//   CONF SlowFilter masks & values 
const uint8_t SF_MASK			 = 0b00000011;
const uint8_t SLOW_FILT_16X      = 0;
const uint8_t SLOW_FILT_8X       = 1;
const uint8_t SLOW_FILT_4X       = 2;
const uint8_t SLOW_FILT_2X       = 3;

//  CONF FastFilter masks & values
const uint8_t FTH_MASK			 = 0b00011100;
const uint8_t FAST_FILT_NONE     = 0;
const uint8_t FAST_FILT_LSB6     = 1;
const uint8_t FAST_FILT_LSB7     = 2;
const uint8_t FAST_FILT_LSB9     = 3;
const uint8_t FAST_FILT_LSB18    = 4;
const uint8_t FAST_FILT_LSB21    = 5;
const uint8_t FAST_FILT_LSB24    = 6;
const uint8_t FAST_FILT_LSB10    = 7;

//  CONF WatchDog masks & values
const uint8_t WD_MASK			 = 0b00100000;
const uint8_t WATCHDOG_OFF       = 0;
const uint8_t WATCHDOG_ON        = 1;

// STATUS masks
const uint8_t MAGNET_HIGH   = 0x08;
const uint8_t MAGNET_LOW    = 0x10;
const uint8_t MAGNET_DETECT = 0x20;

// AGC mask
const uint8_t AGC_MASK				= 0b11111111;

//MAGNITUDE mask
const uint8_t MAGNITUDE_MASK		= 0b00001111;

}