#pragma once
#include <cstdint>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "common/pimoroni_common.hpp"
#include "common/pimoroni_i2c.hpp"

#define TCAADDR 0x70

namespace pimoroni {



  class TCA9548A {
  
    // public:
    //     static const uint8_t DEFAULT_I2C_ADDRESS     = 0x70;
        
    private:
        I2C *i2c;
        uint channels    = 10;

    //---------------------------------------
    //constructor/destructor
    //--------------------------------------
    public:
        TCA9548A(uint channels=8) : TCA9548A(new I2C(),channels) {};
        TCA9548A(I2C *i2c, uint channels = 8) : i2c(i2c), channels(channels) {}

    //--------------------------------------------------
    // Method
    //--------------------------------------------------
    public:
        void tcaselect(uint i);

  };
}



