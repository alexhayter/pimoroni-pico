#include "tca9548a.hpp"
namespace pimoroni {

    void TCA9548A::tcaselect(uint8_t i){
        if (i>7) return;
        uint8_t val = (1 <<i);
        i2c->write_blocking(TCAADDR,&val,1,false);
    }
}
