#include "tca9548a.hpp"
namespace pimoroni {

    void TCA9548A::tcaselect(uint i){
        if (i>7) return;
        i2c->write_blocking(TCAADDR,(1 <<i),1,false);
    }
}
