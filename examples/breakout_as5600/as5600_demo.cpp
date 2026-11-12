#include "pico/stdlib.h"
#include "common/pimoroni_common.hpp"

#include "as5600.hpp"

using namespace pimoroni;

  I2C i2c(BOARD::BREAKOUT_GARDEN);
  AS5600 as5600(&i2c);

  int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("AS5600 Demo\n");
    as5600.init();
    printf("Init done...\n");
    while (true) {
      printf("rawangle: %d \n",as5600.rawAngle());
      sleep_ms(1000);
    }

    return 0;
  }