#include "pico/stdlib.h"
#include "common/pimoroni_common.hpp"

#include "as5600.hpp"

using namespace pimoroni;

  I2C i2c(BOARD::SERVO_2040);
  AS5600 as5600(&i2c);

  int main() {
    stdio_init_all();

    printf("AS5600 Demo\n");
    as5600.init();
    printf("Init done...\n");

 
   //);

    //printf("Gain: C1: %fx C2: %fx C3: %fx\n", reading.gain(1), reading.gain(2), reading.gain(3));

    sleep_ms(1000);
  

    return 0;
  }